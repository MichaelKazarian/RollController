/*
 * Контролер керує трьома кроковими двигунами та обробляє сигнали
 * з двох мікросхем MCP23017.
 *
 * Принцип роботи:
 *
 * 1. Два MCP23017 використовуються як модулі цифрових входів.
 *    Усі входи працюють у режимі INPUT_PULLUP, тому активним
 *    вважається низький рівень (LOW).
 *
 * 2. Стан входів періодично зчитується у масив holdingRegisters.
 *    На їх основі формується стан виходів третього MCP23017.
 *    Запис у вихідний MCP23017 виконується через кеш outCache,
 *    що дозволяє уникнути зайвих передач по шині I²C.
 *
 * 3. Швидкість кожного крокового двигуна задається окремим
 *    потенціометром, підключеним до входу АЦП.
 *
 * 4. Значення АЦП перетворюється у швидкість обертання (RPM),
 *    після чого обчислюється інтервал між STEP-імпульсами.
 *    Чим менший інтервал, тим швидше обертається двигун.
 *
 * 5. STEP-імпульси генеруються у перериванні Timer1 з частотою F_ISR.
 *    Для кожного двигуна підтримується власний лічильник та інтервал
 *    генерації імпульсів, що дозволяє незалежно керувати швидкістю
 *    всіх трьох двигунів.
 *
 * 6. Якщо швидкість двигуна встановлена в 0 (stepInterval == 0),
 *    генерація STEP-імпульсів припиняється і двигун зупиняється.
 */

#include <Wire.h>
#include <MCP23017.h>

#define MCP_LAST_PIN 6 // MCP23017 has problematic GPA7/GPB7
#define GPA_SHIFT_B 8

// Карта holdingRegisters
#define REG_GPA1  0   // MCP IN1: GPA0..GPA6
#define REG_GPB1  8   // MCP IN1: GPB0..GPB6
#define REG_GPA2 16   // MCP IN2: GPA0..GPA6
#define REG_GPB2 24   // MCP IN2: GPB0..GPB6

#define MOTOR_COUNT 3
#define ADC_CH_19 6  // фізичний пін 19 -> ADC6 (лише аналоговий, немає цифрового аналога)
#define ADC_CH_22 7  // фізичний пін 22 -> ADC7 (лише аналоговий, немає цифрового аналога)
#define ADC_CH_23 0  // фізичний пін 23 -> ADC0 / PC0 (Arduino A0)
const uint8_t ADC_CHANNELS[] = { ADC_CH_19, ADC_CH_22, ADC_CH_23 };

#define REG_MOTOR_ENABLE 0
#define ADC_MAX_THRESHOLD 1023

#define ADC_UPDATE_INTERVAL 100 // мс
#define ADC_ZERO_THRESHOLD 10
#define RPM_MIN 50
#define RPM_MAX 200
#define F_ISR 50000UL   // частота тіків таймера — впливає на плавність/роздільність
// STEPS_PER_REV — окреме значення для кожного мотора,
// бо мотори/налаштування мікрокроку можуть відрізнятись.
// Заповніть підтвердженими тестом значеннями для кожного мотора.
const uint16_t STEPS_PER_REV[MOTOR_COUNT] = {4000, 4000, 400};

// Створюємо об'єкти для трьох мікросхем
MCP23017 mcp_out = MCP23017(0x20); // Перша (буде 0x20)
MCP23017 mcp_in1 = MCP23017(0x24); // Друга (буде 0x21)
MCP23017 mcp_in2 = MCP23017(0x22);  // Третя (буде 0x22)
// Кеш стану всіх 16 виходів MCP23017 для уникнення зайвих записів I²C.
uint8_t outCache[16];

#define REGS_SIZE 32
uint8_t holdingRegisters[REGS_SIZE] = {};

void gpioInit() {
  // =========================================================================
  // КОНФІГУРАЦІЯ АДРЕСИ ДЛЯ MCP 1 (Порт B: PB2, PB1, PB0)
  // Бажана адреса: 0x20 (бінарно 000 -> всі піни в LOW)
  // =========================================================================
  
  // Спочатку фіксуємо стан LOW (0) в буфері PORTB для бітів 0, 1, 2
  PORTB &= ~((1 << PB2) | (1 << PB1) | (1 << PB0));
  // Переводимо ці піни в режим ВИХОДУ
  DDRB  |=  ((1 << PB2) | (1 << PB1) | (1 << PB0));

  // =========================================================================
  // КОНФІГУРАЦІЯ АДРЕСИ ДЛЯ MCP 2 (Порт D: PD4, PD3, PD2)
  // Бажана адреса: 0x21 (бінарно 001 -> PD4=LOW, PD3=LOW, PD2=HIGH)
  // =========================================================================
  
  // Очищаємо біти 2, 3, 4 в порті D
  PORTD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2));
  // Виставляємо HIGH на біт PD2 (це лінія A0 для MCP 2)
  PORTD |=  (1 << PD2);
  // Переводимо піни PD2, PD3, PD4 в режим ВИХОДУ
  DDRD  |=  ((1 << PD4) | (1 << PD3) | (1 << PD2));

  // =========================================================================
  // КОНФІГУРАЦІЯ АДРЕСИ ДЛЯ MCP 3 (Порт D: PD7, PD6, PD5)
  // Бажана адреса: 0x22 (бінарно 010 -> PD7=LOW, PD6=HIGH, PD5=LOW)
  // =========================================================================
  
  // Очищаємо біти 5, 6, 7 в порті D
  PORTD &= ~((1 << PD7) | (1 << PD6) | (1 << PD5));
  // Виставляємо HIGH на біт PD6 (це лінія A1 для MCP 3)
  PORTD |=  (1 << PD6);
  // Переводимо піни PD5, PD6, PD7 в режим ВИХОДУ
  DDRD  |=  ((1 << PD7) | (1 << PD6) | (1 << PD5));
}

// =========================================================================
// ЗМІННІ ДЛЯ ГЕНЕРАЦІЇ КРОКІВ (ТАЙМЕР 1)
// =========================================================================
volatile uint16_t stepInterval[3] = {0, 0, 0}; // Інтервали у тиках переривання
volatile uint16_t stepCounter[3]  = {0, 0, 0};

// Timer1 працює в режимі CTC і генерує переривання з частотою F_ISR.
// У перериванні формуються STEP-імпульси для всіх трьох двигунів.
// Prescaler = 1, тому OCR1A = F_CPU / F_ISR - 1.
void timer1Init() {
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | (1 << CS10);
  OCR1A = (F_CPU / F_ISR) - 1;
  TIMSK1 |= (1 << OCIE1A);
}

// Обробник переривання Timer1.
// Викликається з частотою F_ISR і генерує STEP-імпульси для трьох
// крокових двигунів.
// На початку кожного виклику всі STEP-виходи переводяться в LOW,
// завершуючи імпульси, сформовані на попередньому такті.
// Для кожного двигуна ведеться власний лічильник stepCounter[].
// Коли він досягає stepInterval[], формується новий STEP-імпульс
// (встановлення виходу в HIGH) і лічильник скидається.
// Якщо stepInterval[i] == 0, двигун вважається зупиненим і
// імпульси для нього не генеруються.
ISR(TIMER1_COMPA_vect) {
  // Закінчуємо імпульси попереднього циклу
  PORTB &= ~((1 << PB3) | (1 << PB4)); PORTC &= ~(1 << PC2);

  for (uint8_t i = 0; i < 3; i++) {
    if (stepInterval[i] == 0)
      continue;
    if (++stepCounter[i] >= stepInterval[i]) {
      stepCounter[i] = 0;
      if (i == 0)
        PORTB |= (1 << PB3);
      else if (i == 1)
        PORTB |= (1 << PB4);
      else
        PORTC |= (1 << PC2);
    }
  }
}

// =========================================================================
// Ініціалізує мікросхеми MCP23017.
// =========================================================================

// Налаштовує вихідний MCP23017.
void initOutputMcp() {
  for (uint8_t i = 0; i <= MCP_LAST_PIN; i++) {
    mcp_out.pinMode(i, OUTPUT);
    mcp_out.pinMode(i + GPA_SHIFT_B, OUTPUT);

    mcp_out.digitalWrite(i, LOW);
    mcp_out.digitalWrite(i + GPA_SHIFT_B, LOW);
  }
}

// Налаштовує обидва MCP23017.
void initInputMcps() {
  for (uint8_t i = 0; i <= MCP_LAST_PIN; i++) {
    mcp_in1.pinMode(i, INPUT_PULLUP);
    mcp_in1.pinMode(i + GPA_SHIFT_B, INPUT_PULLUP);

    mcp_in2.pinMode(i, INPUT_PULLUP);
    mcp_in2.pinMode(i + GPA_SHIFT_B, INPUT_PULLUP);
  }
}

//
// Після встановлення адресних ліній робиться коротка пауза, щоб адреси
// стабілізувалися. Далі:
// - ініціалізується MCP виходів і очищується кеш їхнього стану;
// - усі виходи налаштовуються як OUTPUT і переводяться в LOW;
// - усі входи налаштовуються як INPUT_PULLUP.
void mcpInit() {
  delayMicroseconds(10); // Даємо MCP23017 час зафіксувати стан адресних входів A0..A2
  mcp_out.init();
  for (uint8_t i = 0; i < 14; i++) outCache[i] = LOW;
  initOutputMcp();
  initInputMcps();
}

// Скидає всі регістри керування до значень за замовчуванням (LOW).
void registersInit() {
  memset(holdingRegisters, LOW, sizeof(holdingRegisters));
}

// =========================================================================
// Ініціалізація АЦП.
// =========================================================================
//
// Використовується опорна напруга AVCC і дільник частоти 128,
// що при тактовій частоті 16 МГц дає частоту АЦП 125 кГц —
// рекомендоване значення для отримання максимальної точності.
void adcInit() {
  // Опорна напруга AVCC, вирівнювання праворуч (за замовчуванням)
  ADMUX = (1 << REFS0);
  // Увімкнути АЦП, дільник частоти 128 (16МГц/128 = 125кГц — оптимально для точності)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

// =========================================================================
// Керування системою
//
// Зчитує стан усіх входів з двох MCP23017 у holdingRegisters.
//
// Карта регістрів:
//   REG_GPA1 (0..6)   <- MCP IN1: GPA0..GPA6
//   REG_GPB1 (8..14)  <- MCP IN1: GPB0..GPB6
//   REG_GPA2 (16..22) <- MCP IN2: GPA0..GPA6
//   REG_GPB2 (24..30) <- MCP IN2: GPB0..GPB6
void readRegisters() {
  for (uint8_t i = 0; i <= MCP_LAST_PIN; i++) {
    holdingRegisters[REG_GPA1 + i] = mcp_in1.digitalRead(i);
    holdingRegisters[REG_GPB1 + i] = mcp_in1.digitalRead(i + GPA_SHIFT_B);

    holdingRegisters[REG_GPA2 + i] = mcp_in2.digitalRead(i);
    holdingRegisters[REG_GPB2 + i] = mcp_in2.digitalRead(i + GPA_SHIFT_B);
  }
}

// Записує значення на вихід MCP23017 лише у разі його зміни.
//
// Використовує кеш станів виходів, щоб уникнути зайвих передач по I²C.
// Якщо нове значення збігається з поточним, запис до MCP23017 не виконується.
inline void mcpWriteCached(uint8_t pin, uint8_t value) {
  if (outCache[pin] != value) {
    outCache[pin] = value;
    mcp_out.digitalWrite(pin, value);
  }
}

// Повертає true, якщо вхідний регістр перебуває в активному стані.
//
// Входи MCP23017 налаштовані як INPUT_PULLUP, тому активним
// вважається низький рівень (LOW).
inline bool isInputActive(uint8_t reg) {
  return holdingRegisters[reg] == LOW;
}

// Формує стан виходів MCP23017 на основі вхідних регістрів.
//
// Для кожної лінії вихід активується, якщо активний хоча б один
// із відповідних входів двох вхідних MCP23017.
//
// Запис у вихідний MCP23017 виконується через кеш, щоб уникнути
// зайвих передач по I²C.
void writeRegisters() {
  for (uint8_t i = 0; i <= MCP_LAST_PIN; i++) {

    bool outA =
      isInputActive(REG_GPA1 + i) ||
      isInputActive(REG_GPA2 + i);

    bool outB =
      isInputActive(REG_GPB1 + i) ||
      isInputActive(REG_GPB2 + i);

    mcpWriteCached(i, outA);
    mcpWriteCached(i + GPA_SHIFT_B, outB);
  }
}

void printRegisters() {
  for (uint8_t i=0; i< REGS_SIZE; i++) {
    if (i == 16) Serial.print("    ");
    Serial.print(holdingRegisters[i]);
  }
  Serial.println("");
  delay(100);
}

// Зчитує значення з вказаного каналу АЦП.
// channel - номер каналу АЦП (0..7).
// Після перемикання каналу робиться коротка пауза для стабілізації
// вхідного сигналу, після чого запускається одне перетворення.
// Повертає 10-бітний результат (0..1023).
uint16_t adcRead(uint8_t channel) {
  // MUX[3:0] обирає канал 0..7, зберігаємо біти REFS[1:0]
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  delayMicroseconds(10);
  ADCSRA |= (1 << ADSC);        // Запуск перетворення
  while (ADCSRA & (1 << ADSC)); // Чекаємо завершення перетворення
  return ADC; // результат 0..1023
}

// Оновлює швидкість мотора на основі вже зчитаного значення АЦП.
//
// motorIndex - індекс двигуна (0..2).
// adcVal     - значення АЦП (0..1023) відповідного потенціометра.
//              Якщо adcVal менше ADC_ZERO_THRESHOLD, швидкість
//              встановлюється в 0 і генерація STEP-імпульсів
//              припиняється.
void updateMotorSpeed(uint8_t motorIndex, uint16_t adcVal) {
  uint16_t interval = 0;
  if (adcVal >= ADC_ZERO_THRESHOLD) {
    uint16_t rpm =
      RPM_MIN +
      ((uint32_t)(adcVal - ADC_ZERO_THRESHOLD) *
       (RPM_MAX - RPM_MIN)) /
      (1023 - ADC_ZERO_THRESHOLD);
    uint32_t freq = ((uint32_t)rpm * STEPS_PER_REV[motorIndex]) / 60;
    interval = (F_ISR + freq / 2) / freq; // округлення
  }
  noInterrupts();
  stepInterval[motorIndex] = interval;
  interrupts();
  //  Serial.print("Motor "); Serial.print(motorIndex);
  //  Serial.print("| ADC: "); Serial.print(adcVal);
  //  Serial.print("| Interval:"); Serial.println(stepInterval[motorIndex]);
}

// Перевіряє, чи дозволено роботу двигунів.
//
// Регістр REG_MOTOR_ENABLE використовує активний низький рівень:
//   LOW  - двигуни увімкнені;
//   HIGH - двигуни вимкнені.
inline bool isMotorsEnabled() {
  return holdingRegisters[REG_MOTOR_ENABLE] == LOW;
}

// Оновлює швидкість всіх двигунів.
//
// Якщо двигуни вимкнені, зупиняє їх.
// Інакше не частіше ніж раз на ADC_UPDATE_INTERVAL
// зчитує потенціометри та оновлює швидкість кожного двигуна.
void updateAllMotorSpeeds() {
  static uint32_t lastAdcUpdate = 0;
  uint16_t adc[MOTOR_COUNT] = {};
  if (isMotorsEnabled()) {
    if (millis() - lastAdcUpdate < ADC_UPDATE_INTERVAL)
      return;
    lastAdcUpdate = millis();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) adc[i] = adcRead(ADC_CHANNELS[i]);
  }
  for (uint8_t i = 0; i < MOTOR_COUNT; i++) updateMotorSpeed(i, adc[i]);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  registersInit();
  gpioInit();
  mcpInit();
  adcInit();

  // Налаштування пінів STEP на вихід через регістри напрямку (DDR)
  DDRB |= (1 << PB3) | (1 << PB4); // Налаштовуємо PB3 та PB4 як OUTPUT
  DDRC |= (1 << PC2);              // Налаштовуємо PC2 як OUTPUT

  // Запуск таймера крокових двигунів
  noInterrupts();
  timer1Init();
  interrupts();
}

void loop() {
  readRegisters();
  printRegisters();
  writeRegisters();
  updateAllMotorSpeeds();
}
