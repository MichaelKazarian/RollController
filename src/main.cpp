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
#include "RollController.h"

const uint8_t ADC_CHANNELS[] = { ADC_CH_19, ADC_CH_22, ADC_CH_23 };

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

bool x = false;

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

// Перевіряє, чи значення АЦП знаходиться в робочому діапазоні
// (поза "мертвими зонами" зупинки мотора знизу і зверху).
bool isAdcInWorkingRange(uint16_t adcVal) {
  return adcVal >= ADC_ZERO_THRESHOLD && adcVal < ADC_MAX_THRESHOLD;
}

// Оновлює швидкість мотора на основі вже зчитаного значення АЦП.
//
// motorIndex - індекс двигуна (0..2).
// adcVal     - значення АЦП (0..ADC_ZERO_THRESHOLD) відповідного потенціометра
//              Якщо adcVal менше ADC_ZERO_THRESHOLD або дорівнює/більше
//              ADC_MAX_THRESHOLD, швидкість встановлюється в 0 і генерація
//              STEP-імпульсів припиняється.
void updateMotorSpeed(uint8_t motorIndex, uint16_t adcVal) {
  uint16_t interval = 0;
  if (isAdcInWorkingRange(adcVal)) {
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
  Serial.print("Motor "); Serial.print(motorIndex);
  Serial.print("| ADC: "); Serial.print(adcVal);
  Serial.print("| Interval:"); Serial.println(stepInterval[motorIndex]);
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

// Зупиняє генерацію STEP-імпульсів для всіх трьох моторів,
// встановлюючи їхній інтервал у 0 (ISR ігнорує канал з нульовим
// stepInterval — див. timer1Init/ISR(TIMER1_COMPA_vect)).
void stopAllMotors() {
  noInterrupts();
  stepInterval[MOTOR_ROLL1] = 0;
  stepInterval[MOTOR_ROLL2] = 0;
  stepInterval[MOTOR_TABLE] = 0;
  interrupts();
}

// Mode checkers

// Перевіряє, чи система зараз перебуває в автоматичному режимі,
// на основі стану вхідного сигналу holdingRegisters[IN_MODE_AUTO].
bool isAutoMode() {
  return isInputActive(IN_MODE_AUTO);
}

// Перевіряє, чи система зараз перебуває в ручному режимі,
// на основі стану вхідного сигналу holdingRegisters[IN_MODE_MANUAL].
bool isManualMode() {
  return isInputActive(IN_MODE_MANUAL);
}

// Перевіряє, чи датчик положення столу показує "стіл у вихідній позиції"
// (сигнал LOW). Використовується і в ручному, і в автоматичному режимі
// для визначення моменту зупинки обертання столу.
bool isTableAtPosition() {
  return holdingRegisters[IN_TABLE_POSITION] == LOW;
}

// Обертає стіл до спрацювання датчика положення (IN_TABLE_POSITION == LOW).
// Використовується і в автоматичному режимі, і в ручному режимі
// (коли IN_TABLE_ROTATE_CMD == HIGH).
void rotateTableUntilPosition() {
  if (!isTableAtPosition()) {
    updateMotorSpeed(MOTOR_TABLE, 200);
  } else {
    updateMotorSpeed(MOTOR_TABLE, 0);
  }
}
// Manual Mode

// Встановлює пару протилежних виходів на основі одного командного входу.
// cmdActive == true  -> outOn=HIGH, outOff=LOW
// cmdActive == false -> outOn=LOW,  outOff=HIGH
void setComplementaryOutputs(uint8_t inCmd, uint8_t outOn, uint8_t outOff) {
  bool cmd = isInputActive(inCmd);
  mcpWriteCached(outOn, cmd);
  mcpWriteCached(outOff, !cmd);
}

void handleChuckClamp() {
  setComplementaryOutputs(IN_CHUCK_CLAMP_CMD, OUT_CHUCK_CLAMP, OUT_CHUCK_RELEASE);
}

void handleClamp2() {
  setComplementaryOutputs(IN_CLAMP2_CMD, OUT_CLAMP2_ON, OUT_CLAMP2_OFF);
}

void handleRollForm1() {
  setComplementaryOutputs(IN_ROLL_FORM1_CMD, OUT_ROLL_FORM1_ON, OUT_ROLL_FORM1_OFF);
}

void handleRollForm2() {
  setComplementaryOutputs(IN_ROLL_FORM2_CMD, OUT_ROLL_FORM2_ON, OUT_ROLL_FORM2_OFF);
}

// Ручний режим: рулони керуються потенціометрами як завжди.
// Стіл обертається за одним з двох сценаріїв залежно від IN_TABLE_ROTATE_CMD:
//   LOW  — стіл обертається постійно, поки утримується сигнал,
//          незалежно від датчика положення (ручне "прокручування").
//   HIGH — стіл обертається до першого спрацювання датчика положення
//          (той самий принцип зупинки, що і в автоматичному режимі).
void runManualMode() {
  updateMotorSpeed(MOTOR_ROLL1, adcRead(ADC_CH_19));
  updateMotorSpeed(MOTOR_ROLL2, adcRead(ADC_CH_22));

  if (!isInputActive(IN_TABLE_ROTATE_CMD)) {
    updateMotorSpeed(MOTOR_TABLE, 200);
  } else {
    rotateTableUntilPosition();
  }
  handleChuckClamp();
  handleClamp2();
  handleRollForm1();
  handleRollForm2();
}

// Automatic mode

// Перевіряє, чи всі чотири кінцевики пневмоциліндрів спрацювали (LOW).
bool allCylinderLimitsReached() {
  return isInputActive(IN3) && isInputActive(IN5) &&
         isInputActive(IN7) && isInputActive(IN9);
}

// Опускає всі чотири пневмоциліндри (затискачі + вальцовки).
void lowerCylinders() {
  mcpWriteCached(OUT_CHUCK_CLAMP, HIGH);
  mcpWriteCached(OUT_CHUCK_RELEASE, LOW);
  mcpWriteCached(OUT_CLAMP2_ON, HIGH);
  mcpWriteCached(OUT_CLAMP2_OFF, LOW);
  mcpWriteCached(OUT_ROLL_FORM1_ON, HIGH);
  mcpWriteCached(OUT_ROLL_FORM1_OFF, LOW);
  mcpWriteCached(OUT_ROLL_FORM2_ON, HIGH);
  mcpWriteCached(OUT_ROLL_FORM2_OFF, LOW);
}

// Піднімає всі чотири пневмоциліндри (звільняє затискачі + вальцовки).
void raiseCylinders() {
  mcpWriteCached(OUT_CHUCK_CLAMP, LOW);
  mcpWriteCached(OUT_CHUCK_RELEASE, HIGH);
  mcpWriteCached(OUT_CLAMP2_ON, LOW);
  mcpWriteCached(OUT_CLAMP2_OFF, HIGH);
  mcpWriteCached(OUT_ROLL_FORM1_ON, LOW);
  mcpWriteCached(OUT_ROLL_FORM1_OFF, HIGH);
  mcpWriteCached(OUT_ROLL_FORM2_ON, LOW);
  mcpWriteCached(OUT_ROLL_FORM2_OFF, HIGH);
  x = true;
}

// Стани автоматичного циклу.
enum AutoCycleState : uint8_t {
  AUTO_WAIT_START,      // очікуємо натискання IN_CYCLE_START
  AUTO_ROTATE_TABLE,    // стіл обертається до IN1 == LOW
  AUTO_WAIT_LIMITS,     // циліндри опущені, чекаємо всі 4 кінцевики
};


// Автоматичний режим: послідовний цикл.
//   1) Натискання IN_CYCLE_START запускає обертання столу.
//   2) Коли стіл на позиції (IN1==LOW) — опускаються всі 4 циліндри
//      і вмикаються двигуни вальцовки (MOTOR_ROLL1, MOTOR_ROLL2).
//   3) Коли всі кінцевики (IN3,IN5,IN7,IN9) спрацювали — циліндри
//      підіймаються, двигуни вальцовки зупиняються, і цикл повертається
//      в очікування нового старту.
void runAutoMode() {
  static AutoCycleState state = AUTO_WAIT_START;
  static bool prevCycleStartPressed = false;

  bool cycleStartPressed = isInputActive(IN_CYCLE_START);
  bool cycleStartEdge = cycleStartPressed && !prevCycleStartPressed;
  prevCycleStartPressed = cycleStartPressed;

  updateMotorSpeed(MOTOR_ROLL1, adcRead(ADC_CH_19));
  updateMotorSpeed(MOTOR_ROLL2, adcRead(ADC_CH_22));

  switch (state) {
    case AUTO_WAIT_START:
      if (cycleStartEdge) {
        state = AUTO_ROTATE_TABLE;
      }
      break;

    case AUTO_ROTATE_TABLE:
      if (!x && isTableAtPosition()) {
        updateMotorSpeed(MOTOR_TABLE, 0);
        lowerCylinders();
        state = AUTO_WAIT_LIMITS;
      } else {
        updateMotorSpeed(MOTOR_TABLE, 200);
        x = false;
      }
      break;

    case AUTO_WAIT_LIMITS:
      if (allCylinderLimitsReached()) {
        raiseCylinders();
        state = AUTO_WAIT_START;
      }
      break;
  }
}

// Визначає активний режим роботи (ручний / автоматичний) за станом
// вхідних сигналів holdingRegisters[IN_MODE_MANUAL] і [IN_MODE_AUTO],
// і викликає відповідну функцію керування моторами.
//
// Якщо обидва сигнали активні одночасно — трактуємо це як несправність
// (конфлікт режимів, наприклад через обрив/КЗ проводки перемикача) і
// безпечно зупиняємо всі мотори, а не намагаємось вгадати пріоритет.
// Так само зупиняємо мотори, якщо жоден режим не обраний.
void handleOperatingMode() {
  bool manualMode = isInputActive(IN_MODE_MANUAL) ;
  bool autoMode   = isAutoMode();

  if (manualMode && autoMode) {
    stopAllMotors(); // конфлікт режимів — трактуємо як помилку
  } else if (manualMode) {
    runManualMode();
  } else if (autoMode) {
    runAutoMode();
  } else {
    stopAllMotors();
  }
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

  mcpWriteCached(OUT9,  LOW);
  
  // Запуск таймера крокових двигунів
  noInterrupts();
  timer1Init();
  interrupts();
}

void loop() {
  readRegisters();
  // printRegisters();
  //writeRegisters();
  handleOperatingMode();
}
