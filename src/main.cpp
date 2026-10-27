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
MCP23017 mcp_out = MCP23017(0x20);
MCP23017 mcp_in1 = MCP23017(0x24);
MCP23017 mcp_in2 = MCP23017(0x22);
// Кеш стану всіх 16 виходів MCP23017 для уникнення зайвих записів I²C.
uint8_t outCache[16];

#define REGS_SIZE 32
uint8_t holdingRegisters[REGS_SIZE] = {};

// Прапорці "циліндр вже піднятий у поточному циклі" — потрібні,
// щоб кожен цикл підйому виконувався рівно один раз за прохід циклу.
bool colletRetracted = false;
bool rollForm1Retracted = false;
bool rollForm2Retracted = false;

// Момент, коли циліндри були опущені і почалось очікування кінцевиків.
uint32_t retractionStart = 0;

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

// Розкладає байт порту MCP23017 у holdingRegisters (1 біт = 1 регістр).
static inline void unpackPort(uint8_t base, uint8_t portValue) {
  for (uint8_t i = 0; i <= MCP_LAST_PIN; i++) {
    holdingRegisters[base + i] = (portValue >> i) & 1;
  }
}

// Зчитує стан усіх входів з двох MCP23017 у holdingRegisters.
// Кожен порт читається однією I²C-транзакцією (разом 4 замість 28).
//
// Карта регістрів:
//   REG_GPA1 (0..6)   <- MCP IN1: GPA0..GPA6
//   REG_GPB1 (8..14)  <- MCP IN1: GPB0..GPB6
//   REG_GPA2 (16..22) <- MCP IN2: GPA0..GPA6
//   REG_GPB2 (24..30) <- MCP IN2: GPB0..GPB6
void readRegisters() {
  unpackPort(REG_GPA1, mcp_in1.readPort(MCP23017Port::A));
  unpackPort(REG_GPB1, mcp_in1.readPort(MCP23017Port::B));
  unpackPort(REG_GPA2, mcp_in2.readPort(MCP23017Port::A));
  unpackPort(REG_GPB2, mcp_in2.readPort(MCP23017Port::B));
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
  // Serial.print("Motor "); Serial.print(motorIndex);
  // Serial.print("| ADC: "); Serial.print(adcVal);
  // Serial.print("| Interval:"); Serial.println(stepInterval[motorIndex]);
}

// Встановлює фіксовану швидкість мотора (в об/хв), без участі АЦП.
// Використовується для моторів, керованих не потенціометром,
// а константою (наприклад, стіл в автоматичному режимі).
void setMotorSpeedFixed(uint8_t motorIndex, uint16_t rpm) {
  uint32_t freq = ((uint32_t)rpm * STEPS_PER_REV[motorIndex]) / 60;
  uint16_t interval = (F_ISR + freq / 2) / freq;

  noInterrupts();
  stepInterval[motorIndex] = interval;
  interrupts();
}

// Перевіряє, чи дозволено роботу двигунів.
//
// Регістр REG_MOTOR_ENABLE використовує активний низький рівень:
//   LOW  - двигуни увімкнені;
//   HIGH - двигуни вимкнені.
inline bool isMotorsEnabled() {
  return holdingRegisters[REG_MOTOR_ENABLE] == LOW;
}

// Плавне наближення поточного значень АЦП до цільового (Soft Start) для перших двох моторів
uint16_t getSoftStartAdc(uint8_t motorIndex, uint16_t targetAdc) {
  // Мотор столу (індекс 2) — плавний старт не потрібен
  if (motorIndex >= 2) {
    return targetAdc;
  }

  static uint16_t currentAdc[2] = {0, 0};
  static uint32_t lastTick[2]   = {0, 0};

  // Ціль у зоні зупинки означає "гальмуємо до 0", а не миттєвий стоп
  uint16_t goal = isAdcInWorkingRange(targetAdc) ? targetAdc : 0;

  uint32_t now = millis();
  uint32_t ticks = (now - lastTick[motorIndex]) / SOFT_START_TICK_MS;
  if (ticks == 0) {
    return currentAdc[motorIndex];       // ще не час для наступного кроку
  }
  lastTick[motorIndex] = now;
  if (ticks > SOFT_START_MAX_TICKS) ticks = SOFT_START_MAX_TICKS;

  uint32_t cur = currentAdc[motorIndex];
  if (cur < goal) {                      // розгін
    if (cur < SOFT_START_MIN_ADC) cur = SOFT_START_MIN_ADC;
    else                          cur += ticks * SOFT_START_UP_STEP;
    if (cur > goal) cur = goal;
  } else if (cur > goal) {               // гальмування
    uint32_t dec = ticks * SOFT_START_DOWN_STEP;
    cur = (cur > goal + dec) ? (cur - dec) : goal;
  }

  currentAdc[motorIndex] = cur;
  return cur;
}

// Оновлює швидкість всіх двигунів.
//
// Якщо двигуни вимкнені, зупиняє їх.
// Інакше зчитує значення з потенціометрів та плавно наближає
// швидкість кожного двигуна до цільового значення (soft start)
// з кроком у 2 мс для запобігання ривкам.
void updateAllMotorSpeeds() {
  static uint32_t lastAdcUpdate = 0;
  uint16_t rawAdc[MOTOR_COUNT] = {};

  if (isMotorsEnabled()) {
    if (millis() - lastAdcUpdate < ADC_UPDATE_INTERVAL)
      return;
    lastAdcUpdate = millis();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
      rawAdc[i] = adcRead(ADC_CHANNELS[i]);
    }
  }
  Serial.println(1);
  for (uint8_t i = 0; i < MOTOR_COUNT-1; i++) {
    // Пропускаємо через фільтр плавтого розгону перед оновленням швидкості
    uint16_t smoothAdc = getSoftStartAdc(i, rawAdc[i]);
    updateMotorSpeed(i, smoothAdc);
  }
}

// Зупиняє генерацію STEP-імпульсів для всіх трьох моторів,
// встановлюючи їхній інтервал у 0 (ISR ігнорує канал з нульовим
// stepInterval — див. timer1Init/ISR(TIMER1_COMPA_vect)).
// void stopAllMotors() {
//   updateMotorSpeed(MOTOR_ROLL1, getSoftStartAdc(0, 0));
//   updateMotorSpeed(MOTOR_ROLL2, getSoftStartAdc(1, 0));
//   updateMotorSpeed(MOTOR_TABLE, MOTOR_STOP);
// }

inline void readTablePosition() {
    holdingRegisters[IN_TABLE_POSITION] =
        mcp_in1.digitalRead(IN_TABLE_POSITION);
}

// Перевіряє, чи датчик положення столу показує "стіл у вихідній позиції"
// (сигнал LOW). Використовується і в ручному, і в автоматичному режимі
// для визначення моменту зупинки обертання столу.
bool isTableAtPosition() {
  return holdingRegisters[IN_TABLE_POSITION] == LOW;
}

// Обертає стіл до наступного спрацювання датчика положення.
// Спочатку обов'язково чекає, поки стіл зійде з датчика,
// після чого шукає його повторне спрацювання.
//
// Під час обертання максимально часто опитується лише один
// потрібний вхід MCP23017, що забезпечує мінімальну затримку
// зупинки столу.
void rotateTableToHomePosition() {
  setMotorSpeedFixed(MOTOR_TABLE, TABLE_SPEED_800);
  delay(300);
  bool leftHome = false;
  while (true) {
    readTablePosition();
    if (!leftHome) {
      if (!isTableAtPosition()) {
        leftHome = true;
      }
    } else {
      if (isTableAtPosition()) {
        break;
      }
    }
  }

  updateMotorSpeed(MOTOR_TABLE, MOTOR_STOP);
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

// Manual Mode

// Встановлює пару протилежних виходів на основі одного командного входу.
// cmdActive == true  -> outOn=HIGH, outOff=LOW
// cmdActive == false -> outOn=LOW,  outOff=HIGH
void setComplementaryOutputs(uint8_t inCmd, uint8_t outOn, uint8_t outOff) {
  bool cmd = isInputActive(inCmd);
  if (outCache[outOn] == cmd && outCache[outOff] == !cmd)
    return;                        // стан уже потрібний, нічого не робимо
  mcpWriteCached(outOn, cmd);
  delay(100);                      // пауза лише при реальній зміні
  mcpWriteCached(outOff, !cmd);
}

void handleCollet() {
  setComplementaryOutputs(IN_COLLET_CMD, OUT_COLLET_ON, OUT_COLLET_OFF);
}

void handleTableFix() {
  setComplementaryOutputs(IN_TABLE_FIX_CMD, OUT_TABLE_FIX_ON, OUT_TABLE_FIX_OFF);
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
  if (isInputActive(IN_TABLE_ROTATE_CMD)) {
    updateMotorSpeed(MOTOR_TABLE, MOTOR_STOP);
  } else {
    rotateTableToHomePosition();
  }
  handleCollet();
  handleTableFix();
  handleRollForm1();
  handleRollForm2();
}

// Automatic mode

// Перевіряє, чи всі чотири пневмоциліндри (цанга, фіксація столу,
// вальцовка1, вальцовка2) досягли робочого (опущеного) положення.
bool allCylinderLimitsReached() {
  return isInputActive(IN_COLLET_LIMIT) &&
         isInputActive(IN_TABLE_FIX_LIMIT) &&
         isInputActive(IN_ROLL_FORM1_LIMIT) &&
         isInputActive(IN_ROLL_FORM2_LIMIT);
}

// Чекає, поки кінцевик циліндра стане активним (LOW), і друкує результат.
static bool waitCylinderDown(const __FlashStringHelper* name, uint8_t limitReg) {
  uint32_t t0 = millis();
  while (millis() - t0 < CYLINDER_DOWN_TIMEOUT_MS) {
    readRegisters();
    if (isInputActive(limitReg)) {
      Serial.print(name);
      Serial.print(F(" down in "));
      Serial.print(millis() - t0);
      Serial.println(F(" ms"));
      return true;
    }
  }
  Serial.print(name);
  Serial.println(F(" TIMEOUT: limit not reached"));
  return false;
}

// Опускає один циліндр: той самий імпульс ON+OFF, що й раніше,
// потім чекає його нижній кінцевик.
static bool lowerCylinder(const __FlashStringHelper* name,
                          uint8_t outOn, uint8_t outOff,
                          uint16_t pulseMs, uint8_t limitReg) {
  Serial.print(F("Lowering "));
  Serial.println(name);

  mcpWriteCached(outOn,  HIGH);
  mcpWriteCached(outOff, HIGH);
  delay(pulseMs);
  mcpWriteCached(outOn,  HIGH);
  mcpWriteCached(outOff, LOW);

  return waitCylinderDown(name, limitReg);
}

// Опускає всі чотири пневмоциліндри послідовно, з паузою на кожному
// кроці для сповільнення руху (подвійний імпульс ON+OFF -> потім
// зняття OFF), як у перевіреному робочому зразку. Після опускання —
// здув деталі з цанги стисненим повітрям протягом 1 секунди.
void lowerCylinders() {
  lowerCylinder(F("Release (OUT_COLLET)"), OUT_COLLET_ON, OUT_COLLET_OFF, 80, IN_TABLE_FIX_LIMIT); // 4
  lowerCylinder(F("Clamp (OUT_TABLE_FIX)"), OUT_TABLE_FIX_ON, OUT_TABLE_FIX_OFF, 80, IN_COLLET_LIMIT); // 2
  lowerCylinder(F("Roll1"), OUT_ROLL_FORM1_ON, OUT_ROLL_FORM1_OFF, 200, IN_ROLL_FORM1_LIMIT); // 5
  lowerCylinder(F("Roll2"), OUT_ROLL_FORM2_ON, OUT_ROLL_FORM2_OFF, 200, IN_ROLL_FORM2_LIMIT); // 9

  // Serial.println(F("Blow"));
  mcpWriteCached(OUT_BLOW_OFF, HIGH); // здув деталі з цанги
  delay(700);
  mcpWriteCached(OUT_BLOW_OFF, LOW);
}

// Чекає верхній датчик циліндра і друкує результат.
static bool waitCylinderUp(const __FlashStringHelper* name, uint8_t upReg) {
  uint32_t t0 = millis();
  while (millis() - t0 < CYLINDER_UP_TIMEOUT_MS) {
    readRegisters();
    if (isInputActive(upReg)) {
      // Serial.print(name);
      // Serial.print(F(" up in "));
      // Serial.print(millis() - t0);
      // Serial.println(F(" ms"));
      return true;
    }
  }
  // Serial.print(name);
  // Serial.println(F(" TIMEOUT: up sensor not reached"));
  return false;
}

// Піднімає один циліндр: знімає ON, через 100 мс подає OFF,
// потім чекає верхній датчик.
static bool raiseCylinder(const __FlashStringHelper* name,
                          uint8_t outOn, uint8_t outOff, uint8_t upReg) {
  // Serial.print(F("Raising "));
  // Serial.println(name);

  mcpWriteCached(outOn, LOW);
  delay(50);
  mcpWriteCached(outOff, HIGH);

  return waitCylinderUp(name, upReg);
}

void raiseCylinders() {
  raiseCylinder(F("Roll1"), OUT_ROLL_FORM1_ON, OUT_ROLL_FORM1_OFF, IN_ROLL_FORM1_UP); // 6
  raiseCylinder(F("Roll2"), OUT_ROLL_FORM2_ON, OUT_ROLL_FORM2_OFF, IN_ROLL_FORM2_UP); // 8
  raiseCylinder(F("Clamp (OUT_TABLE_FIX)"), OUT_TABLE_FIX_ON, OUT_TABLE_FIX_OFF, IN_COLLET_UP);   // 1
  raiseCylinder(F("Release (OUT_COLLET)"),  OUT_COLLET_ON,    OUT_COLLET_OFF,    IN_TABLE_FIX_UP); // 3
}

// перевірка кінцевика та підйом циліндра
inline void checkAndRetractCylinder(bool &retractedFlag,
                                    uint8_t limitPin,
                                    uint8_t outOn,
                                    uint8_t outOff) {
  if (!retractedFlag && isInputActive(limitPin)) {
    mcpWriteCached(outOn, LOW);
    mcpWriteCached(outOff, HIGH);
    retractedFlag = true;
  }
}

// true, якщо від опускання циліндрів минув захисний таймаут.
inline bool isRetractionTimedOut() {
  return millis() - retractionStart > RETRACTION_TIMEOUT_MS;
}

// Перевіряє умови завершення підйому і завершує цикл.
// Нормальне завершення: цанга і обидві вальцовки підняті та спрацював
// кінцевик фіксації столу.
// Таймаут: примусово піднімаємо ВСІ циліндри, навіть якщо їхній
// кінцевик так і не спрацював.
inline bool checkCompletionAndFinish() {
  bool allOthersDone = colletRetracted && rollForm1Retracted && rollForm2Retracted;
  bool normalFinish  = allOthersDone && isInputActive(IN_TABLE_FIX_LIMIT);

  if (normalFinish) {
    mcpWriteCached(OUT_TABLE_FIX_ON, LOW);
    mcpWriteCached(OUT_TABLE_FIX_OFF, HIGH);
  } else if (isRetractionTimedOut()) {
    raiseCylinders();   // піднімає цангу, вальцовки і фіксатор столу
  } else {
    return false;       // ще чекаємо
  }

  colletRetracted = false;
  rollForm1Retracted = false;
  rollForm2Retracted = false;
  return true;
}

// Обробляє фазу очікування кінцевиків: піднімає цангу, вальцовку1
// і вальцовку2 одразу, щойно спрацював їхній власний кінцевик,
// незалежно одна від одної. Фіксацію столу підіймає лише після
// того, як усі три вже підняті та спрацював кінцевик фіксації столу.
//
// Встановлено таймаут: якщо якийсь кінцевик
// не спрацює через дребезг чи несправність, цикл не зависне назавжди,
// а примусово завершиться й повернеться в очікування старту.
bool handleCylinderRetraction() {
  checkAndRetractCylinder(colletRetracted, IN_COLLET_LIMIT, OUT_COLLET_ON, OUT_COLLET_OFF);
  checkAndRetractCylinder(rollForm1Retracted, IN_ROLL_FORM1_LIMIT, OUT_ROLL_FORM1_ON, OUT_ROLL_FORM1_OFF);
  checkAndRetractCylinder(rollForm2Retracted, IN_ROLL_FORM2_LIMIT, OUT_ROLL_FORM2_ON, OUT_ROLL_FORM2_OFF);

  static uint8_t lastLimit = 255;
  uint8_t cur = holdingRegisters[IN_COLLET_LIMIT];
  if (cur != lastLimit) {
    lastLimit = cur;
    // Serial.print("COLLET_LIMIT=");
    // Serial.println(cur);
  }

  return checkCompletionAndFinish();
}

// Стани автоматичного циклу.
enum AutoCycleState : uint8_t {
  AUTO_WAIT_START,      // очікуємо натискання IN_CYCLE_START
  AUTO_WAIT_LIMITS,     // циліндри опущені, чекаємо всі 4 кінцевики
};

bool autoInitialized = false;

// Автоматичний режим: послідовний цикл.
//   1) Натискання IN_CYCLE_START запускає обертання столу.
//   2) Коли стіл на позиції (IN1==LOW) — опускаються всі 4 циліндри
//      і вмикаються двигуни вальцовки (MOTOR_ROLL1, MOTOR_ROLL2).
//   3) Коли всі кінцевики (IN3,IN5,IN7,IN9) спрацювали — циліндри
//      підіймаються, двигуни вальцовки зупиняються, і цикл повертається
//      в очікування нового старту.
void runAutoMode() {
  static bool prevCycleStartPressed = false;

  if (!autoInitialized) {
    raiseCylinders();
    autoInitialized = true;
  }

  bool cycleStartPressed = isInputActive(IN_CYCLE_START);
  bool cycleStartEdge = cycleStartPressed && !prevCycleStartPressed;
  prevCycleStartPressed = cycleStartPressed;

  if (cycleStartEdge) {
    rotateTableToHomePosition();
    lowerCylinders();
    raiseCylinders();
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
  bool manualMode = isInputActive(IN_MODE_MANUAL);
  bool autoMode   = isAutoMode();

  // Швидкість вальцовок за потенціометрами (з плавним стартом)
  updateMotorSpeed(MOTOR_ROLL1, getSoftStartAdc(0, adcRead(ADC_CH_19)));
  updateMotorSpeed(MOTOR_ROLL2, getSoftStartAdc(1, adcRead(ADC_CH_22)));

  if (manualMode && autoMode) {          // конфлікт режимів
    autoInitialized = false;
    updateMotorSpeed(MOTOR_TABLE, MOTOR_STOP);
  } else if (manualMode) {
    autoInitialized = false;
    runManualMode();
  } else if (autoMode) {
    runAutoMode();
  } else {                               // жоден режим не обраний
    autoInitialized = false;
    updateMotorSpeed(MOTOR_TABLE, MOTOR_STOP);
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

  mcpWriteCached(OUT_BLOW_OFF, LOW);
  
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
