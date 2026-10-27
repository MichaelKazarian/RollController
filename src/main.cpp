/*
 * Контролер вальцювального верстата: керує трьома кроковими двигунами
 * (дві вальцовки і поворотний стіл) та пневмоциліндрами через
 * три мікросхеми MCP23017.
 *
 * Апаратна частина:
 *
 * 1. Два MCP23017 (mcp_in1, mcp_in2) — цифрові входи: датчики положення,
 *    кінцевики циліндрів, перемикачі режимів і команди оператора.
 *    Усі входи в режимі INPUT_PULLUP, тому активним вважається LOW
 *    (тобто датчик спрацював).
 *    Третій MCP23017 (mcp_out) — виходи на клапани пневмоциліндрів.
 *    Кожен циліндр має пару виходів ON/OFF. Запис у виходи йде через кеш
 *    outCache, щоб не передавати по I²C незмінені значення.
 *
 * 2. Стан входів зчитується на кожному проході loop() у масив
 *    holdingRegisters (індекс = номер входу з нуля; GPA7/GPB7 не
 *    використовуються). Кожен порт читається однією I²C-транзакцією.
 *
 * Двигуни:
 *
 * 3. STEP-імпульси для всіх трьох двигунів генерує переривання Timer1
 *    з частотою F_ISR. Для кожного двигуна є власний інтервал
 *    stepInterval (у тиках переривання); 0 означає, що двигун стоїть.
 *
 * 4. Швидкість вальцовок задається потенціометрами (АЦП). Значення АЦП
 *    переводиться в об/хв (RPM_MIN..RPM_MAX), потім в інтервал між
 *    STEP-імпульсами. Значення поза робочим діапазоном (нижче
 *    ADC_ZERO_THRESHOLD або від ADC_MAX_THRESHOLD) означає зупинку.
 *
 * 5. Між потенціометром і двигуном стоїть плавний розгін/гальмування за
 *    часом (SOFT_START_*), який не залежить від швидкості loop().
 *    Він працює в усіх режимах, тож перемикання режимів вальцовки не
 *    зупиняє і не смикає.
 *
 * 6. Стіл крутиться з фіксованою швидкістю до наступного спрацювання
 *    датчика вихідного положення (IN_TABLE_POSITION).
 *
 * Режими роботи (handleOperatingMode):
 *
 * 7. Ручний режим: оператор керує циліндрами окремими перемикачами
 *    (парні виходи ON/OFF змінюються лише при зміні команди), а столом
 *    через IN_TABLE_ROTATE_CMD: постійне обертання або до датчика.
 *
 * 8. Автоматичний режим: за фронтом IN_CYCLE_START виконується цикл:
 *    стіл у вихідне положення -> циліндри опускаються послідовно, кожен
 *    із очікуванням свого нижнього кінцевика -> здув деталі з цанги ->
 *    циліндри піднімаються послідовно з очікуванням верхніх датчиків.
 *    Якщо кінцевик не спрацював за таймаут (CYLINDER_DOWN_TIMEOUT_MS /
 *    CYLINDER_UP_TIMEOUT_MS), цикл іде далі, а в Serial виводиться
 *    діагностика.
 *
 * 9. Якщо режим не обраний або обидва режими активні одночасно (можлива
 *    несправність перемикача), зупиняється лише стіл. Вальцовки
 *    продовжують керуватися потенціометрами.
 */

#include <Wire.h>
#include <MCP23017.h>
#include "RollController.h"

// STEPS_PER_REV — окреме значення для кожного мотора,
// бо мотори/налаштування мікрокроку можуть відрізнятись.
const uint16_t STEPS_PER_REV[MOTOR_COUNT] = {4000, 4000, 400};

// Створюємо об'єкти для трьох мікросхем
MCP23017 mcp_out = MCP23017(0x20);
MCP23017 mcp_in1 = MCP23017(0x24);
MCP23017 mcp_in2 = MCP23017(0x22);
// Кеш стану всіх 16 виходів MCP23017 для уникнення зайвих записів I²C.
uint8_t outCache[16];

#define REGS_SIZE 32
uint8_t holdingRegisters[REGS_SIZE] = {};

// Виставляє рівні на адресних лініях A0..A2 трьох MCP23017 до їх ініціалізації.
// Адреса мікросхеми = 0x20 | (A2<<2 | A1<<1 | A0). Піни МК, підключені до A0..A2,
// працюють як виходи й постійно тримають потрібні рівні.
//
//   Група   Піни МК       Об'єкт    Адреса   Рівні на лініях A2 A1 A0
//   MCP 1   PB2,PB1,PB0   mcp_out   0x20     0  0  0
//   MCP 2   PD4,PD3,PD2   mcp_in1   0x24     1  0  0  (HIGH на PD2 = лінія A2)
//   MCP 3   PD7,PD6,PD5   mcp_in2   0x22     0  1  0  (HIGH на PD6 = лінія A1)
void gpioInit() {
  // =========================================================================
  // MCP 1 = mcp_out (виходи на клапани). Піни PB2, PB1, PB0.
  // Адреса 0x20 (000): усі адресні лінії в LOW.
  // =========================================================================
  // Спочатку фіксуємо LOW у PORTB для бітів 0, 1, 2
  PORTB &= ~((1 << PB2) | (1 << PB1) | (1 << PB0));
  // Переводимо ці піни в режим ВИХОДУ
  DDRB  |=  ((1 << PB2) | (1 << PB1) | (1 << PB0));

  // =========================================================================
  // MCP 2 = mcp_in1 (входи, перша група). Піни PD4, PD3, PD2.
  // Адреса 0x24 (100): PD2 = HIGH (лінія A2), PD3 і PD4 = LOW.
  // =========================================================================
  // Очищаємо біти 2, 3, 4 у порті D
  PORTD &= ~((1 << PD4) | (1 << PD3) | (1 << PD2));
  // HIGH на PD2: лінія A2 для MCP 2
  PORTD |=  (1 << PD2);
  // Переводимо PD2, PD3, PD4 в режим ВИХОДУ
  DDRD  |=  ((1 << PD4) | (1 << PD3) | (1 << PD2));

  // =========================================================================
  // MCP 3 = mcp_in2 (входи, друга група). Піни PD7, PD6, PD5.
  // Адреса 0x22 (010): PD6 = HIGH (лінія A1), PD5 і PD7 = LOW.
  // =========================================================================
  // Очищаємо біти 5, 6, 7 у порті D
  PORTD &= ~((1 << PD7) | (1 << PD6) | (1 << PD5));
  // HIGH на PD6: лінія A1 для MCP 3
  PORTD |=  (1 << PD6);
  // Переводимо PD5, PD6, PD7 в режим ВИХОДУ
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
// void updateAllMotorSpeeds() {
//   const uint8_t ADC_CHANNELS[] = { ADC_CH_19, ADC_CH_22, ADC_CH_23 };
//   static uint32_t lastAdcUpdate = 0;
//   uint16_t rawAdc[MOTOR_COUNT] = {};

//   if (isMotorsEnabled()) {
//     if (millis() - lastAdcUpdate < ADC_UPDATE_INTERVAL)
//       return;
//     lastAdcUpdate = millis();
//     for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
//       rawAdc[i] = adcRead(ADC_CHANNELS[i]);
//     }
//   }
//   for (uint8_t i = 0; i < MOTOR_COUNT-1; i++) {
//     // Пропускаємо через фільтр плавтого розгону перед оновленням швидкості
//     uint16_t smoothAdc = getSoftStartAdc(i, rawAdc[i]);
//     updateMotorSpeed(i, smoothAdc);
//   }
// }

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

// Піднімає один циліндр: знімає ON, через 50 мс подає OFF,
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
  bool manualMode = isManualMode();
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
  handleOperatingMode();
}
