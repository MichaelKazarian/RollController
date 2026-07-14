#define MCP_LAST_PIN 6 // MCP23017 has problematic GPA7/GPB7
#define GPA_SHIFT_B 8

// Карта holdingRegisters
#define REG_GPA1  0   // MCP IN1: GPA0..GPA6
#define REG_GPB1  8   // MCP IN1: GPB0..GPB6
#define REG_GPA2 16   // MCP IN2: GPA0..GPA6
#define REG_GPB2 24   // MCP IN2: GPB0..GPB6

// Двигуни
#define MOTOR_COUNT 3
#define MOTOR_ROLL1 0   // мотор рулону 1
#define MOTOR_ROLL2 1   // мотор рулону 2
#define MOTOR_TABLE 2   // мотор столу

#define ADC_CH_19 6  // фізичний пін 19 -> ADC6 (лише аналоговий, немає цифрового аналога)
#define ADC_CH_22 7  // фізичний пін 22 -> ADC7 (лише аналоговий, немає цифрового аналога)
#define ADC_CH_23 0  // фізичний пін 23 -> ADC0 / PC0 (Arduino A0)

#define REG_MOTOR_ENABLE 0
#define ADC_MAX_THRESHOLD 1023

#define ADC_UPDATE_INTERVAL 100 // мс
#define ADC_ZERO_THRESHOLD 10
#define RPM_MIN 50
#define RPM_MAX 200
#define F_ISR 50000UL   // частота тіків таймера — впливає на плавність/роздільність

#define IN_MODE_MANUAL  14 // ручний режим керування IN14
#define IN_MODE_AUTO    16 // автоматичний режим керування IN15
