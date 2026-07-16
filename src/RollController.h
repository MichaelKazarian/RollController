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

#define ADC_UPDATE_INTERVAL 0 // мс
#define ADC_ZERO_THRESHOLD 10
#define RPM_MIN 50
#define RPM_MAX 200
#define F_ISR 50000UL   // частота тіків таймера — впливає на плавність/роздільність

#define IN_CYCLE_START       12  // IN12: запуск автоматичного циклу (LOW = натиснуто)
// Кінцевики пневмоциліндрів. Призначення кожного окремо поки не уточнене,
// перевіряються всі разом як група.
#define IN3  2
#define IN5  4
#define IN7  5
#define IN9  9
#define IN_TABLE_POSITION     0  // IN1: датчик вихідного положення столу
#define IN_MODE_MANUAL       14  // IN14: ручний режим керування 
#define IN_MODE_AUTO         16  // IN15: автоматичний режим керування
#define IN_TABLE_ROTATE_CMD  17  // IN16: команда обертання столу в ручному режимі
#define IN_CHUCK_CLAMP_CMD   18  // IN17: команда затиснути цангу (1) / розтиснути (0)
#define IN_CLAMP2_CMD   19  // IN18: команда затиснути (другий затискач)
#define IN_ROLL_FORM1_CMD    20  // IN19: команда вальцовка1
#define IN_ROLL_FORM2_CMD    21  // IN20: команда вальцовка2
                                 // ЦАНГА 12, 13
#define OUT_COLLET_ON      13  // OUT3: сигнал "затиснути" (HIGH при затиску)
#define OUT_COLLET_OFF       12  // OUT4: "розтиснути" цангу (HIGH при розтиску)
#define OUT_CLAMP2_ON         0  // OUT1: сигнал "затиснути"
#define OUT_CLAMP2_OFF       14  // OUT2: сигнал "розтиснути"
#define OUT_ROLL_FORM1_ON    11  // OUT5: вальцовка1 "увімкнути"
#define OUT_ROLL_FORM1_OFF   10  // OUT6: вальцовка1 "вимкнути"
#define OUT_ROLL_FORM2_ON     9  // OUT7: вальцовка2 "увімкнути"
#define OUT_ROLL_FORM2_OFF    8  // OUT8: вальцовка2 "вимкнути"
#define OUT9  12

#define TABLE_SPEED 200
#define MOTOR_STOP    0
