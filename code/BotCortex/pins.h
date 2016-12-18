#define LED_PIN 13

#define PIBOT_PULSE_WIDTH_US 1				// pulse width for PiBot drivers
#define PIN_RX1 0
#define PIN_TX1 1

#define PIN_RX2 9
#define PIN_TX2 10

#define PIN_RX3 7
#define PIN_TX3 8

#define PIN_SDA0 18
#define PIN_SCL0 19
#define PIN_SDA1 38
#define PIN_SCL1 37

#define KNOB_PIN 15

/* Command line via Serial1 */
#define SERIAL_CMD_RX PIN_RX1
#define SERIAL_CMD_TX PIN_TX1

/* logging output via Serial2 (actually only TX is needed) */
#define SERIAL_LOG_RX PIN_RX2
#define SERIAL_LOG_TX PIN_TX2

/* Herkulex Servos are connected via Serial3 */
#define HERKULEX_RX PIN_RX3
#define HERKULEX_TX PIN_TX3

#define SENSOR0_SDA PIN_SDA0
#define SENSOR0_SCL PIN_SCL0
#define SENSOR1_SDA PIN_SDA1
#define SENSOR1_SCL PIN_SCL1

#define SERIAL_LOG_RX PIN_RX2
#define SERIAL_LOG_TX PIN_TX2

#define STEPPER0_EN_PIN 	2
#define STEPPER0_DIR_PIN 	3
#define STEPPER0_CLK_PIN 	4
