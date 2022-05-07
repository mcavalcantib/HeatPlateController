/**
 * @file main.h
 * @author Moisés Cavalcanti (you@domain.com)
 * @brief 
 * @version 0.6
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "config.h"
/** -------------------
 * Variables
 *  ------------------- */
bool dhtActivated = false;
bool isHeatActivated = false;
volatile int targetTemperature = 110;
int16_t actualTemperature = 0;
int16_t PIDValue = 0;
long contador = 0;
int16_t aux;
volatile uint8_t portbHistory = 0xFF;  // padrão é alto por causa do pull-up

int16_t temperatureArray[MOVING_AVERAGE_SIZE], temperatureSum = 0;
uint8_t movingAveragePosition = 0;

#ifdef THERMISTOR_TEMP_SENSOR
const double rx = THERMISTOR_RESISTENCE_VALUE * exp(-THERMISTOR_BETA / 298.15);
#endif
// ---- PID Variables ------

uint16_t activityCount = 0;  ///< How many ticks since last setpoint change.
uint16_t activityThres = 0;  ///< Threshold for turning off the output.
uint8_t errThres = 1;        ///< Threshold with hysteresis.
int32_t integral = 0;        ///< Sum of previous errors for integral.

/** ------------------------------------------
 * Object instances declaration
 * -------------------------------------------- */
Adafruit_SSD1306 display(128, 64, &Wire, -1);
#ifdef MAX6675_TEMP_SENSOR
MAX6675 thermo(MAX6675_SCLK, MAX6675_CS, MAX6675_MISO);
#endif
/** -------------------------------------------
 *  Functions declarations
 ---------------------------------------------- */
void Interruptions_Configuration();
uint8_t PID_update(uint16_t actualValue);
float ThermistorReading();