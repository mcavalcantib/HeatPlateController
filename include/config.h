/**
 * @file config.h
 * @author Moisés Cavalcanti (you@domain.com)
 * @brief 
 * @version 0.6
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */

/** ---------------------------------------
 *  -    System configuration             -
 *  ---------------------------------------
 */

/* ----------- Operation modes ---------- */
//#define DEBUG_MODE
#define PLOT_MODE
// Temperature sensor type
//#define MAX6675_TEMP_SENSOR
#define THERMISTOR_TEMP_SENSOR

/* ---------- Grafical interfaces --------*/
#define SSD1306_OLED

/* ------ Temperature control configurations ------*/
#define SAMPLING_FREQUENCY 10 ///< Sampling frequency 
// - PID configurations 
#define PID_MAXOUTPUT 255                   ///< Maximum control output magnitude (change this if your microcontroller has a greater max value)
#define SAMPLING_TIME 1/SAMPLING_FREQUENCY  ///< Sampling time (seconds)
#define CONSTANT_Kp 4.0f                     ///< Proportional gain
#define CONSTANT_Ki 0.0025f                       ///< Integral gain times SAMPLING_TIME
#define CONSTANT_Kd 20.0f                       ///< Derivative gain divided by SAMPLING_TIME
#define MOVING_AVERAGE_SIZE 10

/* -------- Sensors configuration ----------------- */

#if defined THERMISTOR_TEMP_SENSOR //Using Thermiston as temperature sensor
#define THERMISTOR_PIN A7 //A1 - Thermiston pin (if used)
#define THERMISTOR_RESISTOR_DIVIDER 50000
#define THERMISTOR_RESISTENCE_VALUE 99600
#define THERMISTOR_BETA 3950      // K
#define THERMISTOR_VCC 5.0f    //Supply voltage
#define SYSTEM_ANALOG_VCC 5.0f    //Supply voltage
#define THERMISTOR_CONSTANT1 1.009249522e-03
#define THERMISTOR_CONSTANT2 2.378405444e-04
#define THERMISTOR_CONSTANT3 2.019202697e-07

#elif defined MAX6675_TEMP_SENSOR //Using Thermocouple and MAX6675 as temperature sensor 
#define MAX6675_SCLK 4
#define MAX6675_CS 3
#define MAX6675_MISO 2
#else
//You must have at least one temperature sensor
#error No temperature sensor defined! You must have at least one temperature sensor.
#endif

// -------- Pins configuration --------------

// #define COOLER_FAN 3 //WORK IN PROGRESS: FAST COOLING
#define HEATER_SIGNAL_PIN 5

//Pins configurations
#define BUTTON_TEMP_UP_PINMASK PINB0      // D8 - Temperature UP
#define BUTTON_TURN_ON_OFF_PINMASK PINB1  // D9 - Turn on or off the heating element
#define BUTTON_TEMP_DOWN_PINMASK PINB2    // D10 - Temperature DOWN

/// ------------ Others defines ----------------
#define INTERRUPT_REGISTER_VALUE (16000000 / (64 * SAMPLING_FREQUENCY) - 1) //TODO prescaler configure

/** ------------------------------------
 -             Includes                -
 ---------------------------------------
 */
#include <math.h>
#include <Arduino.h>
#if defined SSD1306_OLED
//Adafruit SSD1306 library dependences.
//If you are using VScode with PlatformIO be sure that you have 
// this libraries in platformio.ini library depencies
// eg.:
//lib_deps = 
//	adafruit/Adafruit GFX Library @ ^1.10.12
//	adafruit/Adafruit SSD1306 @ ^2.5.0
//	adafruit/MAX6675 library@^1.1.0
#include <Adafruit_GFX.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_SSD1306.h>
#endif
#include <Wire.h>
#ifdef MAX6675_TEMP_SENSOR
#include <max6675.h>  // Biblioteca de controle
#endif




