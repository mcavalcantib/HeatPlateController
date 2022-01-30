/**
 * @file config.h
 * @author Moisés Cavalcanti (you@domain.com)
 * @brief 
 * @version 0.5
 * @date 2021-12-24
 * 
 * @copyright Copyright (c) 2021
 * 
 */
/** ---------------------------------------
 *  -    System configuration             -
 *  ---------------------------------------
 */

//#define DEBUG_MODE
#define PLOT_MODE
// Temperature sensor type
#define MAX6675_TEMP_SENSOR
//#define THERMISTOR_TEMP_SENSOR

//Display interface
#define SSD1306_OLED

//Controller configurations
#define SAMPLING_FREQUENCY 4 ///< Sampling frequency 

// -------- Pins configuration --------------

// #define COOLER_FAN 3 //WORK IN PROGRESS: FAST COOLING
#define HEATER_SIGNAL_PIN 5

//Sensor pins configuration
#if defined THERMISTOR_TEMP_SENSOR
//Using Thermiston as temperature sensor
#define ThERMISTOR_PIN A1 //A1 - Thermiston pin (if used)
#elif defined MAX6675_TEMP_SENSOR
//Using Thermocouple and MAX6675 as temperature sensor 
#define MAX6675_SCLK 4
#define MAX6675_CS 3
#define MAX6675_MISO 2
#else
//You must have at least one temperature sensor
#error No temperature sensor defined! You must have at least one temperature sensor.
#endif

//Pins configurations
#define BUTTON_TEMP_UP_PINMASK PINB0      // D8 - Temperature UP
#define BUTTON_TURN_ON_OFF_PINMASK PINB1  // D9 - Turn on or off the heating element
#define BUTTON_TEMP_DOWN_PINMASK PINB2    // D10 - Temperature DOWN

// -------- PID configurations ----------
#define PID_MAXOUTPUT 255                   ///< Maximum control output magnitude (change this if your microcontroller has a greater max value)
#define SAMPLING_TIME 1/SAMPLING_FREQUENCY  ///< Sampling time (seconds)
#define CONSTANT_Kp 3                     ///< Proportional gain
#define CONSTANT_Ki 1                       ///< Integral gain times SAMPLING_TIME
#define CONSTANT_Kd 1                       ///< Derivative gain divided by SAMPLING_TIME


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



