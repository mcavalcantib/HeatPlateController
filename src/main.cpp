/**
 * @file main.cpp
 * @author Moisés Cavalcanti (you@domain.com)
 * @brief 
 * @version 0.6
 * @date 2022-05-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "main.h"

/**--------------------------------------------
 * Functions implementarions
 ----------------------------------------------*/
void setup() {
    Interruptions_Configuration();
    pinMode(HEATER_SIGNAL_PIN, OUTPUT);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    // Clear the buffer.
    display.clearDisplay();
    // Display Text "Hello Word"
    display.clearDisplay();
    contador = millis();
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        temperatureArray[i] = round(ThermistorReading());
    }
#if defined(DEBUG_MODE) || defined(PLOT_MODE)
    Serial.begin(9600);
    Serial.println("START");
#endif
}

void loop() {
    if (millis() - contador > (1000 / SAMPLING_FREQUENCY)) {
        contador = millis();
#ifdef DEBUG_MODE
        Serial.print("Temperatura:");
        Serial.println(actualTemperature, BIN);
#endif
        display.clearDisplay();
        display.setTextColor(WHITE);
        // Header
        String textToDisplay = "HEATER: ";
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (isHeatActivated)
            textToDisplay += "ON";
        else
            textToDisplay += "OFF";
        display.println(textToDisplay);
        // Body
        textToDisplay = "";
        display.setCursor(0, 16);
        display.setTextSize(2);
        textToDisplay = String(actualTemperature);
        textToDisplay += "/";
        textToDisplay += String(targetTemperature);
        textToDisplay += "C";
        display.println(textToDisplay);
        display.display();
#if defined(PLOT_MODE) && !defined(DEBUG_MODE)
        Serial.print("PIDValue:");
        Serial.print(PIDValue);
        Serial.print(",");
        Serial.print("Target_temperature:");
        Serial.print(targetTemperature);
        Serial.print(",");
        Serial.print("Current_Temperature:");
        Serial.println(actualTemperature);
#endif
    }
}

/**
 * @brief Timer Interruption function
 *
 */
ISR(TIMER1_COMPA_vect) {
#if defined THERMISTOR_TEMP_SENSOR  // Using Thermiston as temperature sensor
    temperatureArray[movingAveragePosition] = round(ThermistorReading());
    movingAveragePosition = (movingAveragePosition + 1) % MOVING_AVERAGE_SIZE;
    temperatureSum = 0;
    for (int i = 0; i < MOVING_AVERAGE_SIZE; i++) {
        temperatureSum += temperatureArray[i];
    }
    actualTemperature = round(temperatureSum / MOVING_AVERAGE_SIZE);
#elif defined MAX6675_TEMP_SENSOR  // Using Thermocouple and MAX6675 as
                                   // temperature sensor
    actualTemperature = round(thermo.readCelsius());
#else
#error System unable to read temperature. No sensors defined
#endif
    uint8_t output = PID_update(actualTemperature);
    // Thermistor analog reading
    // if (actualTemperature <= 0.8*targetTemperature)
    // {
    //     output = 255;
    // }
    if (!isHeatActivated) output = 0;
#ifdef DEBUG_MODE
    Serial.print("Final: ");
    Serial.println(output);
#endif
    analogWrite(HEATER_SIGNAL_PIN,
                output);  // aciona a resistencia de aquecimento
}

/**
 * @brief Button reading after interruption
 *
 */
ISR(PCINT0_vect) {
    uint8_t changedbits;
    changedbits = PINB ^ portbHistory;  // detecta quem mudou
    portbHistory = PINB;  // salva qual o ultimo estado de cada pino
    if (changedbits & (1 << PINB0)) {
        targetTemperature++;
    }
    if (changedbits & (1 << PINB1) && PINB & (1 << PINB1)) {
        isHeatActivated = !isHeatActivated;
        if (!isHeatActivated) {
            digitalWrite(HEATER_SIGNAL_PIN,
                         LOW);  // desliga a resistencia de aquecimento
        }
    }

    if (changedbits & (1 << PINB2)) {
        targetTemperature--;
    }
}

/**
 * @brief Interruptions configuration including buttons interruption and timer
 * interruption
 */
void Interruptions_Configuration() {
    cli();       // stop interrupts
    TCCR1A = 0;  // set entire TCCR1A register to 0
    TCCR1B = 0;  // same for TCCR1B
    TCNT1 = 0;   // initialize counter value to 0
    // set compare match register for 4 Hz increments
    OCR1A =
        INTERRUPT_REGISTER_VALUE;  // = 16000000 / (64 * 4) - 1 (must be <65536)
    // turn on CTC mode
    TCCR1B |= (1 << WGM12);
    // Set CS12, CS11 and CS10 bits for 64 prescaler
    TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
    // enable timer compare interrupt
    TIMSK1 |= (1 << OCIE1A);

    // limpa os pinos PB0, PB1, PB2 (D8,D9,D10 na placa do arduino)
    DDRB &= ~((1 << DDB0) | (1 << DDB1) | (1 << DDB2));
    // PB0,PB1,PB2 (PCINT0, PCINT1, PCINT2) agora são entradas

    PORTB |=
        ((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2));  // Ativa o Pull-up
    // PB0, PB1 e PB2 agora são entradas com pull-up habilitado

    PCICR |=
        (1 << PCIE0);  // aciona PCIE0 para habilitar o scanneamento do PCMSK0
    PCMSK0 |=
        (1 << PCINT0) | (1 << PCINT1) |
        (1 << PCINT2);  // aciona PCINT0 to trigger an interrupt on state change
    sei();
}

/**
 * @brief Calculate the output value for a given temp reading
 *
 * @param actualValue last temperature reading
 * @return uint8_t output between 0 and 255
 */
uint8_t PID_update(uint16_t actualValue) {
    // e[k] = r[k] - y[k], error between setpoint and true position
    int16_t error = targetTemperature - actualValue;
    // e_d[k] = (e_f[k] - e_f[k-1]) / Tₛ, filtered derivative
    int16_t derivative = round(error / SAMPLING_TIME);
    // e_i[k+1] = e_i[k] + Tₛ e[k], integral
    int16_t new_integral = round(integral + error * SAMPLING_TIME);

    // PID formula:
    // u[k] = Kp e[k] + Ki e_i[k] + Kd e_d[k], control signal
    int16_t control_u = round(CONSTANT_Kp * error + CONSTANT_Ki * integral +
                              CONSTANT_Kd * derivative);
#ifdef DEBUG_MODE
    Serial.print("PID: ");
    Serial.print(control_u);
    Serial.print(" | ");
#endif

    // Clamp the output
    if (control_u > PID_MAXOUTPUT)
        control_u = PID_MAXOUTPUT;
    else if (control_u < 0)
        control_u = 0;
    else  // Anti-windup
        integral = new_integral;
// return the control signal
#ifdef DEBUG_MODE
    Serial.print("Controle: ");
    Serial.println(control_u);
#endif
#ifdef PLOT_MODE
    PIDValue = (uint8_t)control_u;
#endif
    return (uint8_t)control_u;
}

/**
 * @brief Read the temperature in Celsius using a thermistor
 *
 * @return float temperature in Celsius
 */
float ThermistorReading() {
#ifdef THERMISTOR_TEMP_SENSOR
    // Le o sensor algumas vezes
    int V0 = 0;
    V0 = analogRead(THERMISTOR_PIN);
    // Determina a resistência do termistor
    double v = (SYSTEM_ANALOG_VCC * V0) / (1024.0);
    double rt = (SYSTEM_ANALOG_VCC * THERMISTOR_RESISTOR_DIVIDER) / v -
                THERMISTOR_RESISTOR_DIVIDER;

    // Calcula a temperatura
    double t = THERMISTOR_BETA / log(rt / rx);
    return t - 273.0;
#else
    return 0.0f;
#endif
}