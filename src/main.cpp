#include "config.h"
/** -------------------
 * Variables
 *  ------------------- */
bool dhtActivated = false;
bool isHeatActivated = false;
volatile int targetTemperature = 40;
float actualTemperature = 0;
long contador = 0;
int16_t aux;
volatile uint8_t portbHistory = 0xFF;  // padrão é alto por causa do pull-up
uint16_t P = 1500, I = 900, D = 800;
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
/**--------------------------------------------
 * Functions implementarions
 ----------------------------------------------*/
void setup() {
    Interruptions_Configuration();
    Serial.begin(9600);
    pinMode(HEATER_SIGNAL_PIN, OUTPUT);
    display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
    // Clear the buffer.
    display.clearDisplay();
    // Display Text "Hello Word"
    display.clearDisplay();
    contador = millis();
    Serial.println("START");
}

void loop() {
    if (millis() - contador > 250) {
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
        Serial.print("Target temperature: ");
        Serial.print(targetTemperature);
        Serial.print(",");
        Serial.print("Current Temperature: ");
        Serial.println(actualTemperature);
        #endif
    }
}

ISR(TIMER1_COMPA_vect) {
    //uint8_t output = 0;
    actualTemperature = round(thermo.readCelsius());
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
    return (uint8_t)control_u;
}
