#include "config.h"
/** -------------------
 * Variables
 *  ------------------- */
bool dhtActivated = false;
bool isHeatActivated = false;
volatile int targetTemperature = 30;
float actualTemperature = 0;
long contador = 0;
int16_t aux;
volatile uint8_t portbHistory = 0xFF;  // padrão é alto por causa do pull-up
uint16_t P = 1500, I = 900, D = 800;

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
        Serial.print("Temp: ");
        Serial.println(actualTemperature, BIN);
        display.clearDisplay();
        display.setTextColor(WHITE);
        // Header
        String textToDisplay = "HEATER: ";
        display.setTextSize(1);
        display.setCursor(0, 0);
        if (isHeatActivated) textToDisplay += "ON";
        else  textToDisplay += "OFF";
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
    }
}

ISR(TIMER1_COMPA_vect) {
    actualTemperature = thermo.readCelsius();
    // Thermistor analog reading
    if (isHeatActivated) {
        if (actualTemperature < targetTemperature) {
            digitalWrite(HEATER_SIGNAL_PIN,
                         HIGH);  // aciona a resistencia de aquecimento
        } else if (actualTemperature >
                   targetTemperature *
                       0.95) {  // Little compensation before the PID algorithm
            digitalWrite(HEATER_SIGNAL_PIN,
                         LOW);  // desliga a resistencia de aquecimento
        } else {
            digitalWrite(HEATER_SIGNAL_PIN,
                         LOW);  // desliga a resistencia de aquecimento
        }
    } else {
        digitalWrite(HEATER_SIGNAL_PIN,
                     LOW);  // desliga a resistencia de aquecimento
    }
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
    OCR1A = 62499;  // = 16000000 / (64 * 4) - 1 (must be <65536)
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