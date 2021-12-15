#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_I2CDevice.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <max6675.h> // Biblioteca de controle
/**
 * Pin mapping:
 * D8 - Temperature UP
 * D9 - Turn on or off the heating element
 * D10 - Temperature DOWN
 * A1 - Thermiston pin (if used)
 */
/** ----------------------
 * Pins configuration
 * ----------------------- */

#define COOLER_FAN 3
#define HEATER_SIGNAL_PIN 4
#define TERMISTOR_PIN A1
#define BUTTON_TEMP_UP_MASK PINB0 //D8
#define BUTTON_TURN_ON_OFF_MASK PINB1 //D9
#define BUTTON_TEMP_DOWN_MASK PINB2 //10

/** -------------------
 * Variables
 *  ------------------- */
bool dhtActivated = false;
bool isHeatActivated = false;
volatile int targetTemperature = 30;
float actualTemperature = 0;
long contador = 0;
int16_t aux;
volatile uint8_t portbHistory = 0xFF; // padrão é alto por causa do pull-up
uint16_t P=1500, I=900, D=800;

/** ------------------------------------------
 * Object instances declaration
 * -------------------------------------------- */
Adafruit_SSD1306 display(128, 64, &Wire, -1);
MAX6675 thermo(4, 3, 2);
/** -------------------------------------------
 *  Functions declarations 
 ---------------------------------------------- */ 

/**--------------------------------------------
 * Functions implementarions
 ----------------------------------------------*/ 
void setup() {
  cli();//stop interrupts
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; // initialize counter value to 0
  // set compare match register for 4 Hz increments
  OCR1A = 62499; // = 16000000 / (64 * 4) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12, CS11 and CS10 bits for 64 prescaler
  TCCR1B |= (0 << CS12) | (1 << CS11) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // limpa os pinos PB0, PB1, PB2 (D8,D9,D10 na placa do arduino)
  DDRB &= ~((1 << DDB0) | (1 << DDB1) | (1 << DDB2));
  // PB0,PB1,PB2 (PCINT0, PCINT1, PCINT2) agora são entradas

  PORTB |= ((1 << PORTB0) | (1 << PORTB1) | (1 << PORTB2)); // Ativa o Pull-up
  // PB0, PB1 e PB2 agora são entradas com pull-up habilitado

  PCICR |= (1 << PCIE0); // aciona PCIE0 para habilitar o scanneamento do PCMSK0 
  PCMSK0 |= (1 << PCINT0) | (1 << PCINT1) | (1 << PCINT2); // aciona PCINT0 to trigger an interrupt on state change 
  sei(); // turn on interrupts
  Serial.begin(9600);
  pinMode(COOLER_FAN, OUTPUT);
  pinMode(HEATER_SIGNAL_PIN, OUTPUT);
  //pinMode(TERMISTOR_PIN, INPUT);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  // Clear the buffer.
  display.clearDisplay();
  // Display Text "Hello Word"
  display.clearDisplay();
  contador = millis();
  Serial.println("START");
}

void loop() {
  if(millis() - contador >250){
    //actualTemperature = dht.readTemperature();
    actualTemperature = thermo.readCelsius();
    contador = millis();
    Serial.print("Temp: ");
    Serial.println(actualTemperature, BIN);
    display.clearDisplay();
    display.setTextSize(3);
    display.setTextColor(WHITE);
    display.setCursor(0,20);
    String tempTarget = String(actualTemperature, 2);
    tempTarget += "C";
    display.println(tempTarget);
    //tempTarget = String(thermo.readCelsius(), 2);
    //tempTarget += "C";
    //display.setCursor(0,20);
    //display.println(tempTarget);
    //display.setCursor(60, 0);
    //display.println(String(h, 1));
    display.display();
  }
}

ISR(TIMER1_COMPA_vect){
  //Thermistor analog reading
    if (isHeatActivated) {
      
      if (actualTemperature < targetTemperature) {
        digitalWrite(COOLER_FAN, LOW); //desliga o cooler
        digitalWrite(HEATER_SIGNAL_PIN, HIGH); //aciona a resistencia de aquecimento
      } else if (actualTemperature > targetTemperature) {
        digitalWrite(COOLER_FAN, HIGH); //liga o cooler
        digitalWrite(HEATER_SIGNAL_PIN, LOW); //desliga a resistencia de aquecimento
      } else {
        digitalWrite(COOLER_FAN, LOW); //desliga o cooler
        digitalWrite(HEATER_SIGNAL_PIN, LOW); //desliga a resistencia de aquecimento
      }
    }else{
      
    }
}

ISR(PCINT0_vect) {
   uint8_t changedbits;

  changedbits = PINB ^ portbHistory; //detecta quem mudou
  portbHistory = PINB; //salva qual o ultimo estado de cada pino
  if (changedbits & (1 << PINB0)) {
    targetTemperature++;
  }

  if (changedbits & (1 << PINB1)&& PINB & (1<<PINB1)) {
    isHeatActivated = !isHeatActivated;
    if (!isHeatActivated) {
      digitalWrite(COOLER_FAN, LOW); //desliga o cooler
      digitalWrite(HEATER_SIGNAL_PIN, LOW); //desliga a resistencia de aquecimento
    }
  }

  if (changedbits & (1 << PINB2)) {
    targetTemperature--;
  }
}
