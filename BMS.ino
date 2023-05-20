#include <Adafruit_MCP3008.h>
//#include <LowPower.h>

//pins that connect to the adc
#define CLOCK_PIN 13
#define MISO_PIN 12
#define MOSI_PIN 11
#define CS_PIN 10

// physical values
const float ARDUINO_REFERENCE_VOLTAGE = 3.3;
const float MAXIMUM_VOLTAGE_RATIO = 1023.0;

const float VOLTAGE_LOWER_LIMIT = 3.7;
const float VOLTAGE_UPPER_LIMIT = 4.2;

const float TEMPERATURE_UPPER_LIMIT = 60; //TODO

const int NUM_OF_BATTERIES = 6;

const float VOLTAGE_PER_AMPERE = 0.040;  // 40 mV/A
const float VOLTAGE_OFFSET = 1.65;        // Centered at 1.65V

// voltage divider definitions, first is for first cell
const float CELL_DIVIDER_RATIO [] = {2.0, 3.0, 4.0, 5.0, 6.0, 7.0};

// pinouts
const int CURRENT_READ_CHANNEL = 1;

const int THERMISTOR_PINS [] = {A1, A0, A4, A5, A3, A2}; // TODO

const int WAKEUP_PIN = 7;
const int CHARGING_ENABLE_PIN = 6;

const int DISCHARGING_SWITCH = 8; // Q2
const int CHARGING_SWITCH = 9; // Q1

const int HIGH_NEGATIVE_CURRENT = -35;
const int HIGH_POSITIVE_CURRENT = 10;

// thermistor parameters
const int SERIES_RESISTOR = 100000;     // value of series resistor
const int BCOEF = 3950;                 // beta coefficient of the thermistor
const int THERMISTOR_NOMINAL = 100000;  // resistance at 25 degrees C
const int TEMP_NOMINAL = 25;            // temp. for nominal resistance

const float KELVIN_TO_CELSIUS = 273.15;

const float ADC_VOLTAGE_RATIO = ARDUINO_REFERENCE_VOLTAGE / MAXIMUM_VOLTAGE_RATIO; //convert to real voltage

// init voltage difference values
int cell_voltage_read[NUM_OF_BATTERIES];
float cell_voltage_adjusted[NUM_OF_BATTERIES];
int cell_temperature[NUM_OF_BATTERIES];
float current;

// values that will decide the output
bool low_voltage = false;
bool high_voltage = false;
bool high_temperature = false;
bool high_current = false;
bool charging_enabled = false;
bool discharging_enabled = false;

// adc driver initialization
Adafruit_MCP3008 adc;

void setup() {
  Serial.begin(9600);   // baud rate, could be changed
  //pinMode(WAKEUP_PIN, INPUT_PULLUP); // Configure the pin as input with pull-up resistor
  adc.begin();
  pinMode(CHARGING_ENABLE_PIN, INPUT_PULLUP); 
  pinMode(A1, INPUT);
  pinMode(A0, INPUT); 
  pinMode(A4, INPUT);
  pinMode(A5, INPUT); 
  pinMode(A3, INPUT); 
  pinMode(A2, INPUT); 
  pinMode(CHARGING_SWITCH, OUTPUT);
  pinMode(DISCHARGING_SWITCH, OUTPUT);
}

void loop() {
  // Arduino sleep program
  // If the pin is LOW, put the Arduino to sleep
  //if (digitalRead(WAKEUP_PIN) == LOW) {
  //  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);
  //}
  
  //check if charging is enabled
  charging_enabled = digitalRead(CHARGING_ENABLE_PIN) == HIGH ? true : false;
 
  // get real voltage of each battery pack
  for (int i = 0; i < NUM_OF_BATTERIES; i++) {
    int channel = (NUM_OF_BATTERIES + 1) - i; // Cell 1 is mapped to channel 7 in our example
    cell_voltage_read[i] = adc.readADC(channel); // read the value from channel
    // read voltage and multiply with divider ratio and adc convertion ratio
    cell_voltage_adjusted[i] = cell_voltage_read[i] * CELL_DIVIDER_RATIO[i] * ADC_VOLTAGE_RATIO; 
  }

  // check if voltage of each battery is within the limit
  for (int i = 0; i < NUM_OF_BATTERIES; i++) {
    if (cell_voltage_adjusted[i] < VOLTAGE_LOWER_LIMIT){
      low_voltage = true;
    }
    if (cell_voltage_adjusted[i] > VOLTAGE_UPPER_LIMIT){
      high_voltage = true;
    }
  }
  
  // check temperature of each battery
  for (int i = 0; i < NUM_OF_BATTERIES; i++) {
    float temperature_read = analogRead(THERMISTOR_PINS[i]);
    // Calculate the resistance of the thermistor 
    double resistance = SERIES_RESISTOR / ((MAXIMUM_VOLTAGE_RATIO / temperature_read) - 1);
    float temp;
    temp = resistance / THERMISTOR_NOMINAL;           // (R/Ro)
    temp = log(temp);                                 // ln(R/Ro)
    temp /= BCOEF;                                    // 1/B * ln(R/Ro)
    temp += 1.0 / (TEMP_NOMINAL + KELVIN_TO_CELSIUS); // + (1/To)
    temp = 1.0 / temp;                                // Invert
    temp -= KELVIN_TO_CELSIUS;                        // convert from Kelvin to Celsius
    // check if temperature is safe
    //if (temp > TEMPERATURE_UPPER_LIMIT) {
    //  high_temperature = true;
    //}
  }

  // check the current
  int current_voltage_read = adc.readADC(CURRENT_READ_CHANNEL);     // read the value from channel, will be between 0 and 1023
  current_voltage_read = current_voltage_read * ADC_VOLTAGE_RATIO;  // Convert ADC value to voltage
  current = (current_voltage_read - VOLTAGE_OFFSET) / VOLTAGE_PER_AMPERE;
  // check if current is within the safe limits
  if (current <= HIGH_NEGATIVE_CURRENT){
    high_current = true;
  } else if (current >= HIGH_POSITIVE_CURRENT){
    high_current = true;
  }

/*  if (high_temperature) {
    Serial.print("H.t ");
  }
  if (high_current) {
    Serial.println(current);
  }
  if (low_voltage) {
    //Serial.print("L.v");
  }
  if (high_voltage) {
    Serial.print("H.v");
  }
*/

  // make decision
  // if any of the conditions apllies, disable the battery
  if (high_temperature || high_current || low_voltage || high_voltage) { 
    digitalWrite(CHARGING_SWITCH, LOW);
    digitalWrite(DISCHARGING_SWITCH, LOW);
  } else if (charging_enabled) {
    digitalWrite(CHARGING_SWITCH, HIGH); 
    digitalWrite(DISCHARGING_SWITCH, LOW);
  } else {
    digitalWrite(CHARGING_SWITCH, LOW); 
    digitalWrite(DISCHARGING_SWITCH, HIGH);
  }
  delay(100); // 100ms interval
}
