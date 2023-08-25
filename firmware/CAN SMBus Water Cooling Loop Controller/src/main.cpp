#include <Arduino.h>
#include <FanController.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Thermistor.h>
#include <NTC_Thermistor.h>

#define REFERENCE_RESISTANCE 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define INT_FLOW 0      // INPUT Internal loop flow sensor
#define EXT_FLOW 1      // INPUT External loop flow sensor
#define LS_OE 5         // OUTPUT Level shifter output enable, active low
#define PERST 6         // INPUT PCIe #PERST status
#define FP_PWR_OUT 22   // OUTPUT Front panel power switch output
#define FP_PWR_IN 23    // INPUT Front panel power switch input
#define EXT_OUT_TEMP A0 // External loop outflow temperature, 10k NTC
#define EXT_IN_TEMP A1  // External loop inflow temperature, 10k NTC
#define INT_IN_TEMP A2  // Internal loop inflow temperature, 10k NTC
#define INT_OUT_TEMP A3 // Internal loop outflow temperature, 10k NTC
#define WS2811B 7       // OUTPUT WS2811B data
#define SMBUS_SDA 18    // SMBus I2C data
#define SMBUS_SCL 19    // SMBus I2C clock
#define I2C_SDA 30      // Local I2C data
#define I2C_SCL 29      // Local I2C clock
#define CAN_STDBY 2     // OUTPUT CAN standby, active high
#define CAN_TX 3        // CAN transmit
#define CAN_RX 4        // CAN receive
#define PUMP_RPM 8      // INPUT Pump RPM

#define SENSOR_THRESHOLD 1000

FanController pump(PUMP_RPM, SENSOR_THRESHOLD);
FanController int_flow(INT_FLOW, SENSOR_THRESHOLD);
FanController ext_flow(EXT_FLOW, SENSOR_THRESHOLD);

Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Thermistor *ext_out_temp;
Thermistor *ext_in_temp;
Thermistor *int_out_temp;
Thermistor *int_in_temp;

double ext_out_temp_reading;
double ext_in_temp_reading;
double int_out_temp_reading;
double int_in_temp_reading;

unsigned int pump_rpm_reading = 0;
unsigned int int_flow_reading = 0;
unsigned int ext_flow_reading = 0;
unsigned int speed_input = 0;
byte speed_target = 0;
byte speed_actual = 0;

const double flow_coeff = 0.181;
const double flow_intercept = -9.75;

void setup()
{
  // put your setup code here, to run once:
  pinMode(FP_PWR_IN, INPUT_PULLUP);
  pinMode(PERST, INPUT_PULLUP);
  pinMode(LS_OE, OUTPUT);
  pinMode(CAN_STDBY, OUTPUT);

  pump.begin();
  int_flow.begin();
  ext_flow.begin();

  Serial.begin(9600);
  Serial.println("CAN SMBus Water Cooling Loop Controller");

  bme.begin();
  bme_temp->printSensorDetails();
  bme_humidity->printSensorDetails();

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC);
  display.setTextSize(1);              // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);             // Start at top-left corner
  display.cp437(true);                 // Use full 256 char 'Code Page 437' font

  ext_out_temp = new NTC_Thermistor(
      EXT_OUT_TEMP,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
  ext_in_temp = new NTC_Thermistor(
      EXT_IN_TEMP,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
  int_out_temp = new NTC_Thermistor(
      INT_OUT_TEMP,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
  int_in_temp = new NTC_Thermistor(
      INT_IN_TEMP,
      REFERENCE_RESISTANCE,
      NOMINAL_RESISTANCE,
      NOMINAL_TEMPERATURE,
      B_VALUE);
}

void loop()
{
  pump_rpm_reading = pump.getSpeed();
  int_flow_reading = flow_coeff * int_flow.getSpeed() + flow_intercept;
  ext_flow_reading = flow_coeff * ext_flow.getSpeed() + flow_intercept;
  sensors_event_t temp_event, humidity_event;
  bme_temp->getEvent(&temp_event);
  bme_humidity->getEvent(&humidity_event);
  ext_out_temp_reading = ext_out_temp->readCelsius();
  ext_in_temp_reading = ext_in_temp->readCelsius();
  int_out_temp_reading = int_out_temp->readCelsius();
  int_in_temp_reading = int_in_temp->readCelsius();
  Serial.printf("Pump: %d RPM\tInt Flow: %d L/h\tExt Flow: %d L/h\n", pump_rpm_reading, int_flow_reading, ext_flow_reading);
  Serial.printf("BME280: %.1f°C %0.1f%%RH\n", temp_event.temperature, humidity_event.relative_humidity);
  Serial.printf("Internal Loop: In %.1f°C Out %.1f°C\n", int_in_temp_reading, int_out_temp_reading);
  Serial.printf("External Loop: In %.1f°C Out %.1f°C\n", ext_in_temp_reading, ext_out_temp_reading);

  display.clearDisplay();
  display.setCursor(0, 0); // Start at top-left corner
  display.printf("Pump: %d RPM\nInt Flow: %d RPM\nExt Flow: %d RPM\n", pump_rpm_reading, int_flow_reading, ext_flow_reading);
  display.printf("BME280: %.1f°C %0.1f%%RH\n", temp_event.temperature, humidity_event.relative_humidity);
  display.printf("Internal Loop: In %.1f°C Out %.1f°C\n", int_in_temp_reading, int_out_temp_reading);
  display.printf("External Loop: In %.1f°C Out %.1f°C\n", ext_in_temp_reading, ext_out_temp_reading);
  display.display();

  // Not really needed, just avoiding spamming the monitor,
  // readings will be performed no faster than once every THRESHOLD ms anyway
  delay(1000);
}