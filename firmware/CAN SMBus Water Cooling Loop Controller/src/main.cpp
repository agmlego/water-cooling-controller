#include <Arduino.h>
#include <FanController.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <Thermistor.h>
#include <NTC_Thermistor.h>

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

#define SENSOR_THRESHOLD 1000
FanController int_flow(INT_FLOW, SENSOR_THRESHOLD);
FanController ext_flow(EXT_FLOW, SENSOR_THRESHOLD);
unsigned int int_flow_reading = 0;
unsigned int ext_flow_reading = 0;
const double flow_coeff = 0.181;
const double flow_intercept = -9.75;

#define BME_ADDRESS 0x76
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define FONT_X 5
#define FONT_Y 8
#define GAUGE_RADIUS 28
#define PAGE_DELAY 5000
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define REFERENCE_RESISTANCE 100000
#define NOMINAL_RESISTANCE 100000
#define NOMINAL_TEMPERATURE 25
#define B_VALUE 3950
Thermistor *ext_out_temp;
Thermistor *ext_in_temp;
Thermistor *int_out_temp;
Thermistor *int_in_temp;

double ext_out_temp_reading;
double ext_in_temp_reading;
double int_out_temp_reading;
double int_in_temp_reading;

uint32_t reading_time = 0;
uint8_t reading_state = 0;
bool led_state = 1;

int ringMeter(const char *, int, int, int, int, int, int, const char *);

void setup()
{
    pinMode(FP_PWR_IN, INPUT_PULLUP);
    pinMode(PERST, INPUT_PULLUP);
    pinMode(LS_OE, OUTPUT);
    pinMode(CAN_STDBY, OUTPUT);
    pinMode(13, OUTPUT);

    int_flow.begin();
    ext_flow.begin();

    Serial.begin(9600);
    while (!Serial && millis() < 5000)
    {
        // wait up to 5 seconds for Arduino Serial Monitor
    }
    Serial.println("CAN SMBus Water Cooling Loop Controller");

    while (!bme.begin(BME_ADDRESS))
    {
        Serial.println("No connect to BME!");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        digitalWrite(LED_BUILTIN, LOW);
        delay(100);
    }
    bme_temp->printSensorDetails();
    bme_humidity->printSensorDetails();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    while (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        Serial.println("No connect to display!");
        digitalWrite(LED_BUILTIN, HIGH);
        delay(250);
        digitalWrite(LED_BUILTIN, LOW);
        delay(250);
    }
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
    digitalWrite(13, LOW);
    sensors_event_t temp_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_humidity->getEvent(&humidity_event);
    int_flow_reading = flow_coeff * int_flow.getSpeed() + flow_intercept;
    ext_flow_reading = flow_coeff * ext_flow.getSpeed() + flow_intercept;
    int_out_temp_reading = int_out_temp->readCelsius();
    int_in_temp_reading = int_in_temp->readCelsius();
    ext_out_temp_reading = ext_out_temp->readCelsius();
    ext_in_temp_reading = ext_in_temp->readCelsius();
    if (millis() - reading_time >= PAGE_DELAY)
    {
        reading_time = millis();
        ++reading_state;
        if (reading_state >= 4)
            reading_state = 0;
        switch (reading_state)
        {
        case 0:
            display.clearDisplay();
            ringMeter("Case T", temp_event.temperature, 0, 100, 0, 0, GAUGE_RADIUS, "\xF8""C");
            ringMeter("Case RH", humidity_event.relative_humidity, 0, 100, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "%");
            display.display();
            break;
        case 1:
            display.clearDisplay();
            ringMeter("Int Flow", int_flow_reading, 0, 300, 0, 0, GAUGE_RADIUS, "L/h");
            ringMeter("Ext Flow", ext_flow_reading, 0, 300, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "L/h");
            display.display();
            break;
        case 2:
            display.clearDisplay();
            ringMeter("Int In", int_in_temp_reading, 0, 100, 0, 0, GAUGE_RADIUS, "\xF8""C");
            ringMeter("Int Out", int_out_temp_reading, 0, 100, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "\xF8""C");
            display.display();
            break;
        case 3:
            display.clearDisplay();
            ringMeter("Ext In", ext_in_temp_reading, 0, 100, 0, 0, GAUGE_RADIUS, "\xF8""C");
            ringMeter("Ext Out", ext_out_temp_reading, 0, 100, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "\xF8""C");
            display.display();
            break;
        default:
            break;
        }
    }
}

// #########################################################################
//  Draw the meter on the screen, returns x coord of righthand side
// #########################################################################
int ringMeter(const char *reading, int value, int vmin, int vmax, int orig_x, int orig_y, int r, const char *units)
{
    int x = orig_x + r;
    int y = orig_y + r + FONT_Y; // Calculate coords of centre of ring

    int w = r / 4; // Width of outer ring is 1/4 of radius

    int angle = 150; // Half the sweep angle of meter (300 degrees)

    int v = map(value, vmin, vmax, -angle, angle); // Map the value to an angle v

    byte seg = 5; // Segments are 5 degrees wide = 60 segments for 300 degrees
    byte inc = 5; // Draw segments every 5 degrees, increase to 10 for segmented ring

    // Draw colour blocks every inc degrees
    for (int i = -angle; i < angle; i += inc)
    {
        // Calculate pair of coordinates for segment start
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (r - w) + x;
        uint16_t y0 = sy * (r - w) + y;
        uint16_t x1 = sx * r + x;
        uint16_t y1 = sy * r + y;

        // Calculate pair of coordinates for segment end
        float sx2 = cos((i + seg - 90) * 0.0174532925);
        float sy2 = sin((i + seg - 90) * 0.0174532925);
        int x2 = sx2 * (r - w) + x;
        int y2 = sy2 * (r - w) + y;
        int x3 = sx2 * r + x;
        int y3 = sy2 * r + y;

        if (i < v)
        { // Fill in coloured segments with 2 triangles
            display.fillTriangle(x0, y0, x1, y1, x2, y2, SSD1306_WHITE);
            display.fillTriangle(x1, y1, x2, y2, x3, y3, SSD1306_WHITE);
        }
        else // Fill in blank segments
        {
            display.fillTriangle(x0, y0, x1, y1, x2, y2, SSD1306_BLACK);
            display.fillTriangle(x1, y1, x2, y2, x3, y3, SSD1306_BLACK);
        }
    }

    // Convert value to a string
    char buf[10];
    byte len = 1;
    if (value > 9)
        len = 2;
    if (value > 99)
        len = 3;
    if (value > 999)
        len = 4;
    dtostrf(value, len, 0, buf);

    // Set the text colour to default
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // Print reading
    display.setTextSize(1);
    display.setCursor(x - ((FONT_X * strlen(reading)) / 2), orig_y);
    display.print(reading);

    // Print value
    display.setTextSize(2);
    display.setCursor(x - (FONT_X * 2 * len / 2), y - FONT_Y); // Value in middle
    display.print(buf);

    // Print units

    display.setTextSize(1);
    display.setCursor(x - ((FONT_X * strlen(units)) / 2), y + FONT_Y); // Units display
    display.print(units);

    // Calculate and return right hand side x coordinate
    return x + r;
}