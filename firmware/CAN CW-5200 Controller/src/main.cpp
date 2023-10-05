#include <Arduino.h>
#include <FreqMeasureMulti.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <RunningAverage.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#include "error_codes.h"

#define TOP_FAN_RPM 5     // INPUT Top fan tach
#define BOTTOM_FAN_RPM 6  // INPUT Bottom fan tach
#define BOTTOM_FAN_PWM 20 // OUTPUT fan PWM signal
#define FLOW_SW 8         // INPUT PULLUP flow switch; low == OK
#define PUMP_RLY 9        // OUTPUT pump
#define VALVE_RLY 10      // OUTPUT valve
#define COMPRESSOR_RLY 11 // OUTPUT compressor
#define ALARMS_RLY 12     // OUTPUT alarms
#define ONE_WIRE 2        // DS18B20 bus
#define I2C_SDA 18        // Local I2C data
#define I2C_SCL 19        // Local I2C clock
#define FILTER_P A0       // Analog input for differential pressure sensor

RunningAverage filterRA(100);
uint16_t filter_high_limit = 500;

#define FAN_SAMPLING_TIME 1000
const float HZ_TO_RPM = 30.0;
uint32_t top_fan_reading = 0;
uint32_t bottom_fan_reading = 0;
uint8_t fan_pwm_level = 0;
FreqMeasureMulti top_fan;
FreqMeasureMulti bottom_fan;
RunningAverage topRA(10);
RunningAverage bottomRA(10);

#define BME_ADDRESS 0x76
Adafruit_BME280 bme; // use I2C interface
Adafruit_Sensor *bme_temp = bme.getTemperatureSensor();
Adafruit_Sensor *bme_humidity = bme.getHumiditySensor();
float case_temperature_reading = 0;
float case_humidity_reading = 0;
uint8_t case_temperature_high_limit = 100;
uint8_t case_temperature_low_limit = 0;
uint8_t case_humidity_high_limit = 80;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define FONT_X 5
#define FONT_Y 8
#define GAUGE_RADIUS 28
#define PAGE_DELAY 5000
#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint32_t reading_time = 0;
uint8_t reading_state = 0;
bool led_state = 1;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
RunningAverage resRA(100);
const double reservoir_area = 31.416;  // mL per mm height
const double reservoir_height = 270.0; // total reservoir height in mm
double reservoir_volume = 0.0;
uint16_t reservoir_volume_low_limit = 5800;

#define TEMPERATURE_PRECISION 9
OneWire oneWire(ONE_WIRE);
DallasTemperature sensors(&oneWire);
DeviceAddress outside_temp = {0x28, 0x4E, 0x6A, 0x45, 0x92, 0x17, 0x02, 0xEC};
DeviceAddress reservoir_temp = {0x28, 0xFF, 0x02, 0x5D, 0xC1, 0x17, 0x05, 0xCB};
float reservoir_temp_reading = 0.0;
float outside_temp_reading = 0.0;
float reservoir_setpoint = 20.0;
float hysteresis = 2.0;
uint8_t reservoir_temp_high_limit = 30;
uint8_t reservoir_temp_low_limit = 5;
uint8_t outside_temp_high_limit = 100;
uint8_t outside_temp_low_limit = 0;
uint32_t last_compressor = 0;
uint32_t last_valve = 0;
uint32_t compressor_time = 0;
uint32_t valve_time = 0;
uint32_t valve_lockout = 30 * 1000;
uint32_t compressor_lockout = 60 * 1000;

uint16_t error_code = 0;
bool running_state = true;

void setError(uint16_t);
void printAddress(DeviceAddress);
int ringMeter(const char *, int, int, int, int, int, int, const char *);

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);

    filterRA.clear(); // explicitly start clean
    pinMode(PUMP_RLY, OUTPUT);
    digitalWrite(PUMP_RLY, LOW);
    pinMode(FLOW_SW, INPUT_PULLUP);
    pinMode(VALVE_RLY, OUTPUT);
    digitalWrite(VALVE_RLY, HIGH);
    pinMode(COMPRESSOR_RLY, OUTPUT);
    digitalWrite(COMPRESSOR_RLY, LOW);
    pinMode(ALARMS_RLY, OUTPUT);
    digitalWrite(ALARMS_RLY, LOW);

    top_fan.begin(TOP_FAN_RPM);
    topRA.clear();
    bottom_fan.begin(BOTTOM_FAN_RPM);
    bottomRA.clear();
    pinMode(BOTTOM_FAN_PWM, OUTPUT);
    analogWriteFrequency(BOTTOM_FAN_PWM, 25000);
    analogWrite(BOTTOM_FAN_PWM, fan_pwm_level);

    Serial.begin(9600);
    while (!Serial && millis() < 5000)
    {
        // wait up to 5 seconds for Arduino Serial Monitor
    }
    Serial.println("CAN CW_5200 Controller");

    if (!bme.begin(BME_ADDRESS))
    {
        setError(CASE_BME280_NO_CONNECT);
        Serial.printf("Error %04X: No connect to BME!", error_code);
    }
    bme_temp->printSensorDetails();
    bme_humidity->printSensorDetails();

    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    {
        setError(CASE_DISPLAY_NO_CONNECT);
        Serial.printf("Error %04X: No connect to display!", error_code);
    }
    display.setTextSize(1);              // Normal 1:1 pixel scale
    display.setTextColor(SSD1306_WHITE); // Draw white text
    display.setCursor(0, 0);             // Start at top-left corner
    display.cp437(true);                 // Use full 256 char 'Code Page 437' font

    if (!lox.begin())
    {
        setError(RESERVOIR_VL53L0X_NO_CONNECT);
        Serial.printf("Error %04X: Failed to boot VL53L0X", error_code);
    }
    lox.startRangeContinuous();
    resRA.clear();

    sensors.begin();
    if (!sensors.getAddress(outside_temp, 0))
    {
        setError(CASE_NO_OUTSIDE_DS18B20_ADDRESS);
        Serial.printf("Error %04X: Unable to find address for outside_temp", error_code);
    }
    else
    {
        Serial.print("outside_temp Address: ");
        printAddress(outside_temp);
        Serial.println();
    }

    if (!sensors.getAddress(reservoir_temp, 1))
    {
        setError(RESERVOIR_NO_DS18B20_ADDRESS);
        Serial.printf("Error %04X: Unable to find address for reservoir_temp", error_code);
    }
    else
    {
        Serial.print("reservoir_temp Address: ");
        printAddress(reservoir_temp);
        Serial.println();
    }

    sensors.setResolution(reservoir_temp, TEMPERATURE_PRECISION);
    sensors.setResolution(outside_temp, TEMPERATURE_PRECISION);
}

void loop()
{
    digitalWrite(LED_BUILTIN, LOW);

    /*
     *   Reservoir Level Measurement
     */
    if (lox.isRangeComplete())
    {
        reservoir_volume = (reservoir_height - lox.readRange()) * reservoir_area;
        resRA.addValue(reservoir_volume);
        if (resRA.getAverage() < reservoir_volume_low_limit)
        {
            setError(RESERVOIR_LEVEL_LOW);
            Serial.printf("Error %04X: Reservoir level too low! %dmL < %dmL\n", error_code, (int)resRA.getAverage(), reservoir_volume_low_limit);
        }
    }

    /*
     *   Case Temp and RH Measurement
     */
    sensors_event_t temp_event, humidity_event;
    bme_temp->getEvent(&temp_event);
    bme_humidity->getEvent(&humidity_event);
    if (temp_event.temperature > case_temperature_high_limit)
    {
        setError(CASE_TEMP_TOO_HIGH);
        Serial.printf("Error %04X: Case temperature too high! %dC > %dC\n", error_code, temp_event.temperature, case_temperature_high_limit);
    }
    if (temp_event.temperature < case_temperature_low_limit)
    {
        setError(CASE_TEMP_TOO_LOW);
        Serial.printf("Error %04X: Case temperature too low! %dC < %dC\n", error_code, temp_event.temperature, case_temperature_low_limit);
    }
    if (humidity_event.relative_humidity > case_humidity_high_limit)
    {
        setError(CASE_HUMIDITY_TOO_HIGH);
        Serial.printf("Error %04X: Case humidity too high! %d%% > %d%%\n", error_code, humidity_event.relative_humidity, case_humidity_high_limit);
    }

    /*
     *   Reservoir Temp Measurement
     */
    sensors.requestTemperatures();
    reservoir_temp_reading = sensors.getTempC(reservoir_temp);
    if (reservoir_temp_reading == DEVICE_DISCONNECTED_C)
    {
        setError(RESERVOIR_NO_DS18B20_READ);
        Serial.printf("Error %04X: Could not read reservoir temperature data", error_code);
        reservoir_temp_reading = 0.0;
    }
    if (reservoir_temp_reading > reservoir_temp_high_limit)
    {
        setError(RESERVOIR_TEMP_TOO_HIGH);
        Serial.printf("Error %04X: Reservoir temperature too high! %dC > %dC\n", error_code, reservoir_temp_reading, reservoir_temp_high_limit);
    }
    if (reservoir_temp_reading < reservoir_temp_low_limit)
    {
        setError(RESERVOIR_TEMP_TOO_LOW);
        Serial.printf("Error %04X: Reservoir temperature too low! %dC < %dC\n", error_code, reservoir_temp_reading, reservoir_temp_low_limit);
    }

    /*
     *   Outside Temp Measurement
     */
    outside_temp_reading = sensors.getTempC(outside_temp);
    if (outside_temp_reading == DEVICE_DISCONNECTED_C)
    {
        setError(CASE_NO_OUTSIDE_DS18B20_READ);
        Serial.printf("Error %04X: Could not read outside temperature data", error_code);
        outside_temp_reading = 0.0;
    }
    if (outside_temp_reading > outside_temp_high_limit)
    {
        setError(CASE_OUTSIDE_TEMP_TOO_HIGH);
        Serial.printf("Error %04X: Outside temperature too high! %dC > %dC\n", error_code, outside_temp_reading, outside_temp_high_limit);
    }
    if (outside_temp_reading < outside_temp_low_limit)
    {
        setError(CASE_OUTSIDE_TEMP_TOO_LOW);
        Serial.printf("Error %04X: Outside temperature too low! %dC < %dC\n", error_code, outside_temp_reading, outside_temp_low_limit);
    }

    /*
     *   Fan RPM Measurement
     */
    if (top_fan.available())
    {
        topRA.addValue(top_fan.read());
    }
    if (bottom_fan.available())
    {
        bottomRA.addValue(bottom_fan.read());
    }
    if (millis() - reading_time > FAN_SAMPLING_TIME)
    {
        if (topRA.getCount() > 0)
        {
            top_fan_reading = HZ_TO_RPM * top_fan.countToFrequency(topRA.getAverage());
        }
        else
        {
            top_fan_reading = 0;
            if (fan_pwm_level > 0)
            {
                setError(CASE_TOP_FAN_LOW_RPM);
                Serial.printf("Error %04X: Top fan RPM too low!", error_code);
            }
        }

        if (bottomRA.getCount() > 0)
        {
            bottom_fan_reading = HZ_TO_RPM * bottom_fan.countToFrequency(bottomRA.getAverage());
        }
        else
        {
            bottom_fan_reading = 0;
            if (fan_pwm_level > 0)
            {
                setError(CASE_BOTTOM_FAN_LOW_RPM);
                Serial.printf("Error %04X: Bottom fan RPM too low!", error_code);
            }
        }
    }

    /*
     *   Filter Delta-P Measurement
     */
    filterRA.addValue(analogRead(FILTER_P));
    if (filterRA.getAverage() > filter_high_limit)
    {
        setError(CASE_FILTERS_CLOGGED);
        Serial.printf("Error %04X: Filter delta-P too high! %d > %d\n", error_code, (int)filterRA.getAverage(), filter_high_limit);
    }

    /*
     *   Cooling Cycle
     */
    compressor_time = millis() - last_compressor;
    valve_time = millis() - last_valve;

    if (running_state)
    {
        if (reservoir_temp_reading > reservoir_setpoint + hysteresis)
        {
            if (digitalRead(VALVE_RLY) == HIGH)
            {
                last_valve = millis();
                digitalWrite(VALVE_RLY, LOW);
            }
            if (valve_time >= valve_lockout && compressor_time >= compressor_lockout)
            {
                last_compressor = millis();
                fan_pwm_level = 255;
                analogWrite(BOTTOM_FAN_PWM, fan_pwm_level);
                digitalWrite(COMPRESSOR_RLY, HIGH);
            }
        }
        if (reservoir_temp_reading <= reservoir_setpoint - hysteresis)
        {
            if (millis() - last_compressor >= compressor_lockout)
            {
                last_compressor = millis();
                last_valve = millis();
                fan_pwm_level = 0;
                analogWrite(BOTTOM_FAN_PWM, fan_pwm_level);
                digitalWrite(COMPRESSOR_RLY, LOW);
                digitalWrite(VALVE_RLY, HIGH);
            }
        }
        if (digitalRead(PUMP_RLY) == HIGH && digitalRead(FLOW_SW) == HIGH)
        {
            setError(RESERVOIR_PUMP_ON_WITH_NO_FLOW);
            Serial.printf("Error %04X: No flow with pump running!", error_code);
        }
    }

    /*
     *   Display Update Cycle
     */
    if (millis() - reading_time >= PAGE_DELAY)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        reading_time = millis();
        ++reading_state;
        if (reading_state >= 4)
            reading_state = 0;
        switch (reading_state)
        {
        case 0:
            display.clearDisplay();
            ringMeter("Case T", temp_event.temperature, 0, 100, 0, 0, GAUGE_RADIUS, "\xF8"
                                                                                    "C");
            ringMeter("Case RH", humidity_event.relative_humidity, 0, 100, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "%");
            display.display();
            break;
        case 1:
            display.clearDisplay();
            ringMeter("Res T", reservoir_temp_reading, 0, 100, 0, 0, GAUGE_RADIUS, "\xF8"
                                                                                   "C");
            ringMeter("Out T", outside_temp_reading, 0, 100, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "\xF8"
                                                                                                               "C");
            display.display();
            break;
        case 2:
            display.clearDisplay();
            ringMeter("Top Fan", top_fan_reading, 0, 6000, 0, 0, GAUGE_RADIUS, "RPM");
            ringMeter("Bot Fan", bottom_fan_reading, 0, 6000, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "RPM");
            display.display();
            break;
        case 3:
            display.clearDisplay();
            ringMeter("Res Lvl", (int)(resRA.getAverage() / 1000.0), 0, (int)(reservoir_area * reservoir_height), 0, 0, GAUGE_RADIUS, "L");
            ringMeter("\x83 P", (int)(filterRA.getAverage()), 0, 1024, SCREEN_WIDTH - 2 * GAUGE_RADIUS, 0, GAUGE_RADIUS, "ADC");
            display.display();
            break;
        default:
            break;
        }
    }
}

void setError(uint16_t error)
{
    error_code = error;
    digitalWrite(ALARMS_RLY, LOW);
}

// function to print a device address
void printAddress(DeviceAddress deviceAddress)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        // zero pad the address if necessary
        if (deviceAddress[i] < 16)
            Serial.print("0");
        Serial.print(deviceAddress[i], HEX);
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