#include <Arduino.h>
#include "abp2.h"

/*!
 *  @brief  class constructor
 */
Honeywell_ABP2::Honeywell_ABP2() {}

Honeywell_ABP2::~Honeywell_ABP2(void)
{
    if (i2c_dev)
    {
        delete i2c_dev;
    }
    if (temp_sensor)
    {
        delete temp_sensor;
    }
    if (pressure_sensor)
    {
        delete pressure_sensor;
    }
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @param addr the I2C address the device can be found on
 *   @param theWire the I2C object to use, defaults to &Wire
 *   @returns true on success, false otherwise
 */
bool Honeywell_ABP2::begin(uint8_t addr, TwoWire *theWire)
{
    // I2C mode
    if (i2c_dev)
        delete i2c_dev;
    i2c_dev = new Adafruit_I2CDevice(addr, theWire);
    if (!i2c_dev->begin())
        return false;

    return init();
}

/*!
 *   @brief  Initialise sensor with given parameters / settings
 *   @returns true on success, false otherwise
 */
bool Honeywell_ABP2::init()
{
    // wait for chip to wake up.
    delay(10);
    abp2_data_packet packet = readData();
    return packet.status.powered & !packet.status.math_error & !packet.status.mem_error;
}

/*!
 *   @brief  Returns the temperature from the sensor
 *   @returns the temperature read from the device
 */
float Honeywell_ABP2::readTemperature(void)
{
    abp2_data_packet packet = readData();
    if (!packet.status.powered || packet.status.math_error || packet.status.mem_error)
        return NAN;

    return ((float)packet.temperature * (ABP2_T_MAX - ABP2_T_MIN)) /
               (ABP2_OUTPUT_BITS_MAX) +
           ABP2_T_MIN;
}

/*!
 *   @brief  Returns the pressure from the sensor
 *   @returns the pressure value (in sensor units)
 */
float Honeywell_ABP2::readPressure(void)
{
    abp2_data_packet packet = readData();
    if (!packet.status.powered || packet.status.math_error || packet.status.mem_error)
        return NAN;

    return (((float)packet.pressure - ABP2_OUTPUT_MIN) * (ABP2_P_MAX - ABP2_P_MIN)) /
               (ABP2_OUTPUT_MAX - ABP2_OUTPUT_MIN) +
           ABP2_P_MIN;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the temp sensor component
    @return Adafruit_Sensor pointer to temperature sensor
 */
Adafruit_Sensor *Honeywell_ABP2::getTemperatureSensor(void)
{
    if (!temp_sensor)
    {
        temp_sensor = new Honeywell_ABP2_Temp(this);
    }

    return temp_sensor;
}

/*!
    @brief  Gets an Adafruit Unified Sensor object for the pressure sensor
   component
    @return Adafruit_Sensor pointer to pressure sensor
 */
Adafruit_Sensor *Honeywell_ABP2::getPressureSensor(void)
{
    if (!pressure_sensor)
    {
        pressure_sensor = new Honeywell_ABP2_Pressure(this);
    }
    return pressure_sensor;
}

/*!
 *   @brief  Reads full packet over I2C
 *   @returns the packet read from the device
 */
abp2_data_packet Honeywell_ABP2::readData()
{
    abp2_data_packet packet;
    packet.status.busy = true;
    uint8_t buffer[7];

    if (i2c_dev)
    {
        while (packet.status.busy)
        {
            i2c_dev->write_then_read(ABP2_OUTPUT_MSMT_CMD, 3, buffer, 3);
            packet.status.powered = bitRead(buffer[0], 6);
            packet.status.busy = bitRead(buffer[0], 5);
            packet.status.mem_error = bitRead(buffer[0], 2);
            packet.status.math_error = bitRead(buffer[0], 0);

            packet.pressure = uint32_t(buffer[1]) << 16 | uint32_t(buffer[2]) << 8 | uint32_t(buffer[3]);
            packet.temperature = uint32_t(buffer[4]) << 16 | uint32_t(buffer[5]) << 8 | uint32_t(buffer[6]);
        }
    }

    return packet;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the ABP2's temperature sensor
*/
/**************************************************************************/
void Honeywell_ABP2_Temp::getSensor(sensor_t *sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "ABP2", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = 0;
    sensor->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    sensor->min_delay = 0;
    sensor->min_value = ABP2_T_MIN;
    sensor->max_value = ABP2_T_MAX;
    sensor->resolution = 0.01;
}

/**************************************************************************/
/*!
    @brief  Gets the temperature as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Honeywell_ABP2_Temp::getEvent(sensors_event_t *event)
{
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = 0;
    event->type = SENSOR_TYPE_AMBIENT_TEMPERATURE;
    event->timestamp = millis();
    event->temperature = _theABP2->readTemperature();
    return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data for the ABP2's pressure sensor
*/
/**************************************************************************/
void Honeywell_ABP2_Pressure::getSensor(sensor_t *sensor)
{
    /* Clear the sensor_t object */
    memset(sensor, 0, sizeof(sensor_t));

    /* Insert the sensor name in the fixed length char array */
    strncpy(sensor->name, "ABP2", sizeof(sensor->name) - 1);
    sensor->name[sizeof(sensor->name) - 1] = 0;
    sensor->version = 1;
    sensor->sensor_id = 0;
    sensor->type = SENSOR_TYPE_PRESSURE;
    sensor->min_delay = 0;
    sensor->min_value = ABP2_P_MIN;
    sensor->max_value = ABP2_P_MAX;
    sensor->resolution = 0.01;
}

/**************************************************************************/
/*!
    @brief  Gets the pressure as a standard sensor event
    @param  event Sensor event object that will be populated
    @returns True
*/
/**************************************************************************/
bool Honeywell_ABP2_Pressure::getEvent(sensors_event_t *event)
{
    /* Clear the event */
    memset(event, 0, sizeof(sensors_event_t));

    event->version = sizeof(sensors_event_t);
    event->sensor_id = 0;
    event->type = SENSOR_TYPE_PRESSURE;
    event->timestamp = millis();
    event->pressure = _theABP2->readPressure();
    return true;
}