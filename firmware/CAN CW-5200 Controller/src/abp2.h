#ifndef __ABP2_H__
#define __ABP2_H__
#include <cstdint>
#include "Arduino.h"
#include <Adafruit_I2CDevice.h>
#include <Adafruit_Sensor.h>

/*!
 *  @brief  default I2C address
 */
#define ABP2_ADDRESS (0x28) // Primary I2C Address

/*
 * Values for the `002ND` differential variant:
 *      -2..2inH2O pressure
 *      -50..150°C temperature
 *      Transfer function B [0.3..0.7]
 */
const float ABP2_P_MAX = 2;          // sensor range high pressure
const float ABP2_P_MIN = -2;         // sensor range low pressure
const float ABP2_T_MAX = 150;        // sensor range high temperature
const float ABP2_T_MIN = -50;        // sensor range low temperature
const float ABP2_TXFR_FN_HI = 0.7;   // high transfer function value
const float ABP2_TXFR_FN_LO = 0.3;   // low transfer function value
const uint8_t ABP2_OUTPUT_BITS = 24; // sensor resolution
/*
 * End variant-specific values
 */

const float ABP2_OUTPUT_MAX = (1 << ABP2_OUTPUT_BITS) * ABP2_TXFR_FN_HI;
const float ABP2_OUTPUT_MIN = (1 << ABP2_OUTPUT_BITS) * ABP2_TXFR_FN_LO;
const float ABP2_OUTPUT_BITS_MAX = (1 << ABP2_OUTPUT_BITS) - 1;
const uint8_t ABP2_OUTPUT_MSMT_CMD[] = {0xAA, 0x00, 0x00};

/**************************************************************************/
/*!
    @brief  status flags
*/
/**************************************************************************/
typedef struct
{
    bool powered;    ///< power indication (bit 6)
    bool busy;       ///< busy indication (bit 5)
    bool mem_error;  ///< memory integrity error (bit 2)
    bool math_error; ///< math saturation error (bit 0)
} abp2_status_flags;
/*=========================================================================*/

/**************************************************************************/
/*!
    @brief  measurement packet
*/
/**************************************************************************/
typedef struct
{
    abp2_status_flags status; ///< status flags
    uint32_t pressure;        ///< 24-bit pressure
    uint32_t temperature;     ///< 24-bit temperature
} abp2_data_packet;
/*=========================================================================*/

class Honeywell_ABP2;

/** Adafruit Unified Sensor interface for temperature component of ABP2 */
class Honeywell_ABP2_Temp : public Adafruit_Sensor
{
public:
    /** @brief Create an Adafruit_Sensor compatible object for the temp sensor
        @param parent A pointer to the ABP2 class */
    Honeywell_ABP2_Temp(Honeywell_ABP2 *parent) { _theABP2 = parent; }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);

private:
    Honeywell_ABP2 *_theABP2 = NULL;
};

/** Adafruit Unified Sensor interface for pressure component of ABP2 */
class Honeywell_ABP2_Pressure : public Adafruit_Sensor
{
public:
    /** @brief Create an Adafruit_Sensor compatible object for the pressure sensor
        @param parent A pointer to the ABP2 class */
    Honeywell_ABP2_Pressure(Honeywell_ABP2 *parent) { _theABP2 = parent; }
    bool getEvent(sensors_event_t *);
    void getSensor(sensor_t *);

private:
    Honeywell_ABP2 *_theABP2 = NULL;
};

/**************************************************************************/
/*!
    @brief  Class that stores state and functions for interacting with ABP2 IC
*/
/**************************************************************************/
class Honeywell_ABP2
{
public:
    // constructors
    Honeywell_ABP2();
    ~Honeywell_ABP2(void);
    bool begin(uint8_t addr = ABP2_ADDRESS, TwoWire *theWire = &Wire);
    bool init();

    float readTemperature(void);
    float readPressure(void);

    Adafruit_Sensor *getTemperatureSensor(void);
    Adafruit_Sensor *getPressureSensor(void);

protected:
    Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
    Honeywell_ABP2_Temp *temp_sensor = NULL;
    //!< Adafruit_Sensor compat temperature sensor component

    Honeywell_ABP2_Pressure *pressure_sensor = NULL;
    //!< Adafruit_Sensor compat pressure sensor component

    abp2_data_packet readData();

private:
};

#endif