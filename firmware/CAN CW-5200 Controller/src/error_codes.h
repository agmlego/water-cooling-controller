#ifndef __CW5200_CAN_ERRORS__
#define __CW5200_CAN_ERRORS__

#define RESERVOIR_VL53L0X_NO_CONNECT 0x0101    // VL53L0X No Connect!
#define RESERVOIR_NO_DS18B20_ADDRESS 0x0102    // No Reservoir DS18B20 Address!
#define RESERVOIR_NO_DS18B20_READ 0x0103       // No Reservoir DS18B20 Read!
#define RESERVOIR_OPEN_LOOP 0x0104             // Reservoir Open Loop!
#define RESERVOIR_LEVEL_LOW 0x0105             // Reservoir Level Low!
#define RESERVOIR_PUMP_ON_WITH_NO_FLOW 0x0106  // Pump On With No Flow!
#define RESERVOIR_TEMP_TOO_HIGH 0x0107         // Reservoir Temp Too High!
#define RESERVOIR_TEMP_TOO_LOW 0x0107          // Reservoir Temp Too Low!
#define CASE_BME280_NO_CONNECT 0x0201          // BME280 No Connect!
#define CASE_NO_OUTSIDE_DS18B20_ADDRESS 0x0202 // No Outside DS18B20 Address!
#define CASE_NO_OUTSIDE_DS18B20_READ 0x0203    // No Outside DS18B20 Read!
#define CASE_DISPLAY_NO_CONNECT 0x0204         // Display No Connect!
#define CASE_OUTSIDE_TEMP_TOO_HIGH 0x0205      // Outside Temp Too High!
#define CASE_OUTSIDE_TEMP_TOO_LOW 0x0206       // Outside Temp Too Low!
#define CASE_TEMP_TOO_HIGH 0x0207              // Case Temp Too High!
#define CASE_TEMP_TOO_LOW 0x0208               // Case Temp Too Low!
#define CASE_HUMIDITY_TOO_HIGH 0x0209          // Case Humidity Too High!
#define CASE_TOP_FAN_LOW_RPM 0x020A            // Top Fan Low RPM!
#define CASE_BOTTOM_FAN_LOW_RPM 0x020B         // Bottom Fan Low RPM!
#define CASE_FILTERS_CLOGGED 0x020C            // Filters Clogged!

#endif