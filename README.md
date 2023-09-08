External Water-Cooling Loop Controller
--------------------------------------

The intent of this project is to monitor and control an external chiller system for a PC water-cooling loop with a liquid-liquid heat exchanger to separate the internal and external loops.

There is a PCIe x1 card inside the PC, tapping into SMBus and +3V3AUX, to monitor the flow, inlet, and outlet temperatures for the internal and external loops, and intercept the front panel power switch to prevent boot or to initiate a shutdown if the loops are not working correctly. The Teensy 3.2 will communicate to the PC, pretending to be a common SMBus device such as those from Winbond or Nuvoton. It also talks with an external board in the chiller over CAN. Onboard WS2811B and an OLED screen show immediate diagnostics. An onboard BME280 lets the Teensy monitor internal case ambient temperature and humidity, to ensure that the chiller does not drop below the local dewpoint to prevent condensation.

The external board listens to CAN and adjusts the chiller's power state and setpoint. It also reports the local BME280 results and has an onboard OLED screen for system status.

## Internal PCIe Card

| Pin | Type    | Signal | Direction | Purpose                                      |
| --- | ------- | ------ | --------- | -------------------------------------------- |
| 1   | POWER   | GND    |           | Digital   ground plane                       |
| 2   | DIGITAL | D0     | INPUT     | Internal loop flow sensor                    |
| 3   | DIGITAL | D1     | INPUT     | External   loop flow sensor                  |
| 4   | DIGITAL | D2     | OUTPUT    | CAN standby, active high                     |
| 5   | CAN     | TX     | OUTPUT    | CAN   transmit                               |
| 6   | CAN     | RX     | INPUT     | CAN receive                                  |
| 7   | DIGITAL | D5     | OUTPUT    | Level   shifter output enable, active low    |
| 8   | DIGITAL | D6     | INPUT     | PCIe #PERST status                           |
| 9   | DIGITAL | D7     | OUTPUT    | WS2811B   data                               |
| 21  | ANALOG  | A0     | INPUT     | External loop outflow temperature, 10k   NTC |
| 22  | ANALOG  | A1     | INPUT     | External   loop inflow temperature, 10k NTC  |
| 23  | ANALOG  | A2     | INPUT     | Internal loop inflow temperature, 10k   NTC  |
| 24  | ANALOG  | A3     | INPUT     | Internal   loop outflow temperature, 10k NTC |
| 25  | I2C     | SDA0   | BIDI      | SMBus I2C data                               |
| 26  | I2C     | SCL0   | BIDI      | SMBus   I2C clock                            |
| 29  | DIGITAL | D22    | OUTPUT    | Front panel power switch output              |
| 30  | DIGITAL | D23    | INPUT     | Front   panel power switch input             |
| 31  | POWER   | 3V3    |           | 3.3V AUX input                               |
| 32  | POWER   | AGND   |           | Analog   ground plane                        |
| 42  | POWER   | 3V3    |           | 3.3V AUX input                               |
| 46  | I2C     | SDA1   | BIDI      | Local   I2C data                             |
| 47  | I2C     | SCL1   | BIDI      | Local I2C clock                              |
| 53  | POWER   | GND    |           | Digital   ground plane                       |

### WS2811B

1. External loop temperature status
2. External loop flow status
3. Internal loop temperature status
4. Internal loop flow status

### CAN DE-9

0. Chassis earth
1. N/C
2. CAN_H
3. GND
4. N/C
5. Chassis earth
6. GND
7. CAN_H
8. N/C
9. +5V

## Chiller Controller

| Pin | Type    | Signal | Direction | Purpose                                      |
| --- | ------- | ------ | --------- | -------------------------------------------- |
| 1   | POWER   | GND    |           | Digital   ground plane                       |
| 2   | DIGITAL | D0     | INPUT     | Top fan RPM sensor                           |
| 3   | DIGITAL | D1     | INPUT     | Bottom fan RPM sensor                        |
| 4   | 1-WIRE  | D2     | BIDI      | DS18B20 bus                                  |
| 5   | CAN     | TX     | OUTPUT    | CAN   transmit                               |
| 6   | CAN     | RX     | INPUT     | CAN receive                                  |
| 7   | DIGITAL | D5     | OUTPUT    | Fan PWM                                      |
| 10  | DIGITAL | D8     | INPUT     | Flow switch                                  |
| 11  | DIGITAL | D9     | OUTPUT    | Pump relay                                   |
| 12  | DIGITAL | D10    | OUTPUT    | Expansion valve relay                        |
| 13  | DIGITAL | D11    | OUTPUT    | Compressor relay                             |
| 14  | DIGITAL | D11    | OUTPUT    | Status lights/buzzer relay                   |
| 21  | ANALOG  | A0     | INPUT     | Filter check diff. pressure sensor           |
| 25  | I2C     | SDA0   | BIDI      | Local I2C data                               |
| 26  | I2C     | SCL0   | BIDI      | Local   I2C clock                            |

### NF-A14 Control
* PWM: 40μs period, 5Vpp
* 8.76% minimum kickon => 468RPM (64ms/15.6Hz on tach.)
* 100% => 3180RPM (9.4ms/106.4Hz on tach.)