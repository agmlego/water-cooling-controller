from time import sleep

import pandas as pd
import serial
from ds1054z import DS1054Z

PING_DELAY = 0.07
PING_ADDR = b'\x09'


def ping(port: serial):
    port.write(PING_ADDR + b'\x51')
    sleep(PING_DELAY)
    port.write(PING_ADDR + b'\x5E')
    return int.from_bytes(port.read(2), 'big', signed=False)


def sample(scope: 'DS1104Z', port: serial, slide):
    print(f'=== Gathering data for slide position {slide} ===')
    data = []
    scope.trigger_single()
    for idx in range(100):
        print(f'IR sample group {idx}')
        for _ in range(10):
            data.append({'type': 'us', 'pos': slide, 'act': ping(port)})
        sleep(0.3)
    print(f'Getting data from {scope.serial}')
    for d in scope.get_waveform_samples(1, 'NORM'):
        data.append({'type': 'ir', 'pos': slide, 'act': d})
    return data


class DS1104Z(DS1054Z):
    def trigger_single(self):
        status = self.query(':TRIGger:STATus?')
        while status != 'TD':
            print(f'Scope is {status}')
            match status:
                case 'WAIT':
                    self.tforce()
                case 'RUN':
                    sleep(0.050)
                case _:
                    self.single()
            status = self.query(':TRIGger:STATus?')


if __name__ == '__main__':
    rigol = DS1104Z('10.11.12.15')
    ftdi = serial.Serial('COM3', baudrate=9600, stopbits=2, parity='N')
    combined = []
    for pos in (0.0, 0.5, 1.0, 1.5, 2.0):
        input(f'Press <ENTER> for position {pos}')
        combined.extend(sample(rigol, ftdi, pos))

    df = pd.DataFrame(combined)
    df.groupby(['type', 'pos']).agg(['mean', 'std']).to_csv('sensors.csv')
