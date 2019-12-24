# BMP280 Digital Pressure Sensor Driver for Python3

BMP280 is a digital pressure sensor manufactured by Bosch Sensortec. With this driver, you can use BMP280 with Python3 over I2C.

## Dependencies
Only smbus2 other than Python. 

## Installation
```bash
pip3 install bmp280
```

## Usage
```bash
run-bmp280
```
or
```python
from bmp_280 import BMP280
from time import sleep

bmp = BMP280(port=1, mode=BMP280.FORCED_MODE, oversampling_p=BMP280.OVERSAMPLING_P_x16, oversampling_t=BMP280.OVERSAMPLING_T_x1,
            filter=BMP280.IIR_FILTER_OFF, standby=BMP280.T_STANDBY_1000)

pressure = bmp.read_pressure()
temp = bmp.read_temperature()

print("Pressure (hPa): " + str(pressure))
print("Temperature (°C): " + str(temp))
```

## Contributing
Pull requests are welcome. For major changes, please open an issue first to discuss what you would like to change.

Please make sure to update tests as appropriate.

## License
[MIT](https://choosealicense.com/licenses/mit/)
