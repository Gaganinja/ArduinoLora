# LorArduino
## Test for using LoRa SX1280 Evaluation board on arduino uno

using the Platformio environment in order to share the code.

## Testing

Tested on Arduino Uno Rev3 with a grove base shield and the SX1280RF1ZHP evaluation board from Semtech.
With DHT22 sensor and Air quality sensor attached to the base shield. 
(Warning : grove cables has been modified to match with DHT22 pinout)
Be careful : To make SX1280RF1ZHP evaluation board working with Arduino Uno (in order to send data), remove the jumper for VDD_RADIO and put it a the 3.3V regulated from the Arduino. 

Gateway side : SX1280Z3DSFGW1 
follow user guide [here](https://www.semtech.com/products/wireless-rf/lora-core/sx1280zxxxxgw1)to install it. 
## Dependancies

- Grove_-_Air_quality_sensor library (automated install by platformio)
- Adafruit Unified Sensor library (automated install by platformio)
- DHT sensor library (automated install by platformio)
- SX12XX-LoRa library from StuartsProjects [link here](https://github.com/StuartsProjects/SX12XX-LoRa)


## Links : 

https://os.mbed.com/components/SX1280RF1ZHP/ : SX1280RF1ZHP kit with nucleo-L476RG + SX1280 evaluation board + screen
Containing :
 - the SX1280PingPong code and SX1280Lib library to flash to nucleo (in order to reset it to Factory settings)
 - SX1280 evaluation board pinout, schematic, layout, BOM
 - Some other libraries
 - SX1280 link to datasheet (dead, last link found : [here](https://www.semtech.com/products/wireless-rf/lora-connect/sx1280))