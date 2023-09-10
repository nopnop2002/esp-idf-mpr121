# esp-idf-mpr121
MPR121 Capacitive Touch Driver for esp-idf.

I ported from [here](https://github.com/BareConductive/mpr121).   

# Software requirements
ESP-IDF V4.4/V5.x.   
ESP-IDF V5.0 is required when using ESP32-C2.   
ESP-IDF V5.1 is required when using ESP32-C6.   

# Hardware requirements   
MPR121 Capacitive Touch switch.

![mpr121-1](https://user-images.githubusercontent.com/6020549/147515909-cd50a16a-5c60-4bd0-bc32-c288f5d8ee88.JPG)


# Installation

```Shell
git clone https://github.com/nopnop2002/esp-idf-mpr121
cd esp-idf-mpr121
idf.py set-target {esp32/esp32s2/esp32s3/esp32c2/esp32c3/esp32c6}
idf.py menuconfig
idf.py flash
```

# Configuration   

![config-top](https://user-images.githubusercontent.com/6020549/147515950-b6e2cf2f-5a82-4114-a565-df7a78085c8d.jpg)

![config-mpr121](https://github.com/nopnop2002/esp-idf-mpr121/assets/6020549/6780b5af-256e-4a12-866a-742868694124)


# Wirering

|MPR121||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6|
|:-:|:-:|:-:|:-:|:-:|
|SCL|--|GPIO22|GPIO12|GPIO6|
|SDA|--|GPIO21|GPIO11|GPIO5|
|IRQ|--|GPIO15|GPIO18|GPIO4|
|GND|--|GND|GND|GND|
|VCC|--|3.3V|3.3V|3.3V|

__You can change it to any pin using menuconfig.__   


# Screen Shot   
![mpr121-2](https://user-images.githubusercontent.com/6020549/147515969-54901561-66f5-4077-b6f8-7dbd7fe49f2c.jpg)

# Reference   
https://github.com/nopnop2002/esp-idf-ttp229
