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
![Image](https://github.com/user-attachments/assets/66e7e417-3a55-49aa-ab57-8e9ff0f848a3)
![Image](https://github.com/user-attachments/assets/40d3a5ac-43cd-4fc7-a80d-4f330d677d94)

## I2C Port selection   
XTENSA's ESP32 has two i2c port: I2C_NUM_0/1.   
You can use these two ports freely.   
If you use this module at the same time as another I2C device using I2C port 0, you must change it to I2C port 1.   

# Wirering

|MPR121||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6|
|:-:|:-:|:-:|:-:|:-:|
|SCL|--|GPIO19|GPIO12|GPIO6|
|SDA|--|GPIO18|GPIO11|GPIO5|
|IRQ|--|GPIO15|GPIO18|GPIO4|
|GND|--|GND|GND|GND|
|VCC|--|3.3V|3.3V|3.3V|

__Note:__It is recommended to add external pull-up resistors for SDA/SCL pins to make the communication more stable, though the driver will enable internal pull-up resistors.   

__You can change it to any pin using menuconfig.__   

# Screen Shot   
![mpr121-2](https://user-images.githubusercontent.com/6020549/147515969-54901561-66f5-4077-b6f8-7dbd7fe49f2c.jpg)

Supports multiple simultaneous touches.   
```
I (55786) MAIN: electrode 0 was just touched
I (55786) MAIN: electrode 1 was just touched
I (55786) MAIN: electrode 2 was just touched
I (55786) MAIN: electrode 3 was just touched
I (58446) MAIN: electrode 0 was just released
I (58446) MAIN: electrode 1 was just released
I (58446) MAIN: electrode 2 was just released
I (58446) MAIN: electrode 3 was just released
```

# How to use this component in your project   
Create idf_component.yml in the same directory as main.c.   
```
YourProject --+-- CMakeLists.txt
              +-- main --+-- main.c
                         +-- CMakeLists.txt
                         +-- idf_component.yml
```

Contents of idf_component.yml.
```
dependencies:
  nopnop2002/mpr121:
    path: components/mpr121
    git: https://github.com/nopnop2002/esp-idf-mpr121.git
```

When you build a projects esp-idf will automaticly fetch repository to managed_components dir and link with your code.   
```
YourProject --+-- CMakeLists.txt
              +-- main --+-- main.c
              |          +-- CMakeLists.txt
              |          +-- idf_component.yml
              +-- managed_components ----- nopnop2002__mpr121
```

# Reference   
https://github.com/nopnop2002/esp-idf-ttp229
