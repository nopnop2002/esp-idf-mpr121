# esp-idf-mpr121
MPR121 Capacitive Touch Driver for esp-idf.

I ported from [here](https://github.com/BareConductive/mpr121).   

# Software requirements
ESP-IDF V5.0 or later.   
ESP-IDF V4.4 release branch reached EOL in July 2024.   
ESP-IDF V5.1 is required when using ESP32-C6.   

__Note for ESP-IDF V5.2.__   
A new i2c driver is now available in ESP-IDF V5.2.   
Under ESP-IDF V5.2 or later, this project uses a new i2c driver.   

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
![Image](https://github.com/user-attachments/assets/0629e4ae-c980-4dc8-b18a-07b98681a1a3)

# I2C Clock speed   
According to the MPR121 datasheet, the maximum i2c clock frequency is 400KHz.   
The i2c clock frequency used by this project is 400KHz.   

# I2C Port selection   
XTENSA's ESP32 has two i2c port: I2C_NUM_0/1.   
You can use these two ports freely.   
If you use this module at the same time as another I2C device using I2C port 0, you must change it to I2C port 1.   
![Image](https://github.com/user-attachments/assets/797d89e9-3d6f-45b5-b5a1-791ce00a8340)

# Force legacy i2c driver
XTENSA's ESP32 has two i2c port: I2C_NUM_0/1.   
I2C_NUM_0/1 are initialized independently, but legacy i2c drivers and new i2c drivers cannot be mixed.   
If I2C_NUM_0 is initialized with the legacy i2c driver, I2C_NUM_1 must also be initialized with the legacy i2c driver.   
If you use the MPR121 at the same time as other I2C devices that use legacy I2C drivers, the MPR121 must also be initialized with the legacy I2C driver.   
Enabling this will use the legacy i2c driver even after ESP-IDF V5.2.   
![Image](https://github.com/user-attachments/assets/d40cdd42-3a03-4ea8-a0ea-ebdfe2cdb8e2)

# Wirering

|MPR121||ESP32|ESP32-S2/S3|ESP32-C2/C3/C6|
|:-:|:-:|:-:|:-:|:-:|
|SCL|--|GPIO19|GPIO12|GPIO6|
|SDA|--|GPIO18|GPIO11|GPIO5|
|IRQ|--|GPIO15|GPIO18|GPIO4|
|GND|--|GND|GND|GND|
|VCC|--|3.3V|3.3V|3.3V|

__Note__   
It is recommended to add external pull-up resistors for SDA/SCL pins to make the communication more stable, though the driver will enable internal pull-up resistors.   

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
