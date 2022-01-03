# esp-idf-mpr121
MPR121 Capacitive Touch Driver for esp-idf.

I ported from [here](https://github.com/BareConductive/mpr121).   

# Hardware requirements   
MPR121 Capacitive Touch switch.

![mpr121-1](https://user-images.githubusercontent.com/6020549/147515909-cd50a16a-5c60-4bd0-bc32-c288f5d8ee88.JPG)


# Software requirements
esp-idf v4.4 or later.   
This is because this version supports ESP32-C3.   

# Installation for ESP32

```Shell
git clone https://github.com/nopnop2002/esp-idf-mpr121
cd esp-idf-mpr121
idf.py set-target esp32
idf.py menuconfig
idf.py flash
```

# Installation for ESP32-S2

```Shell
git clone https://github.com/nopnop2002/esp-idf-mpr121
cd esp-idf-mpr121
idf.py set-target esp32s2
idf.py menuconfig
idf.py flash
```

# Installation for ESP32-C3

```Shell
git clone https://github.com/nopnop2002/esp-idf-mpr121
cd esp-idf-mpr121
idf.py set-target esp32c3
idf.py menuconfig
idf.py flash
```


# Configuration   

![config-top](https://user-images.githubusercontent.com/6020549/147515950-b6e2cf2f-5a82-4114-a565-df7a78085c8d.jpg)

![config-mpr121](https://user-images.githubusercontent.com/6020549/147892705-4672b941-3939-42c2-aad9-8a12f49d2451.jpg)


# Wirering

|MPR121||ESP32|ESP32-S2|ESP32-C3|
|:-:|:-:|:-:|:-:|:-:|
|SCL|--|GPIO4|GPIO16|GPIO6|
|SDA|--|GPIO5|GPIO17|GPIO7|
|IRQ|--|GPIO15|GPIO18|GPIO8|
|GND|--|GND|GND|GND|
|VCC|--|3.3V|3.3V|3.3V|

__You can change it to any pin using menuconfig.__   


# Screen Shot   
![mpr121-2](https://user-images.githubusercontent.com/6020549/147515969-54901561-66f5-4077-b6f8-7dbd7fe49f2c.jpg)

# Reference   
https://github.com/nopnop2002/esp-idf-ttp229
