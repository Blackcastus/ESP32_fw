#03_esp32_sht31

#Hardware
SHT31       |   ESP32
VCC         |   3V3
GND         |   GND
SDA         |   21
SCL         |   22

#firmware

inclue file main/CMakeList.txt
    scr "SHT3x/sht31.c"
    INCLUE_DIR "sht3x"