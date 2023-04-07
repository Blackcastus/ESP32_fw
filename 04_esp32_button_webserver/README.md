#esp32_button_webserver

1. Create main/Kconfig.projbuild
    Copy content
2. Create partitison.csv
    Copy content
3. Open file main/CMakeList
    add scr spiffs_create_partition_image(storage ../data FLASH_IN_PROJECT)
4. Create data/index.html
    Copy content
5. Open menuconfig
    Wifi configuration choose (ussid, password)
    Partitiion table choose (partition table: custom partition table csv)
    Serial flasher config  choose (flash size: 4M)