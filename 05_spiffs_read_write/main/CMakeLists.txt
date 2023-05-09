idf_component_register(SRCS "main.c"
                            "config/config.c"
                    INCLUDE_DIRS "." "config")
spiffs_create_partition_image(storage ../spiffs_data FLASH_IN_PROJECT)