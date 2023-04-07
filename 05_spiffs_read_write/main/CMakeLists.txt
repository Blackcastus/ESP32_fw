idf_component_register(SRCS "main.c"
                            "ntt/ntt.c"
                    INCLUDE_DIRS "." "ntt ")
spiffs_create_partition_image(storage ../spiffs_data FLASH_IN_PROJECT)