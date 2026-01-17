idf_component_register(
    SRCS 
        "main.c" 
        "slave_objectlist.c"
    INCLUDE_DIRS 
        "."
    REQUIRES 
        soes  # This tells ESP-IDF to link against the SOES component
        driver # Required for GPIO
        esp_timer # Required for HAL
)