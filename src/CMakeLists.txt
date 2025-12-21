# BosMag LoRa Transceiver - CMakeLists.txt
# ESP32-S3 + Ebyte E22-900M-30S (SX1262) with RadioLib

idf_component_register(
    SRCS 
        "main.cpp"
        "GPIOManager.cpp"
        "USBManager.cpp"
        "RadioManager.cpp"
        "Tasks.cpp"
        "Utils.cpp"
    INCLUDE_DIRS 
        "."
    REQUIRES
        driver
        esp_timer
        freertos
        # Add RadioLib component here if it's a separate component
        # radiolib
)

# Set C++ standard
target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17)

# Add any RadioLib-specific compile definitions if needed
# target_compile_definitions(${COMPONENT_LIB} PRIVATE 
#     RADIOLIB_PLATFORM=RADIOLIB_PLATFORM_ESPRESSIF
# )