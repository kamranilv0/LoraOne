# BosMag LoRa Transceiver - CMakeLists.txt
# ESP32-S3 + Ebyte E22-900M-30S (SX1262) with RadioLib
# This file should be placed in: src/CMakeLists.txt

idf_component_register(
    SRCS 
        "main.cpp"
        "GPIOManager.cpp"
        "USBManager.cpp"
        "RadioManager.cpp"
        "Tasks.cpp"
        "Utils.cpp"
    INCLUDE_DIRS "../include"
    REQUIRES driver esp_common freertos)
