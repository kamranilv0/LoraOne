idf_component_register(SRCS 
                           "main.cpp"
                           "RadioManager.cpp"
                           "SerialBridge.cpp"
                           "HardwareInit.cpp"
                           "Utils.cpp"
                       INCLUDE_DIRS "../include"
                       REQUIRES driver esp_common freertos)