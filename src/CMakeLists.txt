idf_component_register(SRCS 
                       "main.cpp"
                       "GPIOManager.cpp"
                       "USBManager.cpp"
                       "RadioManager.cpp"
                       "Tasks.cpp"
                       "Utils.cpp"
                       INCLUDE_DIRS "../include"
                       REQUIRES driver esp_common freertos)