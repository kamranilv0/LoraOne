# LoraOne Modularization Plan

## Current State Analysis

The project currently has **all functionality in a single 600+ line `main.cpp` file**, which includes:
- GPIO setup
- SPI initialization
- UART handling
- LoRa radio initialization
- RF switch control
- LNA boost register access
- Serial RX task
- Radio RX task
- Main application logic

## Problems with Current Architecture

1. **Poor Separation of Concerns** - Hardware drivers mixed with application logic
2. **Difficult Testing** - Cannot test modules independently
3. **Hard to Maintain** - Changes ripple through entire file
4. **Code Reusability** - Cannot reuse radio or UART modules in other projects
5. **Scalability** - Adding new features becomes increasingly difficult

## Proposed Modular Architecture

```
src/
├── main.cpp                    # Application entry point only
├── RadioManager.cpp/h          # LoRa radio abstraction
├── SerialBridge.cpp/h          # UART<->LoRa bridge logic
├── HardwareInit.cpp/h          # GPIO, SPI, UART initialization
└── Utils.cpp/h                 # Helper functions (hex dump, delays)

include/
├── Config.h                    # Configuration (existing)
├── EspHal.h                    # RadioLib HAL (existing)
├── RadioManager.h              # Radio interface
├── SerialBridge.h              # Bridge interface
├── HardwareInit.h              # Hardware setup interface
└── Utils.h                     # Utilities interface
```

## Module Breakdown

### 1. **RadioManager** (RadioManager.cpp/h)
**Responsibility**: Complete LoRa radio lifecycle management

**Public Interface**:
```cpp
class RadioManager {
public:
    static RadioManager& getInstance();
    
    void init();
    void startReceive();
    void transmit(const uint8_t* data, size_t length);
    bool hasDataAvailable();
    int16_t readData(uint8_t* buffer, size_t maxLength);
    
    float getRSSI();
    float getSNR();
    
private:
    // SX1262 radio instance
    // Mutex for thread safety
    // State management
    // Interrupt callbacks
};
```

**Encapsulates**:
- Radio initialization
- RF switch control
- LNA boost setup
- TX/RX state machine
- Interrupt handling
- Thread safety (mutex)

---

### 2. **SerialBridge** (SerialBridge.cpp/h)
**Responsibility**: Bidirectional UART<->LoRa data transfer

**Public Interface**:
```cpp
class SerialBridge {
public:
    SerialBridge(RadioManager& radio);
    
    void start();
    void stop();
    
private:
    void serialRxTask();
    void radioRxTask();
    
    RadioManager& m_radio;
    TaskHandle_t m_serialTask;
    TaskHandle_t m_radioTask;
};
```

**Encapsulates**:
- FreeRTOS task management
- Serial to LoRa forwarding
- LoRa to Serial forwarding
- Buffer management

---

### 3. **HardwareInit** (HardwareInit.cpp/h)
**Responsibility**: Low-level hardware initialization

**Public Interface**:
```cpp
namespace HardwareInit {
    void initGPIO();
    void initSPI();
    void initUART();
    void initAll();
    
    spi_device_handle_t getSPIHandle();
}
```

**Encapsulates**:
- GPIO configuration
- SPI bus setup
- UART driver installation
- Pin mapping

---

### 4. **Utils** (Utils.cpp/h)
**Responsibility**: Common utility functions

**Public Interface**:
```cpp
namespace Utils {
    void logHexDump(const uint8_t* data, size_t len, const char* prefix);
    void delayMicroseconds(uint32_t us);
    void printBanner();
}
```

---

## Benefits of Modularization

### ✅ Testability
- Each module can be unit tested independently
- Mock objects can be created for hardware interfaces
- Easier to write integration tests

### ✅ Maintainability
- Changes isolated to specific modules
- Clear ownership and responsibility
- Easier code reviews

### ✅ Reusability
- RadioManager can be used in other LoRa projects
- SerialBridge can be adapted for different protocols
- HardwareInit can be modified per board variant

### ✅ Scalability
- Easy to add new features:
  - Add WiFi module → Create WiFiBridge.cpp
  - Add display → Create DisplayManager.cpp
  - Add sensors → Create SensorManager.cpp
- Parallel development possible

### ✅ Debugging
- Module-level logging
- Clear error boundaries
- Easier to isolate issues

---

## Migration Strategy

### Phase 1: Create Module Skeletons
1. Create header files with interfaces
2. Move declarations from main.cpp
3. No functional changes yet

### Phase 2: Move Implementation
1. Start with HardwareInit (least dependencies)
2. Then Utils (simple functions)
3. Then RadioManager (core functionality)
4. Finally SerialBridge (depends on RadioManager)

### Phase 3: Refactor main.cpp
1. Remove all moved code
2. Keep only application logic
3. Instantiate and orchestrate modules

### Phase 4: Testing
1. Compile and test each phase
2. Verify functionality unchanged
3. Fix any integration issues

---

## Example: Refactored main.cpp (After Modularization)

```cpp
#include "Config.h"
#include "HardwareInit.h"
#include "RadioManager.h"
#include "SerialBridge.h"
#include "Utils.h"

extern "C" void app_main()
{
    // Print banner
    Utils::printBanner();
    
    // Initialize hardware
    HardwareInit::initAll();
    
    // Initialize radio
    RadioManager& radio = RadioManager::getInstance();
    radio.init();
    radio.startReceive();
    
    // Start serial bridge
    SerialBridge bridge(radio);
    bridge.start();
    
    ESP_LOGI("BOSMAG", "=== BosMag Ready ===");
    
    // Main loop (idle)
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

**Result**: main.cpp reduced from 600+ lines to ~30 lines!

---

## Additional Improvements

### 1. **Design Patterns to Apply**

- **Singleton**: RadioManager (only one radio instance)
- **Factory**: Radio module selection (SX1262, SX1268, etc.)
- **Observer**: Event notifications (RX done, TX done)
- **Strategy**: Different RF switch configurations

### 2. **Error Handling**

- Return error codes instead of ESP_LOGE only
- Custom exception classes (if using C++ exceptions)
- Graceful degradation

### 3. **Configuration Management**

- Runtime configuration via UART commands
- Store/load config from NVS (non-volatile storage)
- Multiple profiles (short-range, long-range, etc.)

### 4. **Logging Framework**

- Per-module log levels
- Log to SD card or flash
- Remote logging via LoRa

### 5. **Power Management**

- Sleep modes between transmissions
- Dynamic TX power adjustment
- Battery monitoring

---

## Implementation Checklist

- [ ] Create module header files
- [ ] Implement HardwareInit module
- [ ] Implement Utils module
- [ ] Implement RadioManager module
- [ ] Implement SerialBridge module
- [ ] Refactor main.cpp
- [ ] Update CMakeLists.txt
- [ ] Test compilation
- [ ] Test functionality
- [ ] Update documentation
- [ ] Create module-specific tests

---

## File Structure After Modularization

```
LoraOne/
├── include/
│   ├── Config.h                 # [Existing] Configuration
│   ├── EspHal.h                 # [Existing] RadioLib HAL
│   ├── RadioManager.h           # [New] Radio interface
│   ├── SerialBridge.h           # [New] Bridge interface
│   ├── HardwareInit.h           # [New] Hardware setup
│   └── Utils.h                  # [New] Utilities
├── src/
│   ├── main.cpp                 # [Modified] Application entry
│   ├── RadioManager.cpp         # [New] Radio implementation
│   ├── SerialBridge.cpp         # [New] Bridge implementation
│   ├── HardwareInit.cpp         # [New] Hardware implementation
│   ├── Utils.cpp                # [New] Utilities implementation
│   └── CMakeLists.txt           # [Modified] Build config
├── test/
│   ├── test_RadioManager.cpp    # [New] Unit tests
│   ├── test_SerialBridge.cpp    # [New] Unit tests
│   └── test_HardwareInit.cpp    # [New] Unit tests
└── README.md                    # [Updated] Documentation
```

---

## Estimated Effort

- **Phase 1** (Skeletons): 1-2 hours
- **Phase 2** (Implementation): 3-4 hours
- **Phase 3** (Refactor main): 1 hour
- **Phase 4** (Testing): 2-3 hours
- **Total**: ~8-10 hours

---

## Next Steps

1. Review and approve this plan
2. Create a feature branch: `git checkout -b feature/modularization`
3. Start with Phase 1
4. Commit after each module
5. Test thoroughly before merging to main

---

**Status**: 📋 Planning Complete - Ready for Implementation
