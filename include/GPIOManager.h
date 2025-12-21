#ifndef GPIO_MANAGER_H
#define GPIO_MANAGER_H

// ============================================================================
// GPIO MANAGER CLASS
// ============================================================================
class GPIOManager {
public:
    static bool init();
    
private:
    static void configurePins();
};

#endif // GPIO_MANAGER_H