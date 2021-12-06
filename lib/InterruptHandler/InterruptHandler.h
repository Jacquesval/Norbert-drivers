#ifndef INTERRUPT_HANDLER_H
#define INTERRUPT_HANDLER_H

#define PORT_NUMBER 3
#define MAX_INTERUPT_FUNCTION_NUMBER 8

#define PORT_B 0
#define PORT_C 1
#define PORT_D 2


#define PORT_B_RANGE_MIN 8
#define PORT_B_RANGE_MAX 13
#define PORT_C_RANGE_MIN 14
#define PORT_C_RANGE_MAX 19
#define PORT_D_RANGE_MIN 0
#define PORT_D_RANGE_MAX 7


typedef void (*InterruptFunctions) (void);

class InterruptHandler{
private:
    static int counter;
    InterruptHandler();
    ~InterruptHandler();

    bool m_isPortActivate[PORT_NUMBER];
    static InterruptHandler* interruptHandlerInstance;
    
    bool activatePinChangeInterrupt(uint8_t interruptPin, uint8_t port);
    bool deactivatePinChangeInterrupt(uint8_t interruptPin, uint8_t port);
    bool activateInterruptOnPort(uint8_t port);
    bool deactivateInterruptOnPort(uint8_t port);
    static int8_t digitalToInterruptPin(uint8_t digitalPin, uint8_t &port);


public:
    static bool m_isPinInterruptEnable[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER];
    static int m_interruptPin[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER];
    static bool m_pinStates[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER];
    static InterruptFunctions m_interruptFunctions[PORT_NUMBER][2 * MAX_INTERUPT_FUNCTION_NUMBER];

    static InterruptHandler *getInstance();
    static void releaseInstance();

    bool attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode);
    bool detachInterrupt(uint8_t pin);

    
};


#endif