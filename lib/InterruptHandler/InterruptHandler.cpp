#include <avr/interrupt.h>
#include "Arduino.h"
#include "InterruptHandler.h"

//setup static variables
int InterruptHandler::counter = 0; 
bool m_isPinInterruptEnable[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER] = {{false}};
bool m_pinStates[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER] = {{false}};
InterruptFunctions m_interruptFunctions[PORT_NUMBER][2 * MAX_INTERUPT_FUNCTION_NUMBER] = {{NULL}};
int m_interruptPin[PORT_NUMBER][MAX_INTERUPT_FUNCTION_NUMBER] = {{0}};


InterruptHandler::InterruptHandler():m_isPortActivate{false, false, false}{
    for(int i=0; i < PORT_NUMBER; ++i){
        for(int j=0; j<(2 * MAX_INTERUPT_FUNCTION_NUMBER); ++j){
            m_interruptFunctions[i][j] = NULL;
        }
    }
    for(int i=0; i < PORT_NUMBER; ++i){
        for(int j=0; j<(MAX_INTERUPT_FUNCTION_NUMBER); ++j){
            m_isPinInterruptEnable[i][j] = false;
            m_interruptPin[i][j] = -1;
            m_pinStates[i][j] = LOW;
        }
    }

}

InterruptHandler::~InterruptHandler(){};

InterruptHandler *InterruptHandler::getInstance(){

    if(interruptHandlerInstance == NULL){
        interruptHandlerInstance = new InterruptHandler();
    }
    ++counter;

    return interruptHandlerInstance;
}

void InterruptHandler::releaseInstance(){
    --counter;
    if(counter == 0 && interruptHandlerInstance != NULL){
        delete interruptHandlerInstance;
        interruptHandlerInstance = NULL;
    }

}


int8_t InterruptHandler::digitalToInterruptPin(uint8_t digitalPin, uint8_t &port){
    
    if(digitalPin <= PORT_D_RANGE_MAX ){
        port = PORT_D;
        return (digitalPin - PORT_D_RANGE_MIN);
    }
    else if (digitalPin <= PORT_B_RANGE_MAX){
        port = PORT_B;
        return (digitalPin - PORT_B_RANGE_MIN);
    }
    else if (digitalPin <= PORT_C_RANGE_MAX){
        port = PORT_C;
        return (digitalPin - PORT_C_RANGE_MIN);
    }
    else{
        return -1;
    }
}

bool InterruptHandler::activateInterruptOnPort(uint8_t port){
    if(port < PORT_NUMBER){
        PCICR |= (0b00000001 << port);
        m_isPortActivate[port] = true;
        return true;
    }
    else{
        return false;
    }
}
bool InterruptHandler::deactivateInterruptOnPort(uint8_t port){
    if(port < PORT_NUMBER){
        PCICR &= ~(0b00000001 << port);
        m_isPortActivate[port] = false;
        return true;
    }
    else{
        return false;
    }
}

bool InterruptHandler::activatePinChangeInterrupt(uint8_t interruptPin, uint8_t port){
    bool isActivate = true;
    if(port < PORT_NUMBER){
        if(!m_isPortActivate[port]){
            activateInterruptOnPort(port);
        }
        if(m_isPortActivate[port]){
            switch (port)
            {
            case PORT_B:
                PCMSK0 |= (0b00000001 << interruptPin);
                break;
            case PORT_C:
                PCMSK1 |= (0b00000001 << interruptPin);
                break;
            case PORT_D:
                PCMSK2 |= (0b00000001 << interruptPin);
                break;
            }
        }
        else {
            isActivate = false;
        }
    }
    else{
        isActivate = false;
    }


    return isActivate;
}

bool InterruptHandler::deactivatePinChangeInterrupt(uint8_t interruptPin, uint8_t port){
    bool isDeactivate = true;
    bool shouldDeactivatePort = false;
    switch (port){
    case PORT_B:
        PCMSK0 &= ~(0b00000001 << interruptPin);
        if ((PCMSK0&0b00111111) > 0){
            shouldDeactivatePort = true;
        }
        break;
    case PORT_C:
        PCMSK1 &= ~(0b00000001 << interruptPin);
        if ((PCMSK1&0b00111111) > 0){
            shouldDeactivatePort = true;
        }
        break;
    case PORT_D:
        PCMSK2 &= ~(0b00000001 << interruptPin);
        if ((PCMSK2&0b11111111) > 0){
            shouldDeactivatePort = true;
        }
        break;
    default:
        isDeactivate = false;
    }
    if(shouldDeactivatePort){
        deactivateInterruptOnPort(port);
    }
    return isDeactivate;
}

bool InterruptHandler::attachInterrupt(uint8_t pin, void (*userFunc)(void), int mode){
    uint8_t port;
    int8_t interruptPin = digitalToInterruptPin(pin, port);
    if (interruptPin < 0){
        return false;
    }
    if(!activatePinChangeInterrupt(interruptPin, port)){
        return false;
    }

    m_isPinInterruptEnable[(int) port][interruptPin] = true;
    m_interruptPin[(int) port][interruptPin] = (int) pin;

    switch (mode)
    {
    case RISING:
        m_interruptFunctions[port][2 * interruptPin] = userFunc;
        break;

    case FALLING:
        m_interruptFunctions[port][2 * interruptPin + 1] = userFunc;
        break;

    case CHANGE:
        m_interruptFunctions[port][2 * interruptPin] = userFunc;
        m_interruptFunctions[port][2 * interruptPin + 1] = userFunc;
        break;
    
    default:
        break;
    }

    return true;
}

bool InterruptHandler::detachInterrupt(uint8_t pin){
    uint8_t port;
    int8_t interruptPin = digitalToInterruptPin(pin, port);
    if (interruptPin < 0){
        return false;
    }
    if(!deactivatePinChangeInterrupt(interruptPin, port)){
        return false;
    }

    m_isPinInterruptEnable[(int) port][interruptPin] = false;
    m_interruptPin[(int) port][interruptPin] = (int) pin;
    return true;

}

// ISR(PCINT0_vect){
//     for(int i = 0; i < PORT_B_RANGE_MAX - PORT_B_RANGE_MIN + 1 ; ++i){
//         if(InterruptHandler::m_isPinInterruptEnable[PORT_B][i]){
//             if(digitalRead(InterruptHandler::m_interruptPin[PORT_B][i]) && !InterruptHandler::m_pinStates[PORT_B][i]){
//                 InterruptHandler::m_pinStates[PORT_B][i] = HIGH;
//                 InterruptHandler::m_interruptFunctions[PORT_B][2*i]();
//             }
//             else if (!digitalRead(InterruptHandler::m_interruptPin[2][i]) && InterruptHandler::m_pinStates[PORT_B][i]){
//                 InterruptHandler::m_pinStates[PORT_B][i] = LOW;
//                 InterruptHandler::m_interruptFunctions[PORT_B][2*i + 1]();
//             }
//         }
//     }
// }

// ISR(PCINT1_vect){
//     for(int i = 0; i < PORT_C_RANGE_MAX - PORT_C_RANGE_MIN + 1 ; ++i){
//         if(InterruptHandler::m_isPinInterruptEnable[PORT_C][i]){
//             if(digitalRead(InterruptHandler::m_interruptPin[PORT_C][i]) && !InterruptHandler::m_pinStates[PORT_C][i]){
//                 InterruptHandler::m_pinStates[PORT_C][i] = HIGH;
//                 InterruptHandler::m_interruptFunctions[PORT_C][2*i]();
//             }
//             else if (!digitalRead(InterruptHandler::m_interruptPin[2][i]) && InterruptHandler::m_pinStates[PORT_C][i]){
//                 InterruptHandler::m_pinStates[PORT_C][i] = LOW;
//                 InterruptHandler::m_interruptFunctions[PORT_C][2*i + 1]();
//             }
//         }
//     }
// }

// ISR(PCINT2_vect){
//     for(int i = 0; i < PORT_D_RANGE_MAX - PORT_D_RANGE_MIN + 1 ; ++i){
//         if(InterruptHandler::m_isPinInterruptEnable[PORT_D][i]){
//             if(digitalRead(InterruptHandler::m_interruptPin[PORT_D][i]) && !InterruptHandler::m_pinStates[PORT_D][i]){
//                 InterruptHandler::m_pinStates[PORT_D][i] = HIGH;
//                 InterruptHandler::m_interruptFunctions[PORT_D][2*i]();
//             }
//             else if (!digitalRead(InterruptHandler::m_interruptPin[2][i]) && InterruptHandler::m_pinStates[PORT_D][i]){
//                 InterruptHandler::m_pinStates[PORT_D][i] = LOW;
//                 InterruptHandler::m_interruptFunctions[PORT_D][2*i + 1]();
//             }
//         }
//     }
// }
