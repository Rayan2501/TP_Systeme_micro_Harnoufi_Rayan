#ifndef INC_DRIVER_LED_H_
#define INC_DRIVER_LED_H_

#include "main.h"

// Definitions des registres du MCP23S17
#define MCP23S17_ADDRESS  0x40  // Adresse SPI du MCP23S17 (A0, A1, A2 = GND)
#define MCP23S17_IOCON    0x0A  // Registre de configuration
#define MCP23S17_IODIRA   0x00  // Registre direction PORTA
#define MCP23S17_IODIRB   0x01  // Registre direction PORTB
#define MCP23S17_GPIOA    0x12  // Registre des donnees PORTA
#define MCP23S17_GPIOB    0x13  // Registre des donnees PORTB

void LED_Driver_Init(void);
void LED_SetGPIOA(uint8_t value);
void LED_SetGPIOB(uint8_t value);
void LED_SetPin(uint8_t pinNumber, GPIO_PinState state);


#endif /* INC_DRIVER_LED_H_ */
