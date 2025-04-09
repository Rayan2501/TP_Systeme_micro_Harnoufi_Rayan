#include "pilote_led.h"

// DÃ©claration externe du SPI
extern SPI_HandleTypeDef hspi3;

// --- Fonction interne ---
static void MCP23S17_WriteRegister(uint8_t reg, uint8_t value) {
	// Adresse + bit d'Ã©criture
	uint8_t data[3];
    data[0] = MCP23S17_ADDRESS;
    data[1] = reg;
    data[2] = value;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CS LOW
    HAL_SPI_Transmit(&hspi3, data, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);   // CS HIGH
}

// --- Fonctions publiques ---

void LED_Driver_Init(void) {
    // RESET via GPIO
    HAL_GPIO_WritePin(GPIOA, VU_nReset_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(GPIOA, VU_nReset_Pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // Init MCP23S17
    MCP23S17_WriteRegister(MCP23S17_IOCON, 0x08);  // Mode SPI hardware
    MCP23S17_WriteRegister(MCP23S17_IODIRA, 0x00); // PORTA en sortie
    MCP23S17_WriteRegister(MCP23S17_IODIRB, 0x00); // PORTB en sortie
    MCP23S17_WriteRegister(MCP23S17_GPIOA, 0xFF);  // Eteindre LEDs
    MCP23S17_WriteRegister(MCP23S17_GPIOB, 0xFF);
}

void LED_SetGPIOA(uint8_t value) {
    MCP23S17_WriteRegister(MCP23S17_GPIOA, value);
}

void LED_SetGPIOB(uint8_t value) {
    MCP23S17_WriteRegister(MCP23S17_GPIOB, value);
}

void LED_SetPin(uint8_t pinNumber, GPIO_PinState state) {
    static uint8_t gpioA = 0xFF;
    static uint8_t gpioB = 0xFF;

    if (pinNumber < 8) {
        if (state == GPIO_PIN_RESET)
            gpioA &= ~(1 << pinNumber); // LED ON (active low)
        else
            gpioA |= (1 << pinNumber);  // LED OFF
        LED_SetGPIOA(gpioA);
    } else if (pinNumber < 16) {
        pinNumber -= 8;
        if (state == GPIO_PIN_RESET)
            gpioB &= ~(1 << pinNumber);
        else
            gpioB |= (1 << pinNumber);
        LED_SetGPIOB(gpioB);
    }
}


