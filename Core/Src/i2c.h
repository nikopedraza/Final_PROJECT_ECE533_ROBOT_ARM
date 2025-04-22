/*
 * i2c.h
 *
 *  Created on: Apr 12, 2025
 *      Author: ianv
 */
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#define SDA_PIN 9 //PB9
#define SCL_PIN 8 //PB8
#define INT_PIN 6 //PC6
#define RCC_CFGR_SW_HSI16 (2 << 0)
#define MCP23008_GPPU  (0x06)
#define MCP23008_I2C_ADDRESS_1  0b100000   /* Base address if A0,A1,A2 = GND   */
#define MCP23008_I2C_ADDRESS_2  0b100001   /* Base address if A0,A1,A2 = GND   */
#define MCP23008_IODIR        (0x00)   /* I/O Direction Register           */
#define MCP23008_GPINTEN  (0x02)    // Interrupt-on-change enable register
#define MCP23008_INTCON   (0x04)    // Interrupt control register
#define MCP23008_IODIR_INIT   (0x00) // sets all the expanders as outputs
#define MCP23008_GPIO         (0x09)   /* GPIO Register for read/write     */
#define JOYSTICK_L_SADDR		(0x63)
#define JOYSTICK_R_SADDR		(0x64)
#define JOYSTICK_8BIT_REG   (0x10)

#define delayms(ms) for(int i =0;i<ms*16000;i++)
// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

void initI2C();
void writeI2C(uint8_t addr, uint8_t *data, uint8_t length);
uint8_t readI2C(uint8_t addr, uint8_t reg);
void MCP23008_Init();
void MCP23008_WriteRegBlocking(uint8_t slaveaddr,uint8_t regAddr, uint8_t value);
void MCP23008_ReadRegBlocking(uint8_t slaveaddr,uint8_t regAddr,uint8_t length,uint8_t *buffer);
void processExpanderChange();

#endif

