/*
 * i2c.c
 *
 *  Created on: Apr 12, 2025
 *      Author: ianv
 */

#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

void initI2C() {
	// Enable I2C clk
	RCC->APB1ENR1 |= (0b1 << 21); // enable i2c

	// Enable GPIO clk
	RCC->AHB2ENR |= (0b1 << 1); // enable gpiob
	RCC->AHB2ENR |= (0b1 << 2); // enable gpioc

	// AF mode for PB8
	bitclear(GPIOB->MODER, 16);
	bitset(GPIOB->MODER, 17);

	// AF mode for PB9
	bitclear(GPIOB->MODER, 18);
	bitset(GPIOB->MODER, 19);

	// set mode 4 for PB8
	bitset(GPIOB->AFR[1], 2);

	// set mode 4 for PB9
	bitset(GPIOB->AFR[1], 6);

	// set output mode for pc6
	bitset(GPIOC->MODER, 12);

	// open drain pb8 and pb9
	GPIOB->OTYPER |= 0b11 << 8;

	// very high speed pb8 and pb9
	GPIOB->OSPEEDR |= 0b1111 << 16;
	GPIOC->OSPEEDR |= 0b11 << 12;

	// Reset I2C state machine so it is idle state
	I2C1->CR1 |= 0b0;

	// Program I2C timing based on target speed (100kHz)
	I2C1->TIMINGR |= 3 << 28; //PRESC
	I2C1->TIMINGR |= 0x13; // SCLL
	I2C1->TIMINGR |= 0xF << 8; // SCLH
	I2C1->TIMINGR |= 0x2 << 16; // SDADEL
	I2C1->TIMINGR |= 0x4 << 20; // SCLDEL

	// enable I2C
	I2C1->CR1 |= 0b1;
}

void writeI2C(uint8_t addr, uint8_t *data, uint8_t length) {

	I2C1->CR2 = 0;
	// 1 - Make sure that the I2C peripheral is not busy
	while(bitcheck(I2C1->ISR,15));
	// 2 - Write slave address (SADDR)
	I2C1->CR2 |= addr << 1;
	// 3 - Write type of transaction (RD_WRN=0)
	I2C1->CR2 |= 0b0 << 10;
	// 4 - Configure auto-end if needed (AUTOEND)
	I2C1->CR2 |= 0b1 << 25; //auto end mode on
	// 5 - Configure number of bytes to be sent (NBYTES)
	I2C1->CR2 |= length << 16;
	// 6 - Issue the START condition
	I2C1->CR2 |= 0b1 << 13;

	for(int i = 0; i < length; i++) {
		while(!bitcheck(I2C1->ISR,0)); // wait for TX ready flag
		I2C1->TXDR = data[i];
	}
	// Check that the data reg is empty (TXIS or TXE)
	while(!bitcheck(I2C1->ISR,5));

	return;
}

uint8_t readI2C(uint8_t addr, uint8_t reg) {

	writeI2C(addr, &reg, 1);

	uint8_t data;
	I2C1->CR2 = 0;
	// 1 - Make sure that the peripheral is not busy
	while(bitcheck(I2C1->ISR,15));
	// 2 - Write slave address (SADDR)
	I2C1->CR2 |= addr << 1;
	// 3 - Write type of transaction (RD_WRN=1)
	I2C1->CR2 |= 0b1 << 10;
	// 4 - Configure auto-end if needed (AUTOEND)
	I2C1->CR2 |= 0b1 << 25; //auto end mode on
	// 5 - Configure number of bytes to be sent (NBYTES)
	I2C1->CR2 |= 1 << 16;
	// 6 - Issue the START condition
	I2C1->CR2 |= 0b1 << 13;
	// 7 - Check that there is a slave with a matching address (ADDR)
	//while(bitcheck(I2C1->ISR,3)==0);
	// 8 - Issue ACK or NACK
	//I2C1->CR2 = (0b1 << 15);
	// 9 - Wait until RXNE
	while(!bitcheck(I2C1->ISR,2));
	// 10 - Read I2C_RXDR
	data = I2C1->RXDR;

	return data;
}

