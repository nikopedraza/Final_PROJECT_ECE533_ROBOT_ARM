/*
 * i2c.c
 *
 *  Created on: Apr 12, 2025
 *      Author: ianv
 */

#include "stm32l552xx.h"
#include "stdio.h"
#include "i2c.h"


void initI2C() {
	// Enable I2C clk
	RCC->APB1ENR1 |= (0b1 << 21); // enable i2c

	// Enable GPIO clk
	RCC->AHB2ENR |= (0b1 << 1); // enable gpiob
//	RCC->AHB2ENR |= (0b1 << 2); // enable gpioc

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
//	bitset(GPIOC->MODER, 12);

	// open drain pb8 and pb9
	GPIOB->OTYPER |= 0b11 << 8;

//	//enable internl pullups for pb8 and 9
	GPIOB->PUPDR |= (0b0101 << 2*8);

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

void MCP23008_Init(void)
{
    /* IODIR = 0x00 => lower nibble inputs, upper nibble outputs */
	MCP23008_WriteRegBlocking(MCP23008_I2C_ADDRESS_1,MCP23008_IODIR, MCP23008_IODIR_INIT);

	MCP23008_WriteRegBlocking(MCP23008_I2C_ADDRESS_2,MCP23008_IODIR, MCP23008_IODIR_INIT);



}

void MCP23008_WriteRegBlocking(uint8_t slaveaddr, uint8_t regAddr, uint8_t value)
{
    // Wait until I2C bus is free
    while(I2C1->ISR & I2C_ISR_BUSY);

    // Configure the transfer:
    // - Device 7-bit address is shifted left by 1
    // - Write mode (RD_WRN = 0)
    // - 2 bytes to be transmitted (NBYTES = 2)
    // - Generate start condition (START bit)
    I2C1->CR2 = 0;
    I2C1->CR2 |= (slaveaddr << 1);
	I2C1->CR2 |= (0 << I2C_CR2_RD_WRN_Pos);
	I2C1->CR2 |= (2 << I2C_CR2_NBYTES_Pos);
	I2C1->CR2 |= (1 << I2C_CR2_AUTOEND_Pos);

    I2C1->CR2 |= I2C_CR2_START;

    // Wait for TXIS flag to indicate data can be written
    while(!(I2C1->ISR & I2C_ISR_TXE));

    // Write the register address into TXDR
    I2C1->TXDR = regAddr;

    while(!(I2C1->ISR & I2C_ISR_TXE));

    // Write the data byte into TXDR
    I2C1->TXDR = value;

    while (bitcheck(I2C1->ISR, 0) != 1);
    // Wait until the STOPF (stop flag) is set
    while(!(I2C1->ISR & I2C_ISR_STOPF));

    // Clear the stop flag by writing to ICR
//    I2C1->ICR = I2C_ICR_STOPCF;
}


/**
 * @brief  Read a single byte from a register of the MCP23008 using blocking (poll) mode.
 * @param  regAddr: The MCP23008 register address to read from.
 * @return The data byte read from the register.
 *
 * This function performs a two-stage transaction:
 *  1. A write stage that sends the register address.
 *  2. A repeated start in read mode that reads one byte from the specified register.
 */
uint8_t MCP23008_ReadRegBlocking(uint8_t slaveaddr,uint8_t regAddr)
{
    uint8_t data = 0;

    // Wait until I2C bus is free
    while(I2C1->ISR & I2C_ISR_BUSY);



    /***** Stage 1: Write the register address *****/
    // Configure the transfer:
    // - 7-bit address, write mode (RD_WRN = 0)
    // - 1 byte to send (the register address)
    // - Generate start condition
    I2C1->CR2 = 0;
    I2C1->CR2 |= (slaveaddr << 1);
	I2C1->CR2 |= (0 << I2C_CR2_RD_WRN_Pos);
    I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
    I2C1->CR2 |= (1 << I2C_CR2_AUTOEND_Pos);

	I2C1->CR2 |= I2C_CR2_START;
    // Wait for TXIS flag (ready to transmit)
    while(!(I2C1->ISR & I2C_ISR_TXE));

    // Send the register address byte
    I2C1->TXDR = regAddr;

    while (bitcheck(I2C1->ISR, 0) != 1);                 // Wait for the transmit buffer to be empty.
    while (bitcheck(I2C1->ISR, 2) == 1){
		   (void)I2C1->RXDR;
    }


    /***** Stage 2: Read from the register *****/
    // Now configure a new transfer with a repeated start:
    // - 7-bit address, read mode (RD_WRN = 1)
    // - 1 byte to read
    // - Generate start condition (repeated start)
    I2C1->CR2 = 0;
    I2C1->CR2 |= (slaveaddr << 1);
    I2C1->CR2 |= (1 << I2C_CR2_RD_WRN_Pos);
	I2C1->CR2 |= (1 << I2C_CR2_NBYTES_Pos);
	I2C1->CR2 |= (1 << I2C_CR2_AUTOEND_Pos);

    I2C1->CR2 |= I2C_CR2_START;

    // Wait until RXNE flag is set (data received)
    while(!(I2C1->ISR & I2C_ISR_RXNE));

    // Read the received byte from RXDR
    data = I2C1->RXDR;


    // Wait for the STOPF flag indicating the end of the transfer
    while(!(I2C1->ISR & I2C_ISR_STOPF));

    // Clear the stop flag
   // I2C1->ICR = I2C_ICR_STOPCF;

    return data;
}







