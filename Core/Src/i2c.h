/*
 * i2c.h
 *
 *  Created on: Apr 12, 2025
 *      Author: ianv
 */
#ifndef I2C_H
#define I2C_H

#include <stdint.h>

void initI2C();
void writeI2C(uint8_t addr, uint8_t *data, uint8_t length);
uint8_t readI2C(uint8_t addr, uint8_t reg);

#endif

