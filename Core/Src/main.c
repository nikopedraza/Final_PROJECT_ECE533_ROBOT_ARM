#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.

const uint8_t step_sequence[8][4] = [
	[1, 0, 0, 0],
	[1, 1, 0, 0],
	[0, 1, 0, 0],
	[0, 1, 1, 0],
	[0, 0, 1, 0],
	[0, 0, 1, 1],
	[0, 0, 0, 1],
	[1, 0, 0, 1]
];

uint8_t base_sequence_position = 0;
uint8_t lower_sequence_position = 0;
uint8_t upper_sequence_position = 0;

typedef struct {
	GPIO_TypeDef *port;
	uint8_t in1;
	uint8_t in2;
	uint8_t in3;
	uint8_t in4;
}Motor;


// Base
const Motor base = {
	.port = GPIOA,
	.in1 = ;
	.in2 = ;
	.in3 = ;
	.in4 = ;
}

// Upper
const Motor upper = {
	.port = GPIOA,
	.in1 = ;
	.in2 = ;
	.in3 = ;
	.in4 = ;
}

// Lower
const Motor lower = {
	.port = GPIOA,
	.in1 = ;
	.in2 = ;
	.in3 = ;
	.in4 = ;
}

// Grabber

void main() {

	while();
}

void TIM2_IRQHandler() {
	write_motor(base);
	write_motor(upper);
	write_motor(lower)
}
