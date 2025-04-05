#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.


#define SERVO_PIN = 5; // in GPIOC

const uint8_t step_sequence[8][4] = {
	{1, 0, 0, 0},
	{1, 1, 0, 0},
	{0, 1, 0, 0},
	{0, 1, 1, 0},
	{0, 0, 1, 0},
	{0, 0, 1, 1},
	{0, 0, 0, 1},
	{1, 0, 0, 1}
};

// reverse = -1, stationary =0, positve = 1
uint8_t base_dir = 0;

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
	.in1 = 5,
	.in2 = 6,
	.in3 = 7,
	.in4 = 8
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

void tim2_iniit() {
	// initialization
	TIM2->ARR = 1000;
	TIM2->PSC = 16;
}

void i2c_init() {

}

void write_motor(Motor m, uint8_t val) {

}

// highest priority intterrupt
void I2C_IRQHandler() {
	// 4 values
	x1; // base

	y1; // lower arm

	x2; // servo

	y2; // upper arm


	// 256 base
	TIM->ARR = 1000;

	// 128 base
	TIM2->ARR = 2000;
}

void TIM2_IRQHandler() {
	write_motor(base, val);
}
void TIM3_IRQHandler() {
	write_motor(lower, val);
}
void TIM4_IRQHandler() {
	write_motor(upper, val);
}

void TIM5_IRQHandler () { // servo pwm control
	// -90

	// 0

	//90
}
