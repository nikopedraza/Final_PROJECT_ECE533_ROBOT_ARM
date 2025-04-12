#include "stm32l552xx.h"
#include "stdio.h"

// Some helper macros
#define bitset(word,   idx)  ((word) |=  (1<<(idx))) //Sets the bit number <idx> -- All other bits are not affected.
#define bitclear(word, idx)  ((word) &= ~(1<<(idx))) //Clears the bit number <idx> -- All other bits are not affected.
#define bitflip(word,  idx)  ((word) ^=  (1<<(idx))) //Flips the bit number <idx> -- All other bits are not affected.
#define bitcheck(word, idx)  ((word>>idx) &   1    ) //Checks the bit number <idx> -- 0 means clear; !0 means set.
#define SERVO_PIN = 5; // in GPIOC-
#define SCALE_MOTOR 2 /// random value atm will configure

void init_TIM2();
void init_TIM3();
void init_TIM4();

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
    uint8_t current_index;  // Keeps track of the current step index.
} StepIterator;


typedef struct {
	GPIO_TypeDef *port;
	uint8_t in1;
	uint8_t in2;
	uint8_t in3;
	uint8_t in4;
}Motor;



// initalize the stepper iterators
StepIterator base_iter = {0};
StepIterator lower_iter = {0};
StepIterator upper_iter = {0};

Motor base = {
	.port = GPIOA,  // Set the motor port to GPIOA.
	.in1 = 0,
	.in2 = 0,
	.in3 = 0,
	.in4 = 0
};
Motor lower = {
	.port = GPIOA,  // Set the motor port to GPIOA.
	.in1 = 0,
	.in2 = 0,
	.in3 = 0,
	.in4 = 0
};
Motor upper = {
	.port = GPIOA,  // Set the motor port to GPIOA.
	.in1 = 0,
	.in2 = 0,
	.in3 = 0,
	.in4 = 0
};


int main(void) {

	 init_TIM2();
	 init_TIM3();
	 init_TIM4();








	while(1){

		// any thing polling ?

	}
}

void tim2_iniit() {
	// initialization
	TIM2->ARR = 1000;
	TIM2->PSC = 16;
}

void i2c_init() {

}


const uint8_t *get_current_step(StepIterator *it)
{
    return step_sequence[it->current_index];
}


void next_step(StepIterator *it)
{
    it->current_index = (it->current_index + 1) % 8;
}


void updateMotorFromIterator(Motor *motor, StepIterator *iterator)
{
    // Get a pointer to the current step array (4 bytes).
    const uint8_t *step = get_current_step(iterator);

    // Update the Motor struct fields accordingly.

    // Update the Motor struct fields accordingly.
      motor->in1 = step[0];
      motor->in2 = step[1];
      motor->in3 = step[2];
      motor->in4 = step[3];

}

// highest priority intterrupt
void I2C_IRQHandler() {
	// 4 values //256 based
	uint8_t x1; // base

	uint8_t y1; // lower arm

	uint8_t x2; // servo

	uint8_t y2; // upper arm

	// update the reload value with
	 TIM2->ARR = 999 - (SCALE_MOTOR *x1 );
	 TIM3->ARR = 999 - (SCALE_MOTOR *y1 );
	 TIM4->ARR = 999 - (SCALE_MOTOR *y2 );
}

/**
 * @brief FOR BASE MOTOR SPEED
 */

void init_TIM2(void)
{
    // 1. Enable clock for TIM2 (located on APB1 bus)
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;

    // 2. Configure the timer prescaler:
    //    PSC = 15 => Timer clock = 16MHz / 16 = 1MHz.
    TIM2->PSC = 15;

    // 3. Set the auto-reload register (ARR) for a 1 kHz update rate.
    TIM2->ARR = 999;

    // 4. Reset the counter.
    TIM2->CNT = 0;

    // 5. Enable auto-reload preload to buffer ARR updates.
    TIM2->CR1 |= TIM_CR1_ARPE;

    // 6. Enable the Update Interrupt.
     TIM2->DIER |= TIM_DIER_UIE;

    // 6. Start the counter.
    TIM2->CR1 |= TIM_CR1_CEN;

    // 8. Enable TIM2 interrupt in the NVIC.
    NVIC_EnableIRQ(TIM2_IRQn);
}


/**
 * @brief FOR LOWER ARM MOTOR SPEED
 */
void init_TIM3(void)
{
    // 1. Enable clock for TIM3 (located on APB1 bus)
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN;

    // 2. Set prescaler and ARR (same settings as TIM2 for a 1 kHz rate).
    TIM3->PSC = 15;   // Timer clock = 16MHz/16 = 1MHz.
    TIM3->ARR = 999;  // Counter counts 0..999 -> update every 1 ms.

    // 3. Clear the counter.
    TIM3->CNT = 0;

    // 4. Enable auto-reload preload.
    TIM3->CR1 |= TIM_CR1_ARPE;

    // 6. Enable the update interrupt.
    TIM3->DIER |= TIM_DIER_UIE;

    // 5. Start TIM3.
    TIM3->CR1 |= TIM_CR1_CEN;

    // 8. Enable TIM3 NVIC interrupt.
    NVIC_EnableIRQ(TIM3_IRQn);
}


/**
 * @brief FOR  UPPER ARM MOTOR
 */
void init_TIM4(void)
{
    // 1. Enable clock for TIM4 (located on APB1 bus)
    RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;

    // 2. Set prescaler and auto-reload register values.
    TIM4->PSC = 15;   // Divides 16MHz to 1MHz.
    TIM4->ARR = 999;  // Generates update event at 1kHz.

    // 3. Reset counter value.
    TIM4->CNT = 0;

    // 4. Enable auto-reload preload.
    TIM4->CR1 |= TIM_CR1_ARPE;

    TIM4->DIER |= TIM_DIER_UIE;

    // 5. Start TIM4.
    TIM4->CR1 |= TIM_CR1_CEN;

    NVIC_EnableIRQ(TIM4_IRQn);
}


void TIM2_IRQHandler() {
	// Clear the update interrupt flag
	TIM2->SR &= ~TIM_SR_UIF;
	next_step(&base_iter);

	updateMotorFromIterator(&base, &base_iter);
}
void TIM3_IRQHandler() {
	// Clear the update interrupt flag
	TIM3->SR &= ~TIM_SR_UIF;
	next_step(&lower_iter);

	updateMotorFromIterator(&lower, &lower_iter);
}
void TIM4_IRQHandler() {
	// Clear the update interrupt flag
	TIM4->SR &= ~TIM_SR_UIF;
	next_step(&upper_iter);
	updateMotorFromIterator(&upper, &upper_iter);
}

void TIM5_IRQHandler () { // servo pwm control
	// -90

	// 0

	//90
}

