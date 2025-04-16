#include "stm32l552xx.h"
#include "stdio.h"
#include "i2c.h"


#define SERVO_PIN = 5; // in GPIOC-
#define SCALE_MOTOR 2 /// random value atm will configure
#define RCC_CFGR_SW_HSI16 (2 << 0)


void init_sysclk();
void init_TIM2();
void init_TIM3();
void init_TIM4();
void next_step();
void updateMotorFromIterator();


const uint8_t step_sequence[8] = {
	0b10001000,
	0b11001100,
	0b01000100,
	0b01100110,
	0b00100010,
	0b00110011,
	0b00010001,
	0b10011001
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
	uint8_t saddr;
	uint8_t pos ;
	uint8_t data;
}Motor;



// initalize the stepper iterators
StepIterator base_iter = {
		.current_index = 0
};
StepIterator lower_iter = {
		.current_index = 1
};
StepIterator upper_iter = {
		.current_index = 2
};

Motor base = {
	.saddr = MCP23008_I2C_ADDRESS_1,  // MSet the motor port to GPIOA.
	.pos = 0b11110000,    // upper nibble of byte GP7-4
	.data = 0x00,

};
Motor lower = {
	.saddr = MCP23008_I2C_ADDRESS_1,  // Set the motor port to GPIOA.
	.pos = 0b00001111,
	.data = 0x00
};
Motor upper = {
	.saddr = MCP23008_I2C_ADDRESS_2,  // Set the motor port to GPIOA.
	.pos = 0b11110000,
	.data = 0x00
};


int main(void) {

	 init_sysclk();
//	 init_TIM3();
//	 init_TIM4();
	 initI2C();
	 MCP23008_Init();
	 init_TIM2();

// DEMO CODE FOR TESTING THE DC MOTOR SETUP


	while(1){

		// any thing polling ?
//		processExpanderChange();
//
//
//		next_step(&upper_iter);
//		updateMotorFromIterator(&upper,&upper_iter);
//
//		next_step(&lower_iter);
//		updateMotorFromIterator(&lower,&lower_iter);
//
//		next_step(&base_iter);
//		updateMotorFromIterator(&base,&base_iter);

//		delayms(125);

	}
}



void init_sysclk(){
    // Enable the internal high-speed oscillator (HSI).
    // This turns on the HSI oscillator.
    RCC->CR |= RCC_CR_HSION;

    // Wait until the HSI oscillator is stable and ready.
    // The HSIRDY flag in RCC->CR will be set when HSI is ready.
    while ((RCC->CR & RCC_CR_HSIRDY) == 0) {
        /* Wait for HSI ready */
    }

    // Clear the current system clock switch (SW) bits in RCC->CFGR.
    // These bits determine the current system clock source.
    RCC->CFGR &= ~RCC_CFGR_SW;
    RCC->CFGR |= RCC_CFGR_SW_HSI16;

}

void queery_joystick_xy(uint8_t addr, uint8_t *joy_xy) {
	joy_xy[0] = readI2C(addr, 0x10);
	joy_xy[1] = readI2C(addr, 0x11);

}
uint8_t get_current_step(StepIterator *it)
{
    return step_sequence[it->current_index];
}


void next_step(StepIterator *it)
{
    it->current_index = (it->current_index + 1) % 8;
}

void prev_step(StepIterator *it)
{
    it->current_index = (it->current_index + 7) % 8;
}


void updateMotorFromIterator(Motor *motor, StepIterator *iterator)
{
    // Get a pointer to the current step array (4 bytes).
    uint8_t step = get_current_step(iterator);

    // Update the Motor struct fields accordingly.

    // Update the Motor struct fields accordingly.
     motor->data |= (motor->pos & step);

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
    TIM2->ARR = 4999; // 1M/1000 = 1khz timer

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
    TIM4->ARR = 124999;  // Generates update event at 1kHz.

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

	// any thing polling ?
		processExpanderChange();


		next_step(&upper_iter);
		updateMotorFromIterator(&upper,&upper_iter);

		next_step(&lower_iter);
		updateMotorFromIterator(&lower,&lower_iter);

		next_step(&base_iter);
		updateMotorFromIterator(&base,&base_iter);
//	next_step(&base_iter);
//
//	updateMotorFromIterator(&base, &base_iter);
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

void processExpanderChange(void)
{
    uint8_t newGPIO_1;
    uint8_t newGPIO_2;

    // An array of Motor structures for demonstration.
        Motor motors[] = {
            base,
            upper,
            lower };


        // This variable will hold the data from the Motor that matches the target saddr.
        newGPIO_1 = 0x00;
        newGPIO_2 = 0x00;

        // Compute the number of elements in the motors array.
        size_t numMotors = sizeof(motors) / sizeof(motors[0]);

        // Loop through each Motor instance.
        for (size_t i = 0; i < numMotors; i++) {
            if (motors[i].saddr == MCP23008_I2C_ADDRESS_1) {

            	newGPIO_1 |= motors[i].data;

            }
            else if(motors[i].saddr == MCP23008_I2C_ADDRESS_2){
            	newGPIO_2 |= motors[i].data;
            }
        }


    // Combine the lower nibble (DIP switch state) with the new LED state.
    // Since the lower nibble corresponds to input pins (and writing to them is ignored),
    // you can also just write the LED portion. However, for clarity, we combine them.
    // The new GPIO value keeps the lower nibble intact and updates the upper nibble.


    // Write the new GPIO value back to the MCP23008.
    // This updates the LED outputs to follow the DIP switch state.
    MCP23008_WriteRegBlocking(MCP23008_I2C_ADDRESS_1,MCP23008_GPIO, newGPIO_1);
    MCP23008_WriteRegBlocking(MCP23008_I2C_ADDRESS_2,MCP23008_GPIO, newGPIO_2);

}

