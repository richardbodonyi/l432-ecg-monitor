#ifndef INC_TASK_MANAGER_H_
#define INC_TASK_MANAGER_H_

#include "stm32l4xx_hal.h"

#define BUTTON_PRESS 0
#define RIGHT_TURN 1
#define LEFT_TURN 2

void init_tasks(
    SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac);

void manage_tasks();

void task_handle_rotary_change(int32_t value);

void task_handle_interrupt(uint8_t interrupt_event);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif /* INC_TASK_MANAGER_H_ */
