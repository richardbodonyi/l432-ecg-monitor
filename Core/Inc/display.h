
#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#define BUTTON_PRESS 0
#define RIGHT_TURN 1
#define LEFT_TURN 2

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac);

void display_graph();

void reset_values();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

void increase_brightness();

void decrease_brightness();

void display_handle_rotary_change(int32_t value);

void display_handle_interrupt(uint8_t interrupt);

#endif /* INC_DISPLAY_H_ */
