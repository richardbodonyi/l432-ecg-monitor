#include "task_manager.h"
#include "display.h"

void init_tasks(
    SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac) {

  init_display(spi, timer, adc, hdac);
  HAL_TIM_Base_Start_IT(timer);
}

void manage_tasks() {
  display_graph();
}

void task_handle_rotary_change(int32_t value) {
  display_handle_rotary_change(value);
}

void task_handle_interrupt(uint8_t interrupt_event) {
  display_handle_interrupt(interrupt_event);
}
