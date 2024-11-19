#include "main.h"
#include "ili9341_gfx.h"
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "display.h"
#include "ad_header.h"
#include "stm32l4xx_hal_dac.h"
#include "signal_processing.h"

#define VERSION "1.0"

#define MOD_INDEX(x) ((x + BUFFER_SIZE) % BUFFER_SIZE)

#define RR_TO_PULSE(x) (60 * 200 / x)

#define MAX_HEIGHT 239

#define MENU_ITEM_PAUSE 0
#define MENU_ITEM_SOUND 1
#define MENU_ITEM_BACK 2

#define RULER_TICK_Y1 225
#define SEC_RULER_TICK_Y2 210
#define HALF_SEC_RULER_TICK_Y2 217
#define SEC_MOD 800
#define HALF_SEC_MOD 400

#define FILTERED_DC_SHIFT(X) ((X / 50) + 2000)
#define DERIVATIVE_DC_SHIFT(X) ((X / 10000) + 1500)
#define DC_SHIFT(X) ((X / 1000000) + 2000)

#define GRAPH_Y1 50
#define GRAPH_Y2 160

#define MIN_Y 700
#define MAX_Y 3000

#define TEXT_COLOR ILI9341_LIGHTGREY
#define TEXT_BACKGROUND ILI9341_BLACK
#define HIGHLIGHTED_TEXT_COLOR ILI9341_DARKGREY
#define HIGHLIGHTED_TEXT_BACKGROUND ILI9341_BLUE
#define RULER_COLOR ILI9341_DARKGREY
#define RAW_SIGNAL_COLOR ILI9341_DARKGREY
#define FILTERED_SIGNAL_COLOR ILI9341_GREEN
#define QRS_COLOR ILI9341_RED

#define MENU_SIZE 3

char* EVALUATION_TEXTS[] = {"Normal", "Type1", "Type2"};

char* MENU_TEXTS[] = {"Szunet", "Hang", "Vissza"};

char* DEBUG_TEXTS[] = {"< 0", "<= 1000" ">1000", ">2000", ">3000", ">4000"};

char DEBUG_TEXT[20];

typedef enum {
  MEASURE = 0,
  MENU
} T_Mode;

typedef struct {
  uint8_t selected;
} T_Menu;

DAC_HandleTypeDef* hdac_hal;

TIM_HandleTypeDef* timer_hal;

ili9341_t* ili9341_lcd;

uint16_t dma_values[0];

uint16_t raw_values[BUFFER_SIZE] = {0};

uint32_t time_buffer[BUFFER_SIZE] = {0};

float filtered[BUFFER_SIZE] = {0};

//float squared_derivative[BUFFER_SIZE] = {0};

float integral[BUFFER_SIZE] = {0};

uint32_t fill_index = 0;

uint32_t current_index = 0;

pt_result_t result;

uint8_t lcd_brightness = 130;

bool active = false, enabled = true, paused = false;

T_Mode mode = MEASURE;

T_Menu menu;

int16_t rotary_position = 0;

const uint8_t MIN_BRIGHTNESS = 80, MAX_BRIGHTNESS = 250, BRIGHTNESS_STEP = 10;

ili9341_text_attr_t MENU_TEXT_ATTR;

ili9341_text_attr_t PULSE_TEXT_ATTR;

char* get_debug_text(uint32_t index) {
  if (index > 4000) {
    return DEBUG_TEXTS[5];
  }
  else if (index > 3000) {
    return DEBUG_TEXTS[4];
  }
  else if (index > 2000) {
    return DEBUG_TEXTS[3];
  }
  else if (index > 1000) {
    return DEBUG_TEXTS[2];
  }
//  else if (index < 0) {
//    return DEBUG_TEXTS[0];
//  }
  return DEBUG_TEXTS[1];
}

void print_debug_info(uint32_t value) {
  ili9341_text_attr_t attr;
  attr.bg_color = TEXT_BACKGROUND;
  attr.fg_color = TEXT_COLOR;
  attr.font = &ili9341_font_11x18;
  attr.origin_x = 100;
  attr.origin_y = 12;
  sprintf(DEBUG_TEXT, "%ld", value);
  ili9341_draw_string(ili9341_lcd, attr, DEBUG_TEXT);
}

void init_display(SPI_HandleTypeDef* spi,
    TIM_HandleTypeDef* timer,
    ADC_HandleTypeDef* adc,
	DAC_HandleTypeDef* hdac) {
  hdac_hal = hdac;
  HAL_DAC_Start(hdac_hal, DAC_CHANNEL_1);
  HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
  timer_hal = timer;
  ili9341_lcd = ili9341_new(
          spi,
          TFT_RESET_GPIO_Port, TFT_RESET_Pin,
          TFT_CS_GPIO_Port,    TFT_CS_Pin,
          TFT_DC_GPIO_Port,    TFT_DC_Pin,
          isoLandscape,
          TOUCH_CS_GPIO_Port,  TOUCH_CS_Pin,
          TOUCH_IRQ_GPIO_Port, TOUCH_IRQ_Pin,
          itsSupported,
          itnNormalized);
  ili9341_spi_tft_select(ili9341_lcd);
  ili9341_fill_screen(ili9341_lcd, TEXT_BACKGROUND);
  ili9341_text_attr_t attr;
  attr.bg_color = TEXT_BACKGROUND;
  attr.fg_color = TEXT_COLOR;
  attr.font = &ili9341_font_16x26;
  attr.origin_x = 60;
  attr.origin_y = 100;
  ili9341_draw_string(ili9341_lcd, attr, "EKG MONITOR");
  attr.font = &ili9341_font_11x18;
  attr.origin_x = 120;
  attr.origin_y = 150;
  ili9341_draw_string(ili9341_lcd, attr, VERSION);

  MENU_TEXT_ATTR.fg_color = TEXT_COLOR;
  MENU_TEXT_ATTR.font = &ili9341_font_11x18;

  PULSE_TEXT_ATTR.bg_color = TEXT_BACKGROUND;
  PULSE_TEXT_ATTR.fg_color = TEXT_COLOR;
  PULSE_TEXT_ATTR.font = &ili9341_font_11x18;
  PULSE_TEXT_ATTR.origin_x = 260;
  PULSE_TEXT_ATTR.origin_y = 12;

  enableAD();

  HAL_ADC_Start_DMA(adc, (uint32_t*) dma_values, 1);
}

uint16_t translate_y(uint16_t value) {
  return GRAPH_Y1 + GRAPH_Y2 - 1 - (value - MIN_Y) * (float) GRAPH_Y2 / (MAX_Y - MIN_Y);
}

void print_pulse(uint16_t pulse) {
  char pulse_text[20];
  sprintf(pulse_text, "%d", pulse);
  ili9341_draw_string(ili9341_lcd, PULSE_TEXT_ATTR, pulse_text);
}

void draw_menu() {
  // return;
  uint8_t x = 10, y = 10;
  for (uint8_t i = 0; i < MENU_SIZE; i++) {
    MENU_TEXT_ATTR.bg_color = menu.selected == i ? HIGHLIGHTED_TEXT_BACKGROUND : TEXT_BACKGROUND;
    MENU_TEXT_ATTR.origin_x = x;
    MENU_TEXT_ATTR.origin_y = y + i * 18;
    ili9341_draw_string(ili9341_lcd, MENU_TEXT_ATTR, MENU_TEXTS[i]);
  }
}

void display_graph() {
  if (enabled) {
    uint16_t draw_index, previous_draw_index, x;
    while (fill_index > current_index) {
      active = true;
      if (!paused) {
        draw_index = MOD_INDEX(current_index);
        previous_draw_index = MOD_INDEX(draw_index - 1);
        x = current_index % ili9341_lcd->screen_size.width;
        ili9341_draw_line(ili9341_lcd, TEXT_BACKGROUND, x, 0, x, ili9341_lcd->screen_size.height - 1);
        // draw ruler
        uint32_t current_time = time_buffer[draw_index];
        if (current_time % SEC_MOD < 5) {
          ili9341_draw_line(ili9341_lcd, ILI9341_DARKGREY, x, SEC_RULER_TICK_Y2, x, RULER_TICK_Y1);
        }
        else if (current_time % HALF_SEC_MOD < 5) {
          ili9341_draw_line(ili9341_lcd, ILI9341_DARKGREY, x, HALF_SEC_RULER_TICK_Y2, x, RULER_TICK_Y1);
        }

        // draw raw signal
        if (x == 0) {
          ili9341_draw_pixel(ili9341_lcd, RAW_SIGNAL_COLOR, x, translate_y(raw_values[draw_index]));
        }
        else {
          ili9341_draw_line(ili9341_lcd, RAW_SIGNAL_COLOR, x - 1, translate_y(raw_values[previous_draw_index]), x, translate_y(raw_values[draw_index]));
        }

        process_pan_tompkins(raw_values, filtered, integral, current_index, &result);

        if (x == 0) {
          ili9341_draw_pixel(ili9341_lcd, FILTERED_SIGNAL_COLOR, x, translate_y(FILTERED_DC_SHIFT(filtered[draw_index])));
        }
        else {
          ili9341_draw_line(ili9341_lcd, FILTERED_SIGNAL_COLOR, x - 1, translate_y(FILTERED_DC_SHIFT(filtered[previous_draw_index])), x, translate_y(FILTERED_DC_SHIFT(filtered[draw_index])));
        }

//        ili9341_color_t color = result.is_qrs ? QRS_COLOR : FILTERED_SIGNAL_COLOR;

        // draw signals
//        if (x == 0) {
////          ili9341_draw_pixel(ili9341_lcd, ILI9341_MAGENTA, x, translate_y(FILTERED_DC_SHIFT(filtered[draw_index])));
////          ili9341_draw_pixel(ili9341_lcd, ILI9341_CYAN, x, translate_y(DERIVATIVE_DC_SHIFT(squared_derivative[draw_index])));
//  //        ili9341_draw_pixel(ili9341_lcd, ILI9341_YELLOW, x, translate_y(DERIVATIVE_DC_SHIFT(derivative[draw_index])));
//          ili9341_draw_pixel(ili9341_lcd, color, x, translate_y(DC_SHIFT(integral[draw_index])));
//        }
//        else {
////          ili9341_draw_line(ili9341_lcd, ILI9341_MAGENTA, x - 1, translate_y(FILTERED_DC_SHIFT(filtered[previous_draw_index])), x, translate_y(FILTERED_DC_SHIFT(filtered[draw_index])));
////          ili9341_draw_line(ili9341_lcd, ILI9341_CYAN, x - 1, translate_y(DERIVATIVE_DC_SHIFT(squared_derivative[previous_draw_index])), x, translate_y(DERIVATIVE_DC_SHIFT(squared_derivative[draw_index])));
//  //        ili9341_draw_line(ili9341_lcd, ILI9341_YELLOW, x - 1, translate_y(DERIVATIVE_DC_SHIFT(derivative[previous_draw_index])), x, translate_y(DERIVATIVE_DC_SHIFT(derivative[draw_index])));
//          ili9341_draw_line(ili9341_lcd, color, x - 1, translate_y(DC_SHIFT(integral[previous_draw_index])), x, translate_y(DC_SHIFT(integral[draw_index])));
//          ili9341_draw_pixel(ili9341_lcd, ILI9341_YELLOW, x, translate_y(DC_SHIFT(result.signalpeaki)));
//          ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(DC_SHIFT(result.thi1)));
//  //        ili9341_draw_pixel(ili9341_lcd, ILI9341_LIGHTGREY, x, translate_y(DC_SHIFT(result.derivative)));
//          ili9341_draw_pixel(ili9341_lcd, ILI9341_RED, x, translate_y(DC_SHIFT(result.peaki)));
//        }
        if (result.is_qrs) {
          ili9341_draw_line(ili9341_lcd, ILI9341_RED, x, 10, x, 230);
        }

        if (result.rr_average > 0) {
          print_pulse(RR_TO_PULSE((float) result.rr_average));
        }
      }
      current_index++;
//      rotary_index = rotary_index % ili9341_lcd->screen_size.width;
//      ili9341_draw_line(ili9341_lcd, ILI9341_CYAN, rotary_index, 1, rotary_index, rotary_values[rotary_index] % 240);
//      rotary_index++;
    }
//    ili9341_draw_line(ili9341_lcd, ILI9341_CYAN, 0, rotary_position, 99, rotary_position);
    if (mode == MENU) {
      draw_menu();
    }
  }
}

void reset_values() {
	disableAD();
	active = false;
}

void increase_brightness() {
	lcd_brightness += BRIGHTNESS_STEP;
	if (lcd_brightness > MAX_BRIGHTNESS) {
		lcd_brightness = MAX_BRIGHTNESS;
	}
	HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
}

void decrease_brightness() {
	lcd_brightness -= BRIGHTNESS_STEP;
	if (lcd_brightness < MIN_BRIGHTNESS) {
		lcd_brightness = MIN_BRIGHTNESS;
	}
	HAL_DAC_SetValue(hdac_hal, DAC_CHANNEL_1, DAC_ALIGN_8B_R, lcd_brightness);
}

void button_turned_right() {
  if (mode == MENU) {
    menu.selected++;
    if (menu.selected >= MENU_SIZE) {
      menu.selected = MENU_SIZE - 1;
    }
  }
}

void button_turned_left() {
  if (mode == MENU) {
    if (menu.selected > 0) {
      menu.selected--;
    }
    else {
      menu.selected = 0;
    }
  }
}

void display_handle_rotary_change(int32_t value) {
  if (mode == MENU) {
    menu.selected = (menu.selected + value) % MENU_SIZE;
    if (menu.selected >= MENU_SIZE) {
      menu.selected = MENU_SIZE - 1;
    }
  }
}

void display_handle_button_press() {
  if (mode == MEASURE) {
      mode = MENU;
    }
    else {
      switch (menu.selected) {
        case MENU_ITEM_PAUSE:
          paused = !paused;
          break;
        default: mode = MEASURE;
      }
    }
}

void display_shutdown() {
  enabled = false;
  disableAD();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM16) {
    if (enabled) {
      raw_values[MOD_INDEX(fill_index)] = dma_values[0];
      time_buffer[MOD_INDEX(fill_index)] = HAL_GetTick();
      fill_index++;
    }
    else {
      HAL_TIM_Base_Stop_IT(htim);
    }
  }
  else if (htim->Instance == TIM7) {
    HAL_TIM_Base_Stop_IT(htim);
    display_shutdown();
  }
}
