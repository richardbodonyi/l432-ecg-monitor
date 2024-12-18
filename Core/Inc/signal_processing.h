
#ifndef SIGNAL_PROCESSING_H_
#define SIGNAL_PROCESSING_H_

#include "stm32l4xx_hal.h"

#define SAMPLING_FREQUENCY 200          // Sampling frequency.

#define BUFFER_SIZE 500l // The size of the buffers (in samples). Must fit more than 1.66 times an RR interval, which
                         // typically could be around 1 second.

// Structure of the result of Pan-Tompkins algorithm.
typedef struct {
  float peaki;
  float signalpeaki;
  float noisepeaki;
  float thi1;
  bool is_qrs;
  uint16_t rr_average;
  uint16_t rr_average2;
  bool is_regular;
  uint8_t evaluation;
} pt_result_t;

void process_pan_tompkins(uint16_t* signal, float* filtered, uint32_t current_index, pt_result_t* result);

#endif /* SIGNAL_PROCESSING_H_ */
