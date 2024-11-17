#include "stm32l4xx_hal.h"
#include <stdbool.h>
#include "signal_processing.h"

/**
 * ------------------------------------------------------------------------------*
 * File: panTompkins.c                                                           *
 *       ANSI-C implementation of Pan-Tompkins real-time QRS detection algorithm *
 * Author: Rafael de Moura Moreira <rafaelmmoreira@gmail.com>                    *
 * License: MIT License                                                          *
 * ------------------------------------------------------------------------------*
 * ---------------------------------- HISTORY ---------------------------------- *
 *    date   |    author    |                     description                    *
 * ----------| -------------| ---------------------------------------------------*
 * 2019/04/11| Rafael M. M. | - Fixed moving-window integral.                    *
 *           |              | - Fixed how to find the correct sample with the    *
 *           |              | last QRS.                                          *
 *           |              | - Replaced constant value in code by its #define.  *
 *           |              | - Added some casting on comparisons to get rid of  *
 *           |              | compiler warnings.                                 *
 * 2019/04/15| Rafael M. M. | - Removed delay added to the output by the filters.*
 *           |              | - Fixed multiple detection of the same peak.       *
 * 2019/04/16| Rafael M. M. | - Added output buffer to correctly output a peak   *
 *           |              | found by back searching using the 2nd thresholds.  *
 * 2019/04/23| Rafael M. M. | - Improved comparison of slopes.                   *
 *           |              | - Fixed formula to obtain the correct sample from  *
 *           |              | the buffer on the back search.                     *
 * ------------------------------------------------------------------------------*
 * MIT License                                                                   *
 *                                                                               *
 * Copyright (c) 2018 Rafael de Moura Moreira                                    *
 *                                                                               *
 * Permission is hereby granted, free of charge, to any person obtaining a copy  *
 * of this software and associated documentation files (the "Software"), to deal *
 * in the Software without restriction, including without limitation the rights  *
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell     *
 * copies of the Software, and to permit persons to whom the Software is         *
 * furnished to do so, subject to the following conditions:                      *
 *                                                                               *
 * The above copyright notice and this permission notice shall be included in all*
 * copies or substantial portions of the Software.                               *
 *                                                                               *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    *
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      *
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE   *
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER        *
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, *
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE *
 * SOFTWARE.                                                                     *
 *-------------------------------------------------------------------------------*
 * Description                                                                   *
 *                                                                               *
 * The main goal of this implementation is to be easy to port to different opera-*
 * ting systems, as well as different processors and microcontrollers, including *
 * embedded systems. It can work both online or offline, depending on whether all*
 * the samples are available or not - it can be adjusted on the input function.  *
 *                                                                               *
 * The main function, panTompkings(), calls input() to get the next sample and   *
 * store it in a buffer. Then it runs through a chain of filters: DC block, low  *
 * pass @ 15 Hz and high pass @ 5Hz. The filtered signal goes both through a de- *
 * rivative filter, which output is then squared, and through a windowed-integra-*
 * tor.                                                                          *
 *                                                                               *
 * For a signal peak to be recognized as a fiducial point, its correspondent va- *
 * lue on both the filtered signal and the integrator must be above a certain    *
 * threshold. Additionally, there are time-restraints to prevent a T-wave from   *
 * being mistakenly identified as an R-peak: a hard 200ms restrain (a new peak   *
 * 200ms from the previous one is, necessarily, a T-wave) and a soft 360ms res-  *
 * train (the peak's squared slope must also be very high to be considered as a  *
 * real peak).                                                                   *
 *                                                                               *
 * When a peak candidate is discarded, its value is used to update the noise     *
 * thresholds - which are also used to estimate the peak thresholds.             *
 *                                                                               *
 * Two buffers keep 8 RR-intervals to calculate RR-averages: one of them keeps   *
 * the last 8 RR-intervals, while the other keeps only the RR-intervals that res-*
 * pect certain restrictions. If both averages are equal, the heart pace is con- *
 * sidered normal. If the heart rate isn't normal, the thresholds change to make *
 * it easier to detect possible weaker peaks. If no peak is detected for a long  *
 * period of time, the thresholds also change and the last discarded peak candi- *
 * date is reconsidered.                                                         *
 *-------------------------------------------------------------------------------*
 * Instructions                                                                  *
 *                                                                               *
 * Here's what you should change to adjust the code to your needs:               *
 *                                                                               *
 * On panTompkins.h:                                                             *
 * - typedef int dataType;                                                       *
 * Change it from 'int' to whatever format your data is (float, unsigned int etc)*
 *                                                                               *
 * On both panTompkins.h and panTompkins.c:                                      *
 * - void init(char file_in[], char file_out[]);                                 *
 * This function is meant to do any kind of initial setup, such as starting a    *
 * serial connection with an ECG sensor. Change its parameters to whatever info  *
 * you need and its content. The test version included here loads 2 plain text   *
 * files: an input file, with the signal as a list of integer numbers in ASCII   *
 * format and an output file where either 0 or 1 will be written for each sample,*
 * whether a peak was detected or not.                                           *
 *                                                                               *
 * On panTompkins.c:                                                             *
 * - #define WINDOW_SIZE                                                         *
 * Defines the size of the integration window. The original authors suggest on   *
 * their 1985 paper a 150ms window.                                              *
 *                                                                               *
 * - #define SAMPLING_FREQUENCY                                                                  *
 * Defines the sampling frequency.                                               *
 *                                                                               *
 * - #define NOSAMPLE                                                            *
 * A value to indicate you don't have any more samples to read. Choose a value   *
 * which a sample couldn't possibly have (e.g.: a negative value if your A/D con-*
 * verter only works with positive integers).                                    *
 *                                                                               *
 * - #define BUFFER_SIZE                                                         *
 * The size of the signal buffers. It should fit at least 1.66 times an RR-inter-*
 * val. Heart beats should be between 60 and 80 BPS for humans. So, considering  *
 * 1.66 times 1 second should be safe.                                           *
 *                                                                               *
 * - #define DELAY 22                                                            *
 * The delay introduced to the output signal. The first DELAY samples will be ig-*
 * nored, as the filters add a delay to the output signal, causing a mismatch    *
 * between the input and output signals. It's easier to compare them this way.   *
 * If you need them both to have the same amount of samples, set this to 0. If   *
 * you're working with different filters and/or sampling rates, you might need to*
 * adjust this value.                                                            *
 *                                                                               *
 * - #include <stdio.h>                                                          *
 * The file, as it is, both gets its inputs and sends its outputs to files. It   *
 * works on both Windows and Linux. If your source isn't a file, and/or your sys-*
 * tem doesn't have the <stdio.h> header, remove it.                             *
 * Include any other headers you might need to make your implementation work,    *
 * such as hardware libraries provided by your microcontroller manufacturer.     *
 *                                                                               *
 * - The input() function                                                        *
 * Change it to get the next sample from your source (a file, a serial device etc*
 * previously set up in your init() function. Return the sample value or NOSAMPLE*
 * if there are no more available samples.                                       *
 *                                                                               *
 * - The output() function                                                       *
 * Change it to output whatever you see fit, however you see fit: an RR-interval *
 * (which can be sent as a parameter to your function using the RR arrays), the  *
 * index of sample or timestamp which caused a R peak, whether a sample was a R  *
 * peak or not etc, and it can be written on a file, displayed on screen, blink a*
 * LED etc.                                                                      *
 *                                                                               *
 * - The panTompkins() function                                                  *
 * This function is almost entirely ANSI C, which means it should work as is on  *
 * most platforms. The only lines you really have to change are the fclose() ones*
 * at the very end, which are only here to allow testing of the code on systems  *
 * such as Windows and Linux for PC. You may wish to create extra variables or   *
 * increase the buffers' size as you see fit to extract different kinds of infor-*
 * mation to output, or fine tune the detection as you see fit - such as adding  *
 * different conditions for verification, initializing self-updating variables   *
 * with empirically-obtained values or changing the filters.                     *
 *-------------------------------------------------------------------------------*
 */

#define WINDOW_SIZE 30   // Integrator window size, in samples. The article recommends 150ms. So, SAMPLING_FREQUENCY*0.15.
            // However, you should check empirically if the waveform looks ok.
//#define NOSAMPLE -32000 // An indicator that there are no more samples to read. Use an impossible value for a sample.
#define SAMPLING_FREQUENCY 200          // Sampling frequency.

//#define HIGHPASS_INTEGRAL_DELAY 7    // The delay between the filtered sample and the integral. volt már 1, 2, 3, 4, 5, 6, 7, 8, 9, 10

#define MOD_INDEX(x) ((x + BUFFER_SIZE) % BUFFER_SIZE)

#define DELAY_200ms_IN_SAMPLES 40 // SAMPLING_FREQUENCY / 5

#define DELAY_360ms_IN_SAMPLES 72 // 0.36 * SAMPLING_FREQUENCY

//#define DELAYED_INDEX(x) ((x + HIGHPASS_INTEGRAL_DELAY + BUFFER_SIZE) % BUFFER_SIZE)

// The signal array is where the most recent samples are kept. The other arrays are the outputs of each
// filtering module: DC Block, low pass, high pass, integral etc.
int16_t dcblock[BUFFER_SIZE] = {0};
float lowpass[BUFFER_SIZE] = {0},
//  highpass[BUFFER_SIZE] = {0},
  derivative[BUFFER_SIZE] = {0};
//  squared_derivative[BUFFER_SIZE] = {0};
//  integral[BUFFER_SIZE] = {0};

// i and j are iterators for loops.
// sample counts how many samples have been read so far.
// lastQRS stores which was the last sample read when the last R sample was triggered.
// lastSlope stores the value of the squared slope when the last R sample was triggered.
// currentSlope helps calculate the max. square slope for the present sample.
// These are all long unsigned int so that very long signals can be read without messing the count.
int32_t i, j, k;

int32_t sample = 0, lastQRS = 0, lastSlope = 0, currentSlope = 0;

// rr1 holds the last 8 RR intervals. rr2 holds the last 8 RR intervals between rrlow and rrhigh.
// rravg1 is the rr1 average, rr2 is the rravg2. rrlow = 0.92*rravg2, rrhigh = 1.08*rravg2 and rrmiss = 1.16*rravg2.
// rrlow is the lowest RR-interval considered normal for the current_index heart beat, while rrhigh is the highest.
// rrmiss is the longest that it would be expected until a new QRS is detected. If none is detected for such
// a long interval, the thresholds must be adjusted.
uint16_t rr1[8], rr2[8], rravg1, rravg2, rrlow = 0, rrhigh = 200, rrmiss = 0;

// There are the variables from the original Pan-Tompkins algorithm.
// The ones ending in _i correspond to values from the integrator.
// The ones ending in _f correspond to values from the DC-block/low-pass/high-pass filtered signal.
// The peak variables are peak candidates: signal values above the thresholds.
// The threshold 1 variables are the threshold variables. If a signal sample is higher than this threshold, it's a peak.
// The threshold 2 variables are half the threshold 1 ones. They're used for a back search when no peak is detected for too long.
// The spk and npk variables are, respectively, running estimates of signal and noise peaks.
float peak_i = 0,
//    peak_f = 0,
    threshold_i1 = 0,
    threshold_i2 = 0,
//    threshold_f1 = 0,
//    threshold_f2 = 0,
    signalpeak_i = 0,
//    signalpeak_f = 0,
    noisepeak_i = 0;
//    noisepeak_f = 0;

// regular tells whether the heart pace is regular or not.
// prevRegular tells whether the heart beat was regular before the newest RR-interval was calculated.
bool regular = true, prevRegular;

uint32_t beat_count = 0;

/*
    This is the actual QRS-detecting function. It's a loop that constantly calls the input and output functions
    and updates the thresholds and averages until there are no more samples. More details both above and in
    shorter comments below.
    The output is a buffer where we can change a previous result (using a back search) before outputting.
*/
void process_pan_tompkins(uint16_t* signal, float* filtered, float* squared_derivative, float* integral, uint32_t current_index, pt_result_t* result) {

  // This variable is used as an index to work with the signal buffers. If the buffers still aren't
  // completely filled, it shows the last filled position. Once the buffers are full, it'll always
  // show the last position, and new samples will make the buffers shift, discarding the oldest
  // sample and storing the newest one on the last position.
  uint32_t array_index = MOD_INDEX(current_index);

  float* highpass = lowpass;

  // Initializing the RR averages
  for (i = 0; i < 8; i++) {
      rr1[i] = 0;
      rr2[i] = 0;
  }

  // The main loop where everything proposed in the paper happens. Ends when there are no more signal samples.
//  do {
        // Test if the buffers are full.
        // If they are, shift them, discarding the oldest sample and adding the new one at the end.
        // Else, just put the newest sample in the next free position.
        // Update 'current_index' so that the program knows where's the newest sample.

//    if (sample >= BUFFER_SIZE) {
//      for (i = 0; i < BUFFER_SIZE - 1; i++) {
//        signal[i] = signal[i + 1];
//        dcblock[i] = dcblock[i + 1];
//        lowpass[i] = lowpass[i + 1];
//        highpass[i] = highpass[i + 1];
//        derivative[i] = derivative[i + 1];
//        squared_derivative[i] = squared_derivative[i + 1];
//        integral[i] = integral[i + 1];
//        qrs_series[i] = qrs_series[i + 1];
//      }
//      current_index = BUFFER_SIZE - 1;
//    }
//    else {
//      current_index = sample;
//    }
//    signal[current_index] = input();

    // If no sample was read, stop processing!
//    if (signal[current_index] == NOSAMPLE)
//      break;
//    current_index = sample;
//    sample++; // Update sample counter
    sample = MOD_INDEX(array_index + 1l);
    // DC Block filter
    // This was not proposed on the original paper.
    // It is not necessary and can be removed if your sensor or database has no DC noise.
    if (current_index >= 1) {
      dcblock[array_index] = signal[array_index] - signal[MOD_INDEX(array_index - 1l)] + 0.995 * dcblock[MOD_INDEX(array_index - 1l)];
    }
    else {
      dcblock[array_index] = 0;
    }

    // Low Pass filter
    // Implemented as proposed by the original paper.
    // y(nT) = 2y(nT - T) - y(nT - 2T) + x(nT) - 2x(nT - 6T) + x(nT - 12T)
    // Can be removed if your signal was previously filtered, or replaced by a different filter.
    lowpass[array_index] = dcblock[array_index];
//    if (current_index >= 1) {
//      lowpass[array_index] += lowpass[MOD_INDEX(array_index - 1l)] << 1;
//    }
//    if (current_index >= 2) {
//      lowpass[array_index] -= lowpass[MOD_INDEX(array_index - 2l)];
//    }
//    if (current_index >= 6) {
//      lowpass[array_index] -= dcblock[MOD_INDEX(array_index - 6l)] << 1;
//    }
//    if (current_index >= 12) {
//      lowpass[array_index] += dcblock[MOD_INDEX(array_index - 12l)];
//    }
//    if (current_index >= 12) {
      lowpass[array_index] += 2 * lowpass[MOD_INDEX(array_index - 1)];
      lowpass[array_index] -= lowpass[MOD_INDEX(array_index - 2)];
      lowpass[array_index] -= 2 * dcblock[MOD_INDEX(array_index - 6)];
      lowpass[array_index] += dcblock[MOD_INDEX(array_index - 12)];
//    }

    // High Pass filter
    // Implemented as proposed by the original paper.
    // y(nT) = 32x(nT - 16T) - [y(nT - T) + x(nT) - x(nT - 32T)]
    // Can be removed if your signal was previously filtered, or replaced by a different filter.
//    highpass[array_index] = -lowpass[array_index];
//    if (current_index >= 1) {
//      highpass[array_index] -= highpass[MOD_INDEX(array_index - 1)];
//    }
//    if (current_index >= 16) {
//      highpass[array_index] += 32 * lowpass[MOD_INDEX(array_index - 16)];
//    }
//    if (current_index >= 32) {
//      highpass[array_index] += lowpass[MOD_INDEX(array_index - 32)];
//    }

    filtered[array_index] = highpass[array_index];

    // Derivative filter
    // This is an alternative implementation, the central difference method.
    // f'(a) = [f(a+h) - f(a-h)]/2h
    // The original formula used by Pan-Tompkins was:
    // y(nT) = (1/8T)[-x(nT - 2T) - 2x(nT - T) + 2x(nT + T) + x(nT + 2T)]
    derivative[array_index] = highpass[array_index] - highpass[MOD_INDEX(array_index - 1l)];
//    if (current_index > 0) {
//      derivative[array_index] -= highpass[MOD_INDEX(array_index - 1l)];
//    }

    // This just squares the derivative, to get rid of negative values and emphasize high frequencies.
    // y(nT) = [x(nT)]^2.
    squared_derivative[array_index] = derivative[array_index] * derivative[array_index];

    // Moving-Window Integration
    // Implemented as proposed by the original paper.
    // y(nT) = (1/N)[x(nT - (N - 1)T) + x(nT - (N - 2)T) + ... x(nT)]
    // WINDOW_SIZE, in samples, must be defined so that the window is ~150ms.

    integral[array_index] = 0;
    for (i = 0; i < WINDOW_SIZE; i++) {
      if (current_index >= i) {
        integral[array_index] += squared_derivative[MOD_INDEX(array_index - i)];
      }
      else {
        break;
      }
    }
    integral[array_index] /= i;
    uint32_t integral_value = integral[array_index];

    result->derivative = derivative[array_index];
    result->squared_derivative = squared_derivative[array_index];

    // Decision making.
    result->is_qrs = false;

    // If the array_index signal is above one of the thresholds (integral or filtered signal), it's a peak candidate.
    if (integral_value >= threshold_i1) {
        peak_i = integral_value;
    }

    // If both the integral and the signal are above their thresholds, they're probably signal peaks.
    if (integral_value >= threshold_i1) {
      // There's a 200ms latency. If the new peak respects this condition, we can keep testing.
      if (sample > lastQRS + DELAY_200ms_IN_SAMPLES) {
          // If it respects the 200ms latency, but it doesn't respect the 360ms latency, we check the slope.
        if (sample <= lastQRS + DELAY_360ms_IN_SAMPLES) {
          // The squared slope is "M" shaped. So we have to check nearby samples to make sure we're really looking
          // at its peak value, rather than a low one.

          // R R R

          signalpeak_i = 0.125 * peak_i + 0.875 * signalpeak_i; // 0.125 *
          threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
          threshold_i2 = 0.5 * threshold_i1;
          result->is_qrs = true;
          lastQRS = sample;
        }
        // If it was above both thresholds and respects both latency periods, it certainly is an R peak.
        else {
          // R R R
          signalpeak_i = 0.125 * peak_i + 0.875 * signalpeak_i; // 0.125 *
          threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
          threshold_i2 = 0.5 * threshold_i1; // 0.5 *
          result->is_qrs = true;
          result->peaki = peak_i;
          result->signalpeaki = signalpeak_i;
          result->noisepeaki = noisepeak_i;
          result->thi1 = threshold_i1;
          lastQRS = sample;
        }
      }
      // If the new peak doesn't respect the 200ms latency, it's noise. Update thresholds and move on to the next sample.
      else {
        // NOISE NOISE NOISE
        // peak_i = integral_value;
        noisepeak_i = 0.125 * peak_i + 0.875 * noisepeak_i;
        threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i);
        threshold_i2 = 0.5 * threshold_i1; // 0.5 *
        result->is_qrs = false;
        result->peaki = peak_i;
        result->signalpeaki = signalpeak_i;
        result->noisepeaki = noisepeak_i;
        result->thi1 = threshold_i1;
        return;
      }
    }

  // If a R-peak was detected, the RR-averages must be updated.
  if (false && result->is_qrs) {
    // Add the newest RR-interval to the buffer and get the new average.
    rravg1 = 0;
    for (i = 0; i < 7; i++) {
      rr1[i] = rr1[i + 1];
      rravg1 += rr1[i];
    }
    rr1[7] = sample - lastQRS;
    rravg1 += rr1[7];
    rravg1 = rravg1 >> 3; // * 0.125
    lastQRS = sample;

    // If the newly-discovered RR-average is normal, add it to the "normal" buffer and get the new "normal" average.
    // Update the "normal" beat parameters.
    if ((rr1[7] >= rrlow) && (rr1[7] <= rrhigh)) {
      rravg2 = 0;
      for (i = 0; i < 7; i++) {
        rr2[i] = rr2[i + 1];
        rravg2 += rr2[i];
      }
      rr2[7] = rr1[7];
      rravg2 += rr2[7];
      rravg2 = rravg2 >> 3; // * 0.125
      rrlow = 0.92 * rravg2;
      rrhigh = 1.16 * rravg2;
      rrmiss = 1.66 * rravg2;
    }

    prevRegular = regular;
    if (rravg1 == rravg2) {
      regular = true;
    }
    // If the beat had been normal but turned odd, change the thresholds.
    else {
      regular = false;
      if (prevRegular) {
        threshold_i1 = 0.5 * threshold_i1; // 0.5 *
      }
    }
  }
  // If no R-peak was detected, it's important to check how long it's been since the last detection.
//  else {
//    // TODO erre nem kerülhet sor, csak előre keresünk
//
//    // If no R-peak was detected for too long, use the lighter thresholds and do a back search.
//    // However, the back search must respect the 200ms limit and the 360ms one (check the slope).
//    if (false && (sample > (lastQRS + DELAY_200ms_IN_SAMPLES))) {
//      for (k = lastQRS - 1 + DELAY_200ms_IN_SAMPLES; k < current_index; k++) {
//        i = MOD_INDEX(k);
//        if ((integral[i] > threshold_i2)) {
//            peak_i = integral[i];
//            signalpeak_i = 0.25 * peak_i + 0.75 * signalpeak_i; // 0.25 *
//            threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//            threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//            lastSlope = currentSlope;
//            // If a signal peak was detected on the back search, the RR attributes must be updated.
//            // This is the same thing done when a peak is detected on the first try.
//            //RR Average 1
//            rravg1 = 0;
//            for (j = 0; j < 7; j++) {
//              rr1[j] = rr1[j + 1];
//              rravg1 += rr1[j];
//            }
//            rr1[7] = i - 1 - lastQRS;
//            lastQRS = i - 1;
//            rravg1 += rr1[7];
//            rravg1 = rravg1 >> 3; // * 0.125
//            result->is_qrs = true;
//
//            //RR Average 2
//            if ((rr1[7] >= rrlow) && (rr1[7] <= rrhigh)) {
//              rravg2 = 0;
//              for (i = 0; i < 7; i++) {
//                rr2[i] = rr2[i + 1];
//                rravg2 += rr2[i];
//              }
//              rr2[7] = rr1[7];
//              rravg2 += rr2[7];
//              rravg2 = rravg2 >> 3; // * 0.125
//              rrlow = 0.92 * rravg2;
//              rrhigh = 1.16 * rravg2;
//              rrmiss = 1.66 * rravg2;
//            }
//
//            prevRegular = regular;
//            if (rravg1 == rravg2) {
//              regular = true;
//            }
//            else {
//              regular = false;
//              if (prevRegular) {
//                threshold_i1 = 0.5 * threshold_i1; // 0.5 *
//              }
//            }
//            break;
////            }
//        }
//      }
//      if (result->is_qrs) {
//        result->is_regular = regular;
//        result->peaki = peak_i;
//        result->signalpeaki = signalpeak_i;
//        result->noisepeaki = noisepeak_i;
//        result->thi1 = threshold_i1;
//        result->pulse = rravg1;
//        return;
//      }
//    }
//
//    // Definitely no signal peak was detected.
//    if (!result->is_qrs) {
//      // If some kind of peak had been detected, then it's certainly a noise peak. Thresholds must be updated accordingly.
//      if (integral_value >= threshold_i1) {
//        peak_i = integral[array_index];
//        noisepeak_i = 0.125 * peak_i + 0.875 * noisepeak_i; // 0.125 *
//        threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//        threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//      }
//    }
//  }
  // The current_index implementation outputs '0' for every sample where no peak was detected,
  // and '1' for every sample where a peak was detected. It should be changed to fit
  // the desired application.
  // The 'if' accounts for the delay introduced by the filters: we only start outputting after the delay.
  // However, it updates a few samples back from the buffer. The reason is that if we update the detection
  // for the current_index sample, we might miss a peak that could've been found later by backsearching using
  // lighter thresholds. The final waveform output does match the original signal, though.
  result->peaki = peak_i;
  result->signalpeaki = signalpeak_i;
  result->noisepeaki = noisepeak_i;
  result->thi1 = threshold_i1;
  result->pulse = rravg1;
  result->is_regular = regular;
}

//void decision_making_extracted() {
//  if ((integral_value >= threshold_i1)) {
//    // There's a 200ms latency. If the new peak respects this condition, we can keep testing.
//    if (sample > lastQRS + DELAY_200ms_IN_SAMPLES) {
//        // If it respects the 200ms latency, but it doesn't respect the 360ms latency, we check the slope.
//      if (sample <= lastQRS + DELAY_360ms_IN_SAMPLES) {
//        // The squared slope is "M" shaped. So we have to check nearby samples to make sure we're really looking
//        // at its peak value, rather than a low one.
//        signalpeak_i = 0.125 * peak_i + 0.875 * signalpeak_i; // 0.125 *
//        threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//        threshold_i2 = 0.5 * threshold_i1;
//        lastSlope = currentSlope;
//        result->is_qrs = true;
//        lastQRS = sample;
//      }
//      // If it was above both thresholds and respects both latency periods, it certainly is an R peak.
//      else {
//        signalpeak_i = 0.125 * peak_i + 0.875 * signalpeak_i; // 0.125 *
//        threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//        threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//        lastSlope = currentSlope;
//        result->is_qrs = true;
//        lastQRS = sample;
//      }
//    }
//    // If the new peak doesn't respect the 200ms latency, it's noise. Update thresholds and move on to the next sample.
//    else {
//      peak_i = integral_value;
//      noisepeak_i = 0.125 * peak_i + 0.875 * noisepeak_i;
//      threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i);
//      threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//      result->is_qrs = false;
//      result->peaki = peak_i;
//      result->signalpeaki = signalpeak_i;
//      result->noisepeaki = noisepeak_i;
//      result->thi1 = threshold_i1;
//      return;
//    }
//  }
//
//  // If a R-peak was detected, the RR-averages must be updated.
//  if (result->is_qrs) {
//    // Add the newest RR-interval to the buffer and get the new average.
//    rravg1 = 0;
//    for (i = 0; i < 7; i++) {
//      rr1[i] = rr1[i + 1];
//      rravg1 += rr1[i];
//    }
//    rr1[7] = sample - lastQRS;
//    rravg1 += rr1[7];
//    rravg1 = rravg1 >> 3; // * 0.125
//    lastQRS = sample;
//
//    // If the newly-discovered RR-average is normal, add it to the "normal" buffer and get the new "normal" average.
//    // Update the "normal" beat parameters.
//    if ((rr1[7] >= rrlow) && (rr1[7] <= rrhigh)) {
//      rravg2 = 0;
//      for (i = 0; i < 7; i++) {
//        rr2[i] = rr2[i + 1];
//        rravg2 += rr2[i];
//      }
//      rr2[7] = rr1[7];
//      rravg2 += rr2[7];
//      rravg2 = rravg2 >> 3; // * 0.125
//      rrlow = 0.92 * rravg2;
//      rrhigh = 1.16 * rravg2;
//      rrmiss = 1.66 * rravg2;
//    }
//
//    prevRegular = regular;
//    if (rravg1 == rravg2) {
//      regular = true;
//    }
//    // If the beat had been normal but turned odd, change the thresholds.
//    else {
//      regular = false;
//      if (prevRegular) {
//        threshold_i1 = 0.5 * threshold_i1; // 0.5 *
//      }
//    }
//  }
//  // If no R-peak was detected, it's important to check how long it's been since the last detection.
//  else {
//    // If no R-peak was detected for too long, use the lighter thresholds and do a back search.
//    // However, the back search must respect the 200ms limit and the 360ms one (check the slope).
//    if ( // ((sample - lastQRS) > (uint32_t)rrmiss) &&
//        (sample > (lastQRS + DELAY_200ms_IN_SAMPLES))) {
//      for (k = current_index - (sample - lastQRS) + DELAY_200ms_IN_SAMPLES; k < current_index; k++) {
//        i = MOD_INDEX(k);
//        if ((integral[i] > threshold_i2)
//            ) {
//            peak_i = integral[i];
//            signalpeak_i = 0.25 * peak_i + 0.75 * signalpeak_i; // 0.25 *
//            threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//            threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//            lastSlope = currentSlope;
//            // If a signal peak was detected on the back search, the RR attributes must be updated.
//            // This is the same thing done when a peak is detected on the first try.
//            //RR Average 1
//            rravg1 = 0;
//            for (j = 0; j < 7; j++) {
//              rr1[j] = rr1[j + 1];
//              rravg1 += rr1[j];
//            }
//            rr1[7] = sample - MOD_INDEX(array_index - i) - lastQRS;
//            lastQRS = sample - MOD_INDEX(array_index - i);
//            rravg1 += rr1[7];
//            rravg1 = rravg1 >> 3; // * 0.125
//            result->is_qrs = true;
//
//            //RR Average 2
//            if ((rr1[7] >= rrlow) && (rr1[7] <= rrhigh)) {
//              rravg2 = 0;
//              for (i = 0; i < 7; i++) {
//                rr2[i] = rr2[i + 1];
//                rravg2 += rr2[i];
//              }
//              rr2[7] = rr1[7];
//              rravg2 += rr2[7];
//              rravg2 = rravg2 >> 3; // * 0.125
//              rrlow = 0.92 * rravg2;
//              rrhigh = 1.16 * rravg2;
//              rrmiss = 1.66 * rravg2;
//            }
//
//            prevRegular = regular;
//            if (rravg1 == rravg2) {
//              regular = true;
//            }
//            else {
//              regular = false;
//              if (prevRegular)
//              {
//                threshold_i1 = 0.5 * threshold_i1; // 0.5 *
//              }
//            }
//            break;
//        }
//      }
//      if (result->is_qrs) {
//        result->is_regular = regular;
//        result->peaki = peak_i;
//        result->signalpeaki = signalpeak_i;
//        result->noisepeaki = noisepeak_i;
//        result->thi1 = threshold_i1;
//        result->pulse = rravg1;
//        return;
//      }
//    }
//
//    // Definitely no signal peak was detected.
//    if (!result->is_qrs) {
//      // If some kind of peak had been detected, then it's certainly a noise peak. Thresholds must be updated accordingly.
//      if ((integral_value >= threshold_i1)) {
//        peak_i = integral[array_index];
//        noisepeak_i = 0.125 * peak_i + 0.875 * noisepeak_i; // 0.125 *
//        threshold_i1 = noisepeak_i + 0.25 * (signalpeak_i - noisepeak_i); // 0.25 *
//        threshold_i2 = 0.5 * threshold_i1; // 0.5 *
//      }
//    }
//  }
//  // The current_index implementation outputs '0' for every sample where no peak was detected,
//  // and '1' for every sample where a peak was detected. It should be changed to fit
//  // the desired application.
//  // The 'if' accounts for the delay introduced by the filters: we only start outputting after the delay.
//  // However, it updates a few samples back from the buffer. The reason is that if we update the detection
//  // for the current_index sample, we might miss a peak that could've been found later by backsearching using
//  // lighter thresholds. The final waveform output does match the original signal, though.
//  result->peaki = peak_i;
//  result->signalpeaki = signalpeak_i;
//  result->noisepeaki = noisepeak_i;
//  result->thi1 = threshold_i1;
//  result->pulse = rravg1;
//  result->is_regular = regular;
//}


/*
 * Select all the peaks that are at least 231ms apart from each other
 * Initialize the thresholds
 * for each Peak:
 *  if Peak > Threshold1:
 *    Classify it as R-peak
 *    Adjust SPK and NPK using Rule-1
 *  if number of detected beats > 8:
 *    Calculate the Mean RR-interval of the eight most recent beats
 *    Calculate current RR-interval
 *    if RR-interval < 360ms or RR-interval < 0.5 * Mean RR-interval:
 *      Classify it as T-wave or QRS complex based on slope
 *      Adjust SPK and NPK using Rule-1, if QRS complex found
 *    else if RR-interval > 1s or RR-interval > 1.66 * Mean RR-interval:
 *      Threshold3 = 0.5 * Threshold2 + 0.5 * MEANSB
 *      if Peak > Threshold3:
 *        Classify it as R-peak
 *        Adjust SPK and NPK using Rule-2
 *    else if RR-interval>1.4s:
 *      if Peak > 0.2 * Threshold2
 *        Classify it as R-peak
 *        Adjust SPK and NPK using Rule-2
 *    Threshold1 = NPK + 0.25 (SPK - NPK)
 *    Threshold2 = 0.4 * Threshold1
*/
//void pptp(uint32_t array_index, uint32_t sample_index, uint32_t* integral, int32_t* filtered, pt_result_t* result) {
//  int32_t current_value = integral[array_index];
//  // Initialize the thresholds
//  threshold_i1 = noisepeak_i + ((signalpeak_i - noisepeak_i) >> 2); // 0.25 *
//  threshold_i2 = threshold_i1;
//
//  // if last peak happened 230ms ago
//  if (sample > (lastQRS + SAMPLING_FREQUENCY / 5)) {
//    // if Peak > Threshold1
//    if (current_value > threshold_i1) {
//      peak_i = current_value;
//      // Classify it as R-peak
//      result->is_qrs = true;
//      // Adjust SPK and NPK using Rule-1
//      signalpeak_i = (peak_i >> 3) + 0.875 * signalpeak_i; // 0.125 *
//      noisepeak_i = (peak_i >> 3) + 0.875 * noisepeak_i; // 0.125 *
//
//      beat_count++;
//    }
//  }
//
//  // if number of detected beats > 8:
//  if (beat_count > 8) {
//    //  Calculate the Mean RR-interval of the eight most recent beats
//    //  Calculate current RR-interval
//    //  if RR-interval < 360ms or RR-interval < 0.5 * Mean RR-interval:
//    //    Classify it as T-wave or QRS complex based on slope
//    //    Adjust SPK and NPK using Rule-1, if QRS complex found
//    //  else if RR-interval > 1s or RR-interval > 1.66 * Mean RR-interval:
//    //    Threshold3 = 0.5 * Threshold2 + 0.5 * MEANSB
//    //    if Peak > Threshold3:
//    //      Classify it as R-peak
//    //      Adjust SPK and NPK using Rule-2
//    //  else if RR-interval>1.4s:
//    //    if Peak > 0.2 * Threshold2
//    //      Classify it as R-peak
//    //      Adjust SPK and NPK using Rule-2
//    //  Threshold1 = NPK + 0.25 (SPK - NPK)
//    //  Threshold2 = 0.4 * Threshold1
//  }
//
////  threshold_i1 = noisepeak_i + ((signalpeak_i - noisepeak_i) >> 2); // 0.25 *
////  threshold_i2 = threshold_i1;
//
//  result->peaki = peak_i;
//  result->signalpeaki = signalpeak_i;
//  result->noisepeaki = noisepeak_i;
//  result->thi1 = threshold_i1;
//  result->pulse = rravg1;
//  result->is_regular = regular;
//}