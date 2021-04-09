/*
 * fir_filter.h
 *
 *  Created on: Mar 25, 2021
 *      Author: Carl Wallmark
 */

#ifndef INC_FIR_H_
#define INC_FIR_H_

#define BLOCK_SIZE 32
#define NUM_TAPS 5
#define LENGTH_SAMPLES 256
static float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t Output[LENGTH_SAMPLES];

// Our FIR filter, 5 taps, This can be redesigned if using other frequencies, offloaded to DSP (hardware)
// Use this for example http://t-filter.engineerjs.com/ for FIR filter design

static float32_t firCoeffs32[NUM_TAPS] =
{
		  0.10936985167094607,
		  0.28882050651151964,
		  0.4010446654579384,
		  0.28882050651151964,
		  0.10936985167094607
};

#endif /* INC_FIR_H_ */
