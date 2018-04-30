// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/// @file	LowPassFilter.h
/// @brief	A class to implement a second order low pass filter
/// Author: Leonard Hall <LeonardTHall@gmail.com>
/// Adapted for PX4 by Andrew Tridgell
#include "math.h"

#pragma once

namespace math
{
class LowPassFilter2p
{
public:
	// constructor
	LowPassFilter2p(double sample_freq = 0.0f, double cutoff_freq = 0.0f) :
		_cutoff_freq(cutoff_freq),
		_a1(0.0f),
		_a2(0.0f),
		_b0(0.0f),
		_b1(0.0f),
		_b2(0.0f),
		_delay_element_1(0.0f),
		_delay_element_2(0.0f)
	{
		// set initial parameters
		set_cutoff_frequency(sample_freq, cutoff_freq);
	}

	/**
	 * Change filter parameters
	 */
	void set_cutoff_frequency(double sample_freq, double cutoff_freq)
	{
		_cutoff_freq = cutoff_freq;

		if (_cutoff_freq <= 0.0f) {
			// no filtering
			return;
		}

		double fr = sample_freq / _cutoff_freq;
		double ohm = tan(M_PI / fr);
		double c = 1.0f + 2.0f * cos(M_PI / 4.0f) * ohm + ohm * ohm;
		_b0 = ohm * ohm / c;
		_b1 = 2.0f * _b0;
		_b2 = _b0;
		_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
		_a2 = (1.0f - 2.0f * cos(M_PI / 4.0f) * ohm + ohm * ohm) / c;
	}

	/**
	 * Add a new raw value to the filter
	 *
	 * @return retrieve the filtered result
	 */
	double apply(double sample)
	{
		if (_cutoff_freq <= 0.0f) {
			// no filtering
			return sample;
		}

		// do the filtering
		double delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;

		delay_element_0 = sample;

		double output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

		_delay_element_2 = _delay_element_1;
		_delay_element_1 = delay_element_0;

		// return the value.  Should be no need to check limits
		return output;
	}

	/**
	 * Return the cutoff frequency
	 */
	double get_cutoff_freq() const
	{
		return _cutoff_freq;
	}

	/**
	 * Reset the filter state to this value
	 */
	double reset(double sample)
	{
		double dval = sample / (_b0 + _b1 + _b2);
		_delay_element_1 = dval;
		_delay_element_2 = dval;
		return apply(sample);
	}


private:
	double           _cutoff_freq;
	double           _a1;
	double           _a2;
	double           _b0;
	double           _b1;
	double           _b2;
	double           _delay_element_1;        // buffered sample -1
	double           _delay_element_2;        // buffered sample -2
};

} // namespace math
