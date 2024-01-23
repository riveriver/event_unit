/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include "ICM42688P.hpp"

void ICM42688P::ProcessIMU(const hrt_abstime &timestamp_sample, const FIFO::DATA &fifo)
{
	// float accel_x = 0.0, accel_y = 0.0, accel_z = 0.0;
	// float gyro_x = 0.0,  gyro_y = 0.0,  gyro_z = 0.0;
	//
	// // 20 bit hires mode
	//
	// // Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
	// // Accel data is 18 bit
	// int32_t temp_accel_x = reassemble_20bit(fifo.ACCEL_DATA_X1, fifo.ACCEL_DATA_X0,
	// 				   fifo.Ext_Accel_X_Gyro_X & 0xF0 >> 4);
	// int32_t temp_accel_y = reassemble_20bit(fifo.ACCEL_DATA_Y1, fifo.ACCEL_DATA_Y0,
	// 				   fifo.Ext_Accel_Y_Gyro_Y & 0xF0 >> 4);
	// int32_t temp_accel_z = reassemble_20bit(fifo.ACCEL_DATA_Z1, fifo.ACCEL_DATA_Z0,
	// 				   fifo.Ext_Accel_Z_Gyro_Z & 0xF0 >> 4);
	//
	// // Gyro [19:12] + Gyro [11:4] + Gyro [3:0] (bottom 4 bits of 20 bit extension byte)
	// int32_t temp_gyro_x = reassemble_20bit(fifo.GYRO_DATA_X1, fifo.GYRO_DATA_X0,
	//                    fifo.Ext_Accel_X_Gyro_X & 0x0F);
	// int32_t temp_gyro_y = reassemble_20bit(fifo.GYRO_DATA_Y1, fifo.GYRO_DATA_Y0,
	//                    fifo.Ext_Accel_Y_Gyro_Y & 0x0F);
	// int32_t temp_gyro_z = reassemble_20bit(fifo.GYRO_DATA_Z1, fifo.GYRO_DATA_Z0,
	//                    fifo.Ext_Accel_Z_Gyro_Z & 0x0F);

	// // accel samples invalid if -524288
	// if (temp_accel_x != -524288 && temp_accel_y != -524288 && temp_accel_z != -524288) {
	// 	// shift accel by 2 (2 least significant bits are always 0)
	// 	accel_x = (float) temp_accel_x / 4.f;
	// 	accel_y = (float) temp_accel_y / 4.f;
	// 	accel_z = (float) temp_accel_z / 4.f;
	//
	// 	// shift gyro by 1 (least significant bit is always 0)
	// 	gyro_x = (float) temp_gyro_x / 2.f;
	// 	gyro_y = (float) temp_gyro_y / 2.f;
	// 	gyro_z = (float) temp_gyro_z / 2.f;
	//
	// 	// correct frame for publication
	// 	// sensor's frame is +x forward, +y left, +z up
	// 	// flip y & z to publish right handed with z down (x forward, y right, z down)
	// 	accel_y = -accel_y;
	// 	accel_z = -accel_z;
	// 	gyro_y  = -gyro_y;
	// 	gyro_z  = -gyro_z;
	//
	// 	// Scale everything appropriately
	// 	float accel_scale_factor = (CONSTANTS_ONE_G / 8192.f);
	// 	accel_x *= accel_scale_factor;
	// 	accel_y *= accel_scale_factor;
	// 	accel_z *= accel_scale_factor;
	//
	// 	float gyro_scale_factor = math::radians(1.f / 131.f);
	// 	gyro_x *= gyro_scale_factor;
	// 	gyro_y *= gyro_scale_factor;
	// 	gyro_z *= gyro_scale_factor;
	//
	// 	// Store the data in our array
	// 	_imu_server_data.accel_x[_imu_server_index] = accel_x;
	// 	_imu_server_data.accel_y[_imu_server_index] = accel_y;
	// 	_imu_server_data.accel_z[_imu_server_index] = accel_z;
	// 	_imu_server_data.gyro_x[_imu_server_index]  = gyro_x;
	// 	_imu_server_data.gyro_y[_imu_server_index]  = gyro_y;
	// 	_imu_server_data.gyro_z[_imu_server_index]  = gyro_z;
	// 	_imu_server_data.ts[_imu_server_index]      = timestamp_sample;
	// 	_imu_server_index++;
	//
	// 	// If array is full, publish the data
	// 	if (_imu_server_index == 10) {
	// 		_imu_server_index = 0;
	// 		_imu_server_data.timestamp = hrt_absolute_time();
	// 		_imu_server_data.temperature = 0; // Not used right now
	// 		_imu_server_pub.publish(_imu_server_data);
	// 	}
	// }
}

void ICM42688P::ProcessAccel(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_accel_fifo_s accel{};
	accel.timestamp_sample = timestamp_sample;
	accel.samples = 0;
	accel.dt = FIFO_SAMPLE_DT;

	// 18-bits of accelerometer data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {
		// 20 bit hires mode
		// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
		// Accel data is 18 bit ()
		int32_t accel_x = reassemble_20bit(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0,
						   fifo[i].Ext_Accel_X_Gyro_X & 0xF0 >> 4);
		int32_t accel_y = reassemble_20bit(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0,
						   fifo[i].Ext_Accel_Y_Gyro_Y & 0xF0 >> 4);
		int32_t accel_z = reassemble_20bit(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0,
						   fifo[i].Ext_Accel_Z_Gyro_Z & 0xF0 >> 4);

		// sample invalid if -524288
		if (accel_x != -524288 && accel_y != -524288 && accel_z != -524288) {
			// check if any values are going to exceed int16 limits
			static constexpr int16_t max_accel = INT16_MAX;
			static constexpr int16_t min_accel = INT16_MIN;

			if (accel_x >= max_accel || accel_x <= min_accel) {
				scale_20bit = true;
			}

			if (accel_y >= max_accel || accel_y <= min_accel) {
				scale_20bit = true;
			}

			if (accel_z >= max_accel || accel_z <= min_accel) {
				scale_20bit = true;
			}

			// shift by 2 (2 least significant bits are always 0)
			accel.x[accel.samples] = accel_x / 4;
			accel.y[accel.samples] = accel_y / 4;
			accel.z[accel.samples] = accel_z / 4;
			accel.samples++;
		}
	}

	if (!scale_20bit) {
		// if highres enabled accel data is always 8192 LSB/g
		if (!hitl_mode) {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 8192.f);
		}

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			// 20 bit hires mode
			// Sign extension + Accel [19:12] + Accel [11:4] + Accel [3:2] (20 bit extension byte)
			// Accel data is 18 bit ()
			int16_t accel_x = combine(fifo[i].ACCEL_DATA_X1, fifo[i].ACCEL_DATA_X0);
			int16_t accel_y = combine(fifo[i].ACCEL_DATA_Y1, fifo[i].ACCEL_DATA_Y0);
			int16_t accel_z = combine(fifo[i].ACCEL_DATA_Z1, fifo[i].ACCEL_DATA_Z0);

			accel.x[i] = accel_x;
			accel.y[i] = accel_y;
			accel.z[i] = accel_z;
		}

		if (!hitl_mode) {
			_px4_accel.set_scale(CONSTANTS_ONE_G / 2048.f);
		}
	}

	// correct frame for publication
	for (int i = 0; i < accel.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		accel.x[i] = accel.x[i];
		accel.y[i] = (accel.y[i] == INT16_MIN) ? INT16_MAX : -accel.y[i];
		accel.z[i] = (accel.z[i] == INT16_MIN) ? INT16_MAX : -accel.z[i];
	}

	if (!hitl_mode) {
		_px4_accel.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					   perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
	}

	if (accel.samples > 0) {
		if (!hitl_mode) {
			_px4_accel.updateFIFO(accel);
		}
	}
}

void ICM42688P::ProcessGyro(const hrt_abstime &timestamp_sample, const FIFO::DATA fifo[], const uint8_t samples)
{
	sensor_gyro_fifo_s gyro{};
	gyro.timestamp_sample = timestamp_sample;
	gyro.samples = 0;
	gyro.dt = FIFO_SAMPLE_DT;

	// 20-bits of gyroscope data
	bool scale_20bit = false;

	// first pass
	for (int i = 0; i < samples; i++) {
		// 20 bit hires mode
		// Gyro [19:12] + Gyro [11:4] + Gyro [3:0] (bottom 4 bits of 20 bit extension byte)
		int32_t gyro_x = reassemble_20bit(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0, fifo[i].Ext_Accel_X_Gyro_X & 0x0F);
		int32_t gyro_y = reassemble_20bit(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0, fifo[i].Ext_Accel_Y_Gyro_Y & 0x0F);
		int32_t gyro_z = reassemble_20bit(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0, fifo[i].Ext_Accel_Z_Gyro_Z & 0x0F);

		// check if any values are going to exceed int16 limits
		static constexpr int16_t max_gyro = INT16_MAX;
		static constexpr int16_t min_gyro = INT16_MIN;

		if (gyro_x >= max_gyro || gyro_x <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_y >= max_gyro || gyro_y <= min_gyro) {
			scale_20bit = true;
		}

		if (gyro_z >= max_gyro || gyro_z <= min_gyro) {
			scale_20bit = true;
		}

		gyro.x[gyro.samples] = gyro_x / 2;
		gyro.y[gyro.samples] = gyro_y / 2;
		gyro.z[gyro.samples] = gyro_z / 2;
		gyro.samples++;
	}

	if (!scale_20bit) {
		// if highres enabled gyro data is always 131 LSB/dps
		if (!hitl_mode) {
			_px4_gyro.set_scale(math::radians(1.f / 131.f));
		}

	} else {
		// 20 bit data scaled to 16 bit (2^4)
		for (int i = 0; i < samples; i++) {
			gyro.x[i] = combine(fifo[i].GYRO_DATA_X1, fifo[i].GYRO_DATA_X0);
			gyro.y[i] = combine(fifo[i].GYRO_DATA_Y1, fifo[i].GYRO_DATA_Y0);
			gyro.z[i] = combine(fifo[i].GYRO_DATA_Z1, fifo[i].GYRO_DATA_Z0);
		}

		if (!hitl_mode) {
			_px4_gyro.set_scale(math::radians(2000.f / 32768.f));
		}
	}

	// correct frame for publication
	for (int i = 0; i < gyro.samples; i++) {
		// sensor's frame is +x forward, +y left, +z up
		//  flip y & z to publish right handed with z down (x forward, y right, z down)
		gyro.x[i] = gyro.x[i];
		gyro.y[i] = (gyro.y[i] == INT16_MIN) ? INT16_MAX : -gyro.y[i];
		gyro.z[i] = (gyro.z[i] == INT16_MIN) ? INT16_MAX : -gyro.z[i];
	}

	if (!hitl_mode) {
		_px4_gyro.set_error_count(perf_event_count(_bad_register_perf) + perf_event_count(_bad_transfer_perf) +
					  perf_event_count(_fifo_empty_perf) + perf_event_count(_fifo_overflow_perf));
	}

	if (gyro.samples > 0) {
		if (!hitl_mode) {
			_px4_gyro.updateFIFO(gyro);
		}
	}
}

bool ICM42688P::ProcessTemperature(const FIFO::DATA fifo[], const uint8_t samples)
{
	int16_t temperature[FIFO_MAX_SAMPLES];
	float temperature_sum{0};

	int valid_samples = 0;

	for (int i = 0; i < samples; i++) {
		const int16_t t = combine(fifo[i].TEMP_DATA1, fifo[i].TEMP_DATA0);

		// sample invalid if -32768
		if (t != -32768) {
			temperature_sum += t;
			temperature[valid_samples] = t;
			valid_samples++;
		}
	}

	if (valid_samples > 0) {
		const float temperature_avg = temperature_sum / valid_samples;

		for (int i = 0; i < valid_samples; i++) {
			// temperature changing wildly is an indication of a transfer error
			if (fabsf(temperature[i] - temperature_avg) > 1000) {
				perf_count(_bad_transfer_perf);
				return false;
			}
		}

		// use average temperature reading
		const float TEMP_degC = (temperature_avg / TEMPERATURE_SENSITIVITY) + TEMPERATURE_OFFSET;

		if (PX4_ISFINITE(TEMP_degC)) {
			if (!hitl_mode) {
				_px4_accel.set_temperature(TEMP_degC);
				_px4_gyro.set_temperature(TEMP_degC);
				return true;
			}

		} else {
			perf_count(_bad_transfer_perf);
		}
	}

	return false;
}
