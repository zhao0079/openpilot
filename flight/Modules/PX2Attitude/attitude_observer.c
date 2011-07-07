/*=====================================================================

 PIXHAWK Micro Air Vehicle Flying Robotics Toolkit

 (c) 2009, 2010 PIXHAWK PROJECT  <http://pixhawk.ethz.ch>

 This file is part of the PIXHAWK project

 PIXHAWK is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 PIXHAWK is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with PIXHAWK. If not, see <http://www.gnu.org/licenses/>.

 ======================================================================*/

/**
 * @file
 *   @brief Attitude Observer Filter
 *
 *   @author Martin Rutschmann <pixhawk@student.ethz.ch>
 *
 * This Observer Filter, filters the measurement of a vector which is constant in the
 * World Frame. For prediction it uses the gyroscope measurements and for the correction
 * the measurement of the vector (accelerometer or magnetometer). It has a constant Kalman
 * Gain Matrix which can be calculated using MATLAB
 */

#include "attitude_observer.h"
#include <math.h>

#define TS 0.005f
#define K_ACCEL      0.00313666302f	 //~0.2 Hertz
#define K_MAGNET     0.00313666302f	 //~0.2 Hertz
#define K_GYRO       0.145364001f    //~10 Hertz
#define K_GYRO_OFF   0.000313666302f //~0.02 Hertz
#define K_ACCEL_BODY 0.145364001f	 //~10 Hertz

static float_vect3 state_accel;
static float_vect3 state_magnet;
static float_vect3 state_gyro;
static float_vect3 state_gyro_offset;
static float_vect3 state_body_accel;

void attitude_observer_init(float_vect3 init_state_accel,
		float_vect3 init_state_magnet)
{
	state_accel = init_state_accel;
	state_magnet = init_state_magnet;
	state_gyro.x = 0;
	state_gyro.y = 0;
	state_gyro.z = 0;
	state_gyro_offset.x = 0;
	state_gyro_offset.y = 0;
	state_gyro_offset.z = 0;
	state_body_accel.x = 0;
	state_body_accel.y = 0;
	state_body_accel.z = 0;
}

void attitude_observer_predict(float fTime)
{

	float w1 = fTime * (state_gyro.x-state_gyro_offset.x);
	float w2 = fTime * (state_gyro.y-state_gyro_offset.y);
	float w3 = fTime * (state_gyro.z-state_gyro_offset.z);

	//Predict accel Vector
	float x = state_accel.x + w3 * state_accel.y - w2 * state_accel.z;
	float y = -w3 * state_accel.x + state_accel.y + w1 * state_accel.z;
	float z = w2 * state_accel.x - w1 * state_accel.y + state_accel.z;
	state_accel.x = x;
	state_accel.y = y;
	state_accel.z = z;

	//Predict magnet Vector
	x = state_magnet.x + w3 * state_magnet.y - w2 * state_magnet.z;
	y = -w3 * state_magnet.x + state_magnet.y + w1 * state_magnet.z;
	z = w2 * state_magnet.x - w1 * state_magnet.y + state_magnet.z;

	state_magnet.x = x;
	state_magnet.y = y;
	state_magnet.z = z;
}

void attitude_observer_correct_gyro(float_vect3 gyros)
{
	state_gyro.x = (1.0f - K_GYRO) * state_gyro.x + K_GYRO * gyros.x;
	state_gyro.y = (1.0f - K_GYRO) * state_gyro.y + K_GYRO * gyros.y;
	state_gyro.z = (1.0f - K_GYRO) * state_gyro.z + K_GYRO * gyros.z;
}

void attitude_observer_correct_accel(float_vect3 accel, float fDeltaTime)
{
	float_vect3 state_accel_old;
	float_vect3 gyro_offset_calc;

	if(fDeltaTime > 0)
	{
		state_accel_old.x = state_accel.x;
		state_accel_old.y = state_accel.y;
		state_accel_old.z = state_accel.z;
	}

	state_accel.x = state_accel.x * (1.0f - K_ACCEL) + K_ACCEL * accel.x;
	state_accel.y = state_accel.y * (1.0f - K_ACCEL) + K_ACCEL * accel.y;
	state_accel.z = state_accel.z * (1.0f - K_ACCEL) + K_ACCEL * accel.z;

	if(fDeltaTime > 0)
	{
		gyro_offset_calc.x = ( state_accel_old.y * state_accel.z - state_accel_old.z * state_accel.y )/fDeltaTime;
		gyro_offset_calc.y = ( state_accel_old.z * state_accel.x - state_accel_old.x * state_accel.z )/fDeltaTime;
		gyro_offset_calc.z = ( state_accel_old.x * state_accel.y - state_accel_old.y * state_accel.x )/fDeltaTime;

		state_gyro_offset.x += K_GYRO_OFF * gyro_offset_calc.x;
		state_gyro_offset.y += K_GYRO_OFF * gyro_offset_calc.y;
		state_gyro_offset.z += K_GYRO_OFF * gyro_offset_calc.z;
	}
}

void attitude_observer_correct_magnet(float_vect3 magnet, float fDeltaTime)
{
	float_vect3 state_magnet_old;
	float_vect3 gyro_offset_calc;

	if(fDeltaTime > 0)
	{
		state_magnet_old.x = state_magnet.x;
		state_magnet_old.y = state_magnet.y;
		state_magnet_old.z = state_magnet.z;
	}

	state_magnet.x = state_magnet.x * (1.0f-K_MAGNET) + K_MAGNET * magnet.x;
	state_magnet.y = state_magnet.y * (1.0f-K_MAGNET) + K_MAGNET * magnet.y;
	state_magnet.z = state_magnet.z * (1.0f-K_MAGNET) + K_MAGNET * magnet.z;

	if(fDeltaTime > 0)
	{
		gyro_offset_calc.x = ( state_magnet_old.y * state_magnet.z - state_magnet_old.z * state_magnet.y )/fDeltaTime;
		gyro_offset_calc.y = ( state_magnet_old.z * state_magnet.x - state_magnet_old.x * state_magnet.z )/fDeltaTime;
		gyro_offset_calc.z = ( state_magnet_old.x * state_magnet.y - state_magnet_old.y * state_magnet.x )/fDeltaTime;

		state_gyro_offset.x += K_GYRO_OFF * gyro_offset_calc.x;
		state_gyro_offset.y += K_GYRO_OFF * gyro_offset_calc.y;
		state_gyro_offset.z += K_GYRO_OFF * gyro_offset_calc.z;
	}
}

void attitude_observer_get_angles(float_vect3* angles, float_vect3* angular_rates){
	// rotated the states to the correct coordinate system
	float_vect3 accel_rot = { state_accel.y, state_accel.x, -state_accel.z };

	angles->x = atan2(-accel_rot.y, -accel_rot.z);
	angles->y = asin(accel_rot.x / 9.81f);

//	float x = cos(angles->y) * state_magnet.x + sin(angles->x) * sin(angles->y)
//			* state_magnet.y + cos(angles->x) * sin(angles->y) * state_magnet.z;
//	float y = cos(angles->x) * state_magnet.y - sin(angles->x) * state_magnet.z;
//	angles->z = atan2(x, y);

	// write out the offset corrected body angular rates. Also change the coordinate system.
	angular_rates->x =  (state_gyro.y - state_gyro_offset.y);
	angular_rates->y =  (state_gyro.x - state_gyro_offset.x);
	angular_rates->z = -(state_gyro.z - state_gyro_offset.z);
}
