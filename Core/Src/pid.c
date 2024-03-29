//
//	André A. M. Araújo
//	2022/01/28
//
//	This library handles PID Control calculations. It was based on
// 	the algorithm available at Wikipedia and also took inspiration
// 	on Phil's Lab video available on:
//	"https://www.youtube.com/watch?v=zOByx3Izf5U".
//

#include "pid.h"

void 	PID_init(sPID* PID)
{
	PID->dt = 0.010;			// Sampling time: 10 ms (100 Hz)

	PID->kP =  0.50;			// Proportional gain
	PID->kI = 12.00;			// Integral gain
	PID->kD =  0.05;			// Derivative gain

	PID->proportional = 0.0;	// Proportional term
	PID->integral 	  = 0.0;	// Integral term
	PID->derivative   = 0.0;	// Derivative term

	PID->set_point 	= 250.0;	// Set Point - desired controlled variable value
	PID->feedback 	= 0.0;		// Feedback  - measurement from sensor
	PID->error 		= 0.0;		// Error == (Set Point - Feedback)

	PID->output = 0.0;			// Output    - actuator drive signal

	PID->prev_feedback = 0.0;	// Feedback of previous iteration
	PID->prev_error = 0.0;		// Error of previous iteration
}

void 	PID_update(sPID* PID)
{
	PID->error = PID->set_point - PID->feedback;

	PID->proportional = PID->kP * PID->error;

	PID->integral = PID->integral + PID->kI * PID->error * PID->dt;

	// Integral term saturation to avoid cumulative error effects:
	if 		(PID->integral >  PID_INTEGRAL_SATURATION)	PID->integral =  PID_INTEGRAL_SATURATION;
	else if	(PID->integral < -PID_INTEGRAL_SATURATION)	PID->integral = -PID_INTEGRAL_SATURATION;

	PID->derivative = PID->kD * (PID->error - PID->prev_error) / PID->dt;

	PID->output = PID->proportional + PID->integral + PID->derivative;

	// Output saturation to keep value within actuator limits:
	if 		(PID->output > PID_MAX_OUT)	PID->output = PID_MAX_OUT;
	else if	(PID->output < PID_MIN_OUT)	PID->output = PID_MIN_OUT;

	PID->output = PID->output/PID_MAX_OUT; 	// Normalized output (from 0.000 to 1.000)

	// Variable storage for next iteration
	PID->prev_error	 	= PID->error;
	PID->prev_feedback 	= PID->feedback;
}
