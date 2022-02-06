#ifndef __PID_H
#define __PID_H

//
// 	PID Controller Diagram:
//
//						-----------------		-------------
//	Set Point  + 		|         		|  PWM  |			|          Output (Flow)
//			---> O ---->|  Controller	|------>|  System	|---------->
//			     ^		|         		|		|			|		|
//				 |      -----------------		-------------		|
//				 |													|
//			     |                     --------------				|
//	             |      			   |			|          		|
//		         ----------------------|  Sensor	|<---------------
//		  Feedback                     |			|
//			                           --------------
//
//	System characteristics:
//		Minimum flow: =~ 1 V 	(PWM duty cycle 26%)
//		Maximum flow: =~ 2.15 V	(PWM duty cycle 100%)
//
//
//
//

#define PID_INTEGRAL_SATURATION	100.0f
#define PID_MAX_OUT				1000
#define PID_MIN_OUT				 260

typedef struct sPID
{
	float dt;			// Sampling time in seconds
	float tau;			// Low Pass Filter time constant

	float kP;			// Proportional gain
	float kI;			// Integral gain
	float kD;			// Derivative gain

	float proportional;	// Proportional term
	float integral;		// Integral term
	float derivative;	// Derivative term

	float set_point;	// Set Point - desired controlled variable value
	float feedback;		// Feedback - measurement from sensor
	float error;		// Error == (Set Point - Feedback)

	float output;		// Output - actuator drive signal

	float prev_feedback;// Feedback of previous iteration
	float prev_error;	// Error of previous iteration

} sPID;

void 	PID_init(sPID* PID);
void 	PID_update(sPID* PID);

#endif // __PID_H
