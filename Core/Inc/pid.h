#ifndef __PID_H__
#define __PID_H__

//
// 	PID Controller Diagram:
//
//						   ------------------		-------------
//	Set Point  	 + 		   |         		|  PWM  |			|          Output (Flow)
//	(Flow)	    --> O ---->|  Controller	|------>|  System	|---------->
//			    	^ -	   |         		|		|			|		|
//					|      ------------------		-------------		|
//					|													|
//			   		|                     	--------------				|
//	             	|      			   		|			 |          	|
//		         	------------------------|  Sensor	 |<--------------
//		 		Feedback (Flow)          	|			 |
//			                           		--------------
//
//	System characteristics:
//		Minimum flow: =~  85 mL/min (Sensor voltage: 1.00 V; Pump PWM duty cycle:  26.0%)
//		Maximum flow: =~ 500 mL/min (Sensor voltage: 2.15 V; Pump PWM duty cycle: 100.0%)
//

#define PID_INTEGRAL_SATURATION	1000.0f
#define PID_MAX_OUT				1000.0f
#define PID_MIN_OUT				 260.0f

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

#endif // __PID_H__
