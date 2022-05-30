#ifndef PID_H
#define PID_H

/*
 * my savior:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
 * 
 * All the credits to my man Brett
 */

class PID {

private:

	int update_interval_ms = 1000; // 1 sec by default
	float error = 0.0, last_input = 0.0;
	float p = 0.0, i = 0.0, d = 0.0;
	float output, min_output, max_output;

	/*
	 * gain: Kp = 100/P
	 * 
	 * Determines how fast the system respond.
	 * Increasing P may cause more sensitive, less stable loops. Same thing happens decreasing Kp.
	 */
	float Kp = 0.5;
	
	/*
	 * reset: Ki = 1/T
	 * 
	 * It's the sum of all the signal values recorded, captured from when we started the controller.
	 * Determines how fast a steady state error is removed.
	 * Smaller values for T makes it take longer to converge to a steady state. Same thing happens 
	 * increasing Ki.
	 */
	float Ki = 0.01;
	
	/*
	 * The purpose for the derivative term is to predict change.
	 * The value of this parameter basically means how far in the future you want to 
	 * predict the rate of change.
	 * Better to keep it to 0 since we don't know if the singal is clean (I bet it isn't)
	 */
	float Kd = 0.0;

	// pid mode (manual/auto)
	bool mode = AUTO;

public:

	static const bool AUTO = false;
	static const bool MANUAL = true;

	float compute (float setpoint, float input);    

	// void autotune (void);

	void set_parameters (float Kp, float Ki, float Kd);

	void set_update_interval (int update_interval_ms);

	void set_output_limit (float min, float max);

	inline void set_auto (void) {mode = AUTO;}

	inline void set_manual (void) {mode = MANUAL;}

	void set_mode (bool mode);

	void initialize (void);

};

#endif