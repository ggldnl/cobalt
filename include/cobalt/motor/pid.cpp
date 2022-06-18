#include "pid.h"
#include "utils.h"

/**
 * @brief clamps the value v between the two provided thresholds
 * 
 */
template <typename T>
static T clamp(T v, T bottom, T top) {
	if (v > top)
		return top;
	if (v < bottom)
		return bottom;
	return v;
}

float PID::compute (float setpoint, float input) {

	if (mode == MANUAL)
		return setpoint; // internal variables remain constant

	error = setpoint - input;

	// pid values
	p = error;

	/*
	 * The integral term is the only term that changes drastically
	 * when the parameters change. This is due to the definition of
	 * the integral itself:
	 * 
	 * Ki * integral(E, dt) ~ Ki * (E1, E2, ...)
	 * 
	 * If we change Ki midway, the whole integral term is affected.
	 * If we bring Ki inside the sum, each term is scaled by its
	 * current Ki:
	 * 
	 * Ki * integral(E, dt) -> integral(Ki * E, dt)
	 * ~ (E1 * Ki1 + E2 * Ki2 + E3 * Ki3 + ...)
	 * 
	 * Ki only affects us moving forward. Before, when ki was changed, 
	 * it rescaled the entire sum of the error; every error value we 
	 * had seen. This way the previous error remains untouched and the 
	 * new Ki only affects things moving forward
	 */
	i += (Ki * error);
	i = clamp<float> (i, min_output, max_output); // clamp integral term (see set_output_limits)

	d = input - last_input;

	/* 
	 * It turns out that the derivative of the error is equal 
	 * to negative derivative of the input, except when the setpoint is changing. 
	 * Instead of adding (Kd * derivative of error), we subtract 
	 * (Kd * derivative of input). This is known as using 
	 * “Derivative on Measurement”
	 * 
	 * d(E)/dt = d(setpoint)/dt - d(input)/dt
	 * 
	 * when setpoint is constant:
	 * 
	 * d(E)/dt = - d(input)/dt
	 * 
	 * instead of keeping track of last_error, we store last_input
	 */
	output = Kp * p + i - Kd * d;

	// ROS_INFO("error=%f	| p=%f	| i=%f	| d=%f	| output=%f", error, p, i, d, output);

	// clamp output term
	output = clamp<float> (output, min_output, max_output);

	last_input = input;

	return output;
}

void PID::set_parameters (float Kp, float Ki, float Kd) {

	float update_interval_s = (float) (update_interval_ms / 1000);

	Kp = Kp;
	Ki = Ki * update_interval_s;
	Kd = Kd / update_interval_s;
}

void PID::set_update_interval (int new_update_interval_ms) {

	if (new_update_interval_ms > 0) {

		float ratio = (float)new_update_interval_ms / (float)update_interval_ms;

		Ki *= ratio;
		Kd /= ratio;

		update_interval_ms = new_update_interval_ms;
	}
}

/*
 * For example, the PWM output on an Arduino accepts values in range 0 to 255. 
 * By default the PID controller doesn’t know this. If it thinks that 
 * 300-400-500 will work, it’s going to try those values expecting to get 
 * what it needs. Since in reality the value is clamped at 255 it’s just going 
 * to keep trying higher and higher numbers without getting anywhere. 
 * The problem reveals itself in the form of weird lags. 
 * Furthermore, when the setpoint is dropped the output has to wind down before 
 * getting below that 255 limit. We can solve this by setting a limit, so the
 * controller knows when it needs to stop.
 * 
 * This means there’s no need for external clamping of the output
 */
void PID::set_output_limit (float min, float max) {

	if (min > max)
		return;

	min_output = min;
	max_output = max;

	i = clamp<float> (i, min_output, max_output);
}

void PID::initialize (void) {
	// last_input = input // done by compute() at each call
	i = output;
	i = clamp<float> (i, min_output, max_output);
}

/*
 * If the controller is off and we then turn if on, the output value jumps 
 * back to the last output value it sent, then starts adjusting from there. 
 * This results in an input bump that we’d rather not have.
 * 
 * I'm not so sure about this thing but I trust him with all my heart
 */
void PID::set_mode (bool mode) {

	// if we are in manual mode and try to transition to auto mode
	if (this -> mode == MANUAL && mode == AUTO)
		initialize();

	this -> mode = mode;
}