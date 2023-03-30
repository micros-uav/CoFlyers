#include"pid_controller.h"

namespace pid
{
	pid_controller::pid_controller()
	{
	}

	pid_controller::~pid_controller()
	{
	}

	pid_controller::pid_controller(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->proportion = proportion;
		this->integral = integral;
		this->derivative = derivative;
		this->c_filter_d = c_filter_d;
		this->dt = dt;
		this->output_min = output_min;
		this->output_max = output_max;
		this->integral_min = integral_min;
		this->integral_max = integral_max;
	}

	void pid_controller::reset()
	{
		this->error_former = 0.0f;
		this->error_i_former = 0.0f;
		this->error_d_former = 0.0f;
		this->first = 0.0f;
	}

	void pid_controller::reset_integral()
	{
		this->error_i_former = 0.0f;
	}

	float pid_controller::update(const float error)
	{
		// Derivate
		float error_d = (error - this->error_former) / this->dt;
		error_d = (error_d * this->c_filter_d +  this->error_d_former * (1.0f - this->c_filter_d)) * this->first;
		// Integral
		float error_i = this->error_i_former + (this->error_former + error) / 2.0f * this->dt;
		constrain(error_i, this->integral_min, this->integral_max);
		// Output
		float output = this->proportion * error + this->derivative * error_d + this->integral * error_i;
		constrain(output, this->output_min, this->output_max);
		// Save
		this->error_former = error;
		this->error_d_former = error_d;
		this->error_i_former = error_i;
		this->first = 1.0f;

		return output;
	}

	void pid_controller::set_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max)
	{
		this->proportion = proportion;
		this->integral = integral;
		this->derivative = derivative;
		this->c_filter_d = c_filter_d;
		this->dt = dt;
		this->output_min = output_min;
		this->output_max = output_max;
		this->integral_min = integral_min;
		this->integral_max = integral_max;
	}

	void constrain(float&input, const float& value_min, const float& value_max)
	{
		if (input < value_min)
		{
			input = value_min;
		}
		if (input > value_max)
		{
			input = value_max;
		}
		return;
	}

}