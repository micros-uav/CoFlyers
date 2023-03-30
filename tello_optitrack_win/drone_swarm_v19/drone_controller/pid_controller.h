#pragma once

namespace pid
{
	class pid_controller
	{
	public:
		pid_controller();
		~pid_controller();
		pid_controller(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);

		float proportion = 0.0f;
		float integral = 0.0f;
		float derivative = 0.0f;
		float c_filter_d = 1.0f;
		float dt = 0.01f;
		float output_min = 0.0f;
		float output_max = 1.0f;
		float integral_min = -1.0f;
		float integral_max = 1.0f;

		void reset();
		void reset_integral();

		float update(const float error);

		void set_params(float proportion, float integral, float derivative, float c_filter_d, float dt, float output_min, float output_max, float integral_min, float integral_max);
	private:
		float error_former = 0.0f;
		float error_i_former = 0.0f;
		float error_d_former = 0.0f;
		float first = 0.0f;
	};

	void constrain(float& input,const float& value_min, const float& value_max);
}