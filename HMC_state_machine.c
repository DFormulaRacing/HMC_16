#include <stdint.h>
#include <string.h>
#include "HMC_state_machine.h"
#include "stm32f4xx_can.h"


/*
enum HMC_states{
	off,
	start,
	init,
	brake,
	clutch,
	prop_torqe,
};

enum HMC_states state = off;

float kP;

int main (void)
{
	state = prop_torqe; // need to figure out how we will change states
	int kP = 0;
	float engine_torque;
	float motor_torque;
	
	switch(state){

	case off:
		kP = 0;
	case start:
		kP = 0;
	case init:
		kP = 0
	;case brake:
		kP = 0;
	case clutch:
		kP = 0;
	case prop_torqe:
		kP = desired_kP();
		engine_torque = get_engine_torque(); 
		motor_torque = kP*engine_torque;
		torque_command(motor_torque);
	}
}
*/

float desired_kP(void){

	
	int gear;

	/* Do we have to put in some timing thing????*/

	float kP_gear1 = .1;
	float kP_gear2 = .1;
	float kP_gear3 = .1;
	float kP_gear4 = .1;
	float kP_gear5 = .1;
	float kP_invalid = 0;
	float kP;

	gear = get_gear();

	switch(gear){

	case 1:
		kP = kP_gear1;
		break;
	case 2:
		kP = kP_gear2;
		break;
	case 3:
		kP = kP_gear3;
		break;
	case 4:
		kP = kP_gear4;
		break;
	case 5:
		kP = kP_gear5;
		break;
	case 0:
		kP = kP_invalid;
	}

	return kP;

}


int get_gear(void){
	
	extern input_vector_t input_vector;

	float motor_shaft_speed = input_vector.motor_rpm; /* cast from msg table*/
	float engine_rpm_speed = input_vector.engine_rpm; /* cast from msg table*/

	int gear;
	float current_gear_ratio;
	float gear_1_ratio = (13.0f/32.0f)*(24.0f/73.0f); // = .13356
	float gear_2_ratio = (16.0f/32.0f)*(24.0f/73.0f); // = .16438
	float gear_3_ratio = (17.0f/28.0f)*(24.0f*73.0f); // = .19961
	float gear_4_ratio = (19.0f/26.0f)*(24.0f/73.0f); // = .24025
	float gear_5_ratio = (21.0f/25.0f)*(24.0f/73.0f); // = .27616

	current_gear_ratio = motor_shaft_speed/engine_rpm_speed;

	if (current_gear_ratio <= gear_1_ratio + .01 && current_gear_ratio >= gear_1_ratio -.01)			// create .02 band around each gear ratio
		{
			gear = 1;
		}
		else if (current_gear_ratio <= gear_2_ratio + .01 && current_gear_ratio >= gear_2_ratio -.01)
		{
			gear = 2;
		}
		else if (current_gear_ratio <= gear_3_ratio + .01 && current_gear_ratio >= gear_3_ratio -.01)
		{
			gear = 3;
		}
		else if (current_gear_ratio <= gear_4_ratio + .01 && current_gear_ratio >= gear_4_ratio -.01)
		{
			gear = 4;
		}
		else if (current_gear_ratio <= gear_5_ratio + .01 && current_gear_ratio >= gear_5_ratio -.01)
		{
			gear = 5;
		}
		else
		{
			gear = 0; // default to 5th  gear
		}

	return gear;

}


float get_engine_torque(void){

	float m_dot; /* mass air flow */
	float temp, torque;
	float p_measured;
	float p_atm; // check units
	float temp_measured;
	float temp_atm; // check units, rankine or kelvin
	float torque_wot; // torque at Wide Open Throttle

	torque = torque_wot*p_measured/p_atm*temp_atm/temp_measured;

	torque = 1;
	/* torque = some function */

	return torque;

}
/* need calculator for torque */

void torque_command(float motor_torque)
{

	float motor_torque_output;
	
	
	/*craft CAN message to output to torqeu */
	/* don't forget that the Unitek uses counts instead of torque */

}
