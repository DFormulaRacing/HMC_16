#include <stdint.h>
#include <string.h>
#include "HMC_state_machine.h"
#include "stm32f4xx_can.h"
#include 	"stm32f4xx_can.h"
#include "ring_buffer.h" // change
#include "DFR_SPI.h" // need for ready_to_drive_flag

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
		break;
	}

	return kP;

}

uint16_t max_current = 200; // Amps

float electric_torque(void){
	
	float tps = input_vector.accel_rdval;
	float motor_torque;
	
	motor_torque = (tps/100.0f)*max_current;
	
	return motor_torque;
}

int get_gear(void){
	
	extern input_vector_t input_vector;

	float motor_shaft_speed = input_vector.motor_rpm; 
	float engine_rpm_speed = input_vector.engine_rpm;

	int gear;
	float current_gear_ratio;
	float gear_1_ratio = (13.0f/32.0f)*(24.0f/73.0f); // = .13356
	float gear_2_ratio = (16.0f/32.0f)*(24.0f/73.0f); // = .16438
	float gear_3_ratio = (17.0f/28.0f)*(24.0f*73.0f); // = .19961
	float gear_4_ratio = (19.0f/26.0f)*(24.0f/73.0f); // = .24025
	float gear_5_ratio = (21.0f/25.0f)*(24.0f/73.0f); // = .27616

	current_gear_ratio = motor_shaft_speed/engine_rpm_speed;

	if (current_gear_ratio <= gear_1_ratio + .01f && current_gear_ratio >= gear_1_ratio -.01f)			// create .02 band around each gear ratio
		{
			gear = 1;
		}
		else if (current_gear_ratio <= gear_2_ratio + .01f && current_gear_ratio >= gear_2_ratio -.01f)
		{
			gear = 2;
		}
		else if (current_gear_ratio <= gear_3_ratio + .01f && current_gear_ratio >= gear_3_ratio -.01f)
		{
			gear = 3;
		}
		else if (current_gear_ratio <= gear_4_ratio + .01f && current_gear_ratio >= gear_4_ratio -.01f)
		{
			gear = 4;
		}
		else if (current_gear_ratio <= gear_5_ratio + .01f && current_gear_ratio >= gear_5_ratio -.01f)
		{
			gear = 5;
		}
		else
		{
			gear = 0;
		}

	return gear;

}


float get_engine_torque(void){

	// float m_dot; /* mass air flow */
	// float temp
	float torque;
	float p_measured;
	float p_atm; // check units
	float temp_measured;
	float temp_atm; // check units, rankine or kelvin
	float torque_wot; // torque at Wide Open Throttle

	torque = torque_wot*p_measured/p_atm*temp_atm/temp_measured;

	torque = 1;
	/* torque = some function */
	// MAKE SURE THAT TORQUE IS IN NM!!!!! 

	return torque;

}
/* need calculator for torque */

	uint16_t current_output; // change type to some int
	float kT = .75f; // = Nm / (A rms) 
	uint16_t count = 2^15;
//	uint16_t max_current = 200; // Amps

extern volatile CanTxMsg bamocar_msg;

void torque_command(float motor_torque)
{
	current_output = (uint16_t)(motor_torque/kT)*(count/max_current);
	
	memcpy((void*)&bamocar_msg.Data[1], (void*)&current_output, sizeof(uint16_t));
	
	add_to_output_ring(bamocar_msg);
	
	/*craft CAN message to output to torqeu */
	/* don't forget that the Unitek uses counts instead of torque */

}

car_mode_t mode = car_off; // make external?
car_mode_t old_mode;


typedef enum lock_unlock_e {
	LU_UNLOCK = 0,
	LU_LOCK = 1,
}lock_unlock_t;

lock_unlock_t	lock_state = LU_UNLOCK;

uint8_t push_button_2_z1 = 0;

void lock_unlock_state(void){
	
	switch(lock_state){
		case (LU_UNLOCK):
		{
			mode = (car_mode_t)((input_vector.motor_mode << 1 ) |  input_vector.ice_mode + (~ready_to_drive_flag));
			if(input_vector.push_button_2 & ~push_button_2_z1) // detect rising edge
			{
				lock_state = LU_LOCK;
			}
			else
			{
				// .. stay in LU_UNLOCK state
			}
		}
		break;
		
		case(LU_LOCK):
		{
			if(~input_vector.motor_mode & ~input_vector.ice_mode)
			{
				lock_state = LU_UNLOCK;
			}
			else
			{
				// .. stay in LU_UNLOCK state
			}
		}
		break;
	}
	
	push_button_2_z1 = input_vector.push_button_2;
	old_mode = mode;
	
}

extern CAN_msg_t msgTable[];
void assign_inputs(void){
	
	// Input Assignments
		
		// PE3 inputs
  input_vector.engine_rpm = msgTable[0].data._16[0];
	input_vector.engine_MAP = msgTable[1].data._16[1];
	input_vector.engine_temp = msgTable[4].data._16[1];

	input_vector.accel_rdval = msgTable[2].data._8[0];
	input_vector.brake_rdval = msgTable[2].data._16[1];
	input_vector.clutch_rdval = msgTable[2].data._16[2]; 
		// might need to create two input values since the reading
		// is on two different analog signals
		

		// BMS 1
	input_vector.pack_current 		= (short)(msgTable[6].data._16[0]);
	input_vector.high_temp 				= (int8_t)(msgTable[6].data._8[2]);
	input_vector.low_temp 				= (int8_t)(msgTable[6].data._8[3]);;
	input_vector.avg_temp 				= (int8_t)(msgTable[6].data._8[4]);;
	input_vector.internal_temp 		= (int8_t)(msgTable[6].data._8[5]);;
	
		// BMS 2
	input_vector.pack_inst_voltage 	= (msgTable[7].data._16[0]);
	input_vector.pack_open_voltage 	= (msgTable[7].data._16[1]);
	input_vector.pack_DOD 					= (msgTable[7].data._8[3]);
	input_vector.pack_health 				= (msgTable[7].data._8[4]);
	input_vector.max_pack_voltage 	=  (msgTable[7].data._16[3]);
	
		// BMS 3
	input_vector.min_pack_voltage 		= (msgTable[8].data._16[0]);
	input_vector.low_cell_voltage 		= (msgTable[8].data._16[1]);
	input_vector.high_cell_voltage 		= (msgTable[8].data._16[2]);
	input_vector.low_cell_voltage_id  = (msgTable[8].data._8[6]);
	input_vector.high_cell_voltage_id = (msgTable[8].data._8[7]);
	
		// BMS 4
	input_vector.pack_SOC 						= (msgTable[9].data._16[0]);
	input_vector.pack_amphours 				= (msgTable[9].data._16[1]);
	input_vector.high_opencell_id 		= (msgTable[9].data._8[4]);
	input_vector.low_opencell_id			= (msgTable[9].data._8[5]);
	input_vector.high_thermistor_ID		= (msgTable[9].data._8[6]);
	input_vector.low_thermistor_ID		= (msgTable[9].data._8[7]);
	
		// BMS 5
	input_vector.high_opencell_voltage 	= (msgTable[10].data._16[0]);
	input_vector.low_opencell_voltage 	= (msgTable[10].data._16[1]);
	input_vector.pack_resistance 				= (msgTable[10].data._16[2]);
	input_vector.pack_summed_voltage 		= (msgTable[10].data._16[3]);
	
		// ------ BAMOCAR inputs ------
		// Have to use memcpy() because it's 16bit data in an 8bit address
		// find conversion rates
		// motor rpm
	memcpy((void*)&input_vector.motor_rpm, (void*)&msgTable[11].data._8[1], sizeof(uint16_t));
		// motor current
	memcpy((void*)&input_vector.motor_current, (void*)&msgTable[12].data._8[1], sizeof(uint16_t));
		// motor torque read value
	memcpy((void*)&input_vector.motor_torque_rdval, (void*)&msgTable[13].data._8[1], sizeof(uint16_t));
		// motor fault
	memcpy((void*)&input_vector.motor_fault, (void*)&msgTable[14].data._8[1], sizeof(uint16_t));
		// motor temperature
	memcpy((void*)&input_vector.motor_temp, (void*)&msgTable[15].data._8[1], sizeof(uint16_t));
		
	
	// End of input assignments
	
}



	
