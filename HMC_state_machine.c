#include <stdint.h>
#include <string.h>
#include "HMC_state_machine.h"
#include "stm32f4xx_can.h"
#include 	"stm32f4xx_can.h"
#include "ring_buffer.h" // change
#include "DFR_SPI.h" // need for ready_to_drive_flag
#include "dfrshifter.h"

#define SUMMED_VOLTAGE_CONVERSION (2^16-1)

float desired_kP(void){

	
	enum gear gear_num;

	/* Do we have to put in some timing thing????*/

	float kP_gear1 = .1;
	float kP_gear2 = .1;
	float kP_gear3 = .1;
	float kP_gear4 = .1;
	float kP_gear5 = .1;
	float kP_invalid = 0;
	float kP;

	gear_num = input_vector.c_gear;
// {Gear1, Gear2, Gear3, Gear4, Gear5, Neutral, Shifting};
	switch(gear_num){

	case Gear1:
		kP = kP_gear1;
		break;
	case Gear2:
		kP = kP_gear2;
		break;
	case Gear3:
		kP = kP_gear3;
		break;
	case Gear4:
		kP = kP_gear4;
		break;
	case Gear5:
		kP = kP_gear5;
		break;
	case Neutral: // step down to Shifting for kP_invalid
	case Shifting:
		kP = kP_invalid;
		break;
	}

	return kP;

}

// uint16_t max_torque = 200; // N*m
uint16_t max_torque = 30; // N*m for testing

#define RPM_MAX (30000.0f)

volatile CanTxMsg bamocar_speed_msg = 
	{
	0x210,								//  uint32_t StdId;  /*!< Specifies the standard identifier.
										//                        This parameter can be a value between 0 to 0x7FF. */
										//
	0,					//  uint32_t ExtId;  /*!< Specifies the extended identifier.
										//                        This parameter can be a value between 0 to 0x1FFFFFFF. */
										//
	CAN_Id_Standard,	//  uint8_t IDE;     /*!< Specifies the type of identifier for the message that 
										//                        will be transmitted. This parameter can be a value 
										//                        of @ref CAN_identifier_type */
										//
	CAN_RTR_Data,			//  uint8_t RTR;     /*!< Specifies the type of frame for the message that will 
										//                        be transmitted. This parameter can be a value of 
										//                        @ref CAN_remote_transmission_request */
										//
	3,								//  uint8_t DLC;     /*!< Specifies the length of the frame that will be 
										//                        transmitted. This parameter can be a value between 
										//                        0 to 8 */
										//
										//  uint8_t Data[8]; /*!< Contains the data to be transmitted. It ranges from 0 
										//                        to 0xFF. */
	{0x31,    0, 0 , 0, 0, 0, 0, 0}
// ^REGID
	};
volatile float speed_command_debug;
volatile uint16_t speed_cmd;


float electric_torque(void){
	
	
	#if 000
	
	float motor_torque;
	
	// should this be max torque????
	motor_torque = (tps/100.0f)*max_torque;
	
	return motor_torque;
	#else
	
	float temp = ((input_vector.accel_pot1-130.0f)/(1300.0f-130.0f))*RPM_MAX;
	
	if(temp > RPM_MAX){
		temp = RPM_MAX;
	}else
	if(temp <= 0)
	{
		temp = 0;
	}
	else
	{
		// leave it alone
	}
	
	speed_cmd = (uint16_t)temp;

	
	bamocar_speed_msg.Data[1]	=	 (speed_cmd & 0x00FF);
	bamocar_speed_msg.Data[2]	=	((speed_cmd & 0xFF00)>>8);
	
	add_to_output_ring(bamocar_speed_msg);
	
	
	speed_command_debug = temp;
	#endif
}

//int get_gear(void){
//	
//	extern input_vector_t input_vector;

//	float motor_shaft_speed = input_vector.motor_rpm; 
//	float engine_rpm_speed = input_vector.engine_rpm;

//	int gear;
//	float current_gear_ratio;
//	float gear_1_ratio = (13.0f/32.0f)*(24.0f/73.0f); // = .13356
//	float gear_2_ratio = (16.0f/32.0f)*(24.0f/73.0f); // = .16438
//	float gear_3_ratio = (17.0f/28.0f)*(24.0f/73.0f); // = .19961
//	float gear_4_ratio = (19.0f/26.0f)*(24.0f/73.0f); // = .24025
//	float gear_5_ratio = (21.0f/25.0f)*(24.0f/73.0f); // = .27616

//	current_gear_ratio = motor_shaft_speed/engine_rpm_speed;

//	if (current_gear_ratio <= gear_1_ratio + .01f && current_gear_ratio >= gear_1_ratio -.01f)			// create .02 band around each gear ratio
//		{
//			gear = 1;
//		}
//		else if (current_gear_ratio <= gear_2_ratio + .01f && current_gear_ratio >= gear_2_ratio -.01f)
//		{
//			gear = 2;
//		}
//		else if (current_gear_ratio <= gear_3_ratio + .01f && current_gear_ratio >= gear_3_ratio -.01f)
//		{
//			gear = 3;
//		}
//		else if (current_gear_ratio <= gear_4_ratio + .01f && current_gear_ratio >= gear_4_ratio -.01f)
//		{
//			gear = 4;
//		}
//		else if (current_gear_ratio <= gear_5_ratio + .01f && current_gear_ratio >= gear_5_ratio -.01f)
//		{
//			gear = 5;
//		}
//		else
//		{
//			gear = 0;
//		}

//	return gear;

//}


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
		uint16_t max_current = 20; // Amps for testing

extern volatile CanTxMsg bamocar_msg;


void torque_command(float motor_torque)
{
	current_output	=	(uint16_t)(motor_torque/kT)*(count/max_current);	
	
	bamocar_msg.Data[1]	=	 (current_output & 0x00FF);
	bamocar_msg.Data[2]	=	((current_output & 0xFF00)>>8);
	
	add_to_output_ring(bamocar_msg);
	
	/*craft CAN message to output to torque */
	/* don't forget that the Unitek uses counts instead of torque */
	
	/* 
	should we put something for demoing in electric mode?
	*/

}

volatile car_mode_t car_mode = car_off; // make external?
car_mode_t old_mode;


typedef enum lock_unlock_e {
	LU_UNLOCK = 0,
	LU_LOCK = 1,
}lock_unlock_t;

lock_unlock_t	lock_state = LU_UNLOCK;

uint8_t push_button_2_z1 = 0;

extern volatile int ready_to_drive_flag;
extern bool safety_init_done_flag;
void lock_unlock_state(void){
	
	switch(lock_state){
		case (LU_UNLOCK):
		{
			car_mode = (car_mode_t)( ((input_vector.motor_mode) |  input_vector.ice_mode << 1) + ((!safety_init_done_flag) << 2 )); // Double check with John to see if this makes sense

			if(input_vector.push_button_2 & ~push_button_2_z1) // detect rising edge
			{
				lock_state = LU_LOCK;
//				car_mode = (car_mode_t)( ((input_vector.motor_mode) |  input_vector.ice_mode << 1) + ((!safety_init_done_flag) << 2 )); // Double check with John to see if this makes sense

			}
			else
			{
				// .. stay in LU_UNLOCK state
			}
		}
		break;
		
		case(LU_LOCK):
		{
			if((!input_vector.motor_mode) && (!input_vector.ice_mode))
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
	old_mode = car_mode;
	
}

extern CAN_msg_t msgTable[];
void assign_inputs(void){
	
	// Input Assignments
		
		// PE3 inputs
  input_vector.engine_rpm = msgTable[0].data._16[0];
	input_vector.engine_MAP = msgTable[1].data._16[1];
	input_vector.engine_temp = msgTable[4].data._16[1];

	input_vector.accel_rdval = msgTable[0].data._16[1];
	
	input_vector.accel_pot1 = msgTable[2].data._16[0];
	input_vector.accel_pot2 = msgTable[2].data._16[1];
	
	input_vector.brake_rdval = msgTable[2].data._16[1];
// has to be remapped to analog input
// input_vector.clutch_rdval = msgTable[2].data._16[2]; 
// input_vector.clutch_pot1 = ....;
// input_vector.clutch_pot2 = ....;

		// might need to create two input values since the reading
		// is on two different analog signals
		

		// BMS 1
	//input_vector.pack_current 		= (short)(msgTable[6].data._16[0]);
	input_vector.pack_current 		=  (msgTable[6].data._8[0] << 8) | (msgTable[6].data._8[1]); // test this
	//input_vector.pack_current 		= (int16_t)(msgTable[6].data._16[0]);
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
//	input_vector.pack_summed_voltage 		= ((65535 - (msgTable[10].data._16[3])));
	input_vector.pack_summed_voltage    = (msgTable[10].data._16[3])/100.0f;
	
	
	
		// ------ BAMOCAR inputs ------
		// Have to use memcpy() because it's 16bit data in an 8bit address
		// find conversion rates
		
		// motor rpm
	memcpy((void*)&input_vector.motor_rpm, (void*)&msgTable[11].data._8[1], sizeof(uint16_t));
		// motor current
	memcpy((void*)&input_vector.motor_current, (void*)&msgTable[12].data._8[1], sizeof(uint16_t));
		// motor torque read value
	memcpy((void*)&input_vector.motor_torque_rdval, (void*)&msgTable[13].data._8[1], sizeof(uint16_t));
		// motor voltage
	memcpy((void*)&input_vector.motor_voltage, (void*)&msgTable[14].data._8[1], sizeof(uint16_t));
		// motor temperature
	memcpy((void*)&input_vector.motor_temp, (void*)&msgTable[15].data._8[1], sizeof(uint16_t));
		
		// motor fault
	memcpy((void*)&input_vector.bamocar_fault, (void*)&msgTable[16].data._8[1], sizeof(uint32_t));
	input_vector.bamocar_fault = input_vector.bamocar_fault & 0x93E70000; // **** MASK TO look at data ****
		
		// bamoar bus voltage
	memcpy((void*)&input_vector.bamocar_bus_voltage, (void*)&msgTable[17].data._8[1], sizeof(uint16_t));
	
		// bamocar DOUT_1 ***** MAKE SURE THIS IS THE RIGHT WAY TO GET DOUT1********
	memcpy((void*)&input_vector.bamocar_dout_1, (void*)&msgTable[18].data._8[1], sizeof(uint16_t));
	
	
	
	// End of input assignments
	
	

	
}



	
