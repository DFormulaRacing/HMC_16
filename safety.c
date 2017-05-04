#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include <stdbool.h>
#include "safety.h"
#include "HMC_state_machine.h"
#include "DFR_SPI.h"

volatile safety_states_t safety_state = safety_init;

extern bool safety_init_done_flag;
extern volatile	CLT_Read_u_t	CLT_Read;
extern volatile int CAN_error_flag;
extern volatile int ready_to_drive_flag;
extern input_vector_t input_vector;
extern volatile car_mode_t car_mode;
extern volatile	uint32_t		JLM_Debug;

int safety_off_count = 0;
#define SAFETY_OFF_COUNT_LOAD 5
#define BUZZER_TIMER_LOAD 100 // 5000/50 = 100 --> 5 seconds

int ready_to_drive_count = 0;
volatile int ready_to_drive_flag = OFF; // make external, use in initialize

bool buzzer_done = false;
bool ready_to_drive_func (void){
	
	if(ready_to_drive_flag){
		if(SPI_output_vector.ready_to_drive == OFF){
			SPI_output_vector.ready_to_drive = ON;
			ready_to_drive_count++;
		} 
		else if(SPI_output_vector.ready_to_drive == ON && ready_to_drive_count >= (10000/50)) { // Nmilliseconds / 50
			ready_to_drive_flag = OFF;
			SPI_output_vector.ready_to_drive = OFF;
			ready_to_drive_count = 0;
			return true;
		}
		else {
			ready_to_drive_count++;
		}
	} else {
		return false;
	}
	
	
}
// need to set these
const float CLUTCH_POT1_MAX = 100.0f;
const float CLUTCH_POT2_MAX = 100.0f;
const float CLUTCH_TOL = 0.3f;

const float ACCEL_POT1_MAX = 100.0f;
const float ACCEL_POT2_MAX = 100.0f;
const float ACCEL_TOL = 0.3f;

float clutch_pot1_weighted;
float clutch_pot2_weighted;

float accel_pot1_weighted;
float accel_pot2_weighted;
float tps_weighted;

#define ACCEL_1_UPPER_THRESHOLD (950)
#define ACCEL_1_LOWER_THRESHOLD (75)
#define ACCEL_2_UPPER_THRESHOLD (550)
#define ACCEL_2_LOWER_THRESHOLD (75)

volatile bool ACCEL_PADAL_FAULT = false; 

typedef enum accel_pedal_fault_e {
	
	no_fault_state,
	fault_state,
} accel_pedal_fault_t;

accel_pedal_fault_t accel_pedal_fault_state = no_fault_state;

int accel_pedal_fault_state_count = 0;
bool pedal_safety_check(void)
{
	switch(accel_pedal_fault_state){
		case(no_fault_state):
		{
			if((input_vector.accel_pot1 > ACCEL_1_UPPER_THRESHOLD) || (input_vector.accel_pot1 < ACCEL_1_LOWER_THRESHOLD)  ||
				 (input_vector.accel_pot2 > ACCEL_2_UPPER_THRESHOLD) || (input_vector.accel_pot2 < ACCEL_2_LOWER_THRESHOLD)
				)
			{
				accel_pedal_fault_state_count++;
			}
			else
			{
				accel_pedal_fault_state_count = 0;
			}
			
			if(accel_pedal_fault_state_count >= 5){
			
				accel_pedal_fault_state = fault_state;
				ACCEL_PADAL_FAULT = true;
				accel_pedal_fault_state_count = 0;

			}
			else {
				// stay here
			}
		}
		break;
		
		case(fault_state):
		{
			if((input_vector.accel_pot1 > ACCEL_1_UPPER_THRESHOLD) || (input_vector.accel_pot1 < ACCEL_1_LOWER_THRESHOLD)  ||
				 (input_vector.accel_pot2 > ACCEL_2_UPPER_THRESHOLD) || (input_vector.accel_pot2 < ACCEL_2_LOWER_THRESHOLD)
				)
			{
				accel_pedal_fault_state_count = 0;
			}
			else
			{
				accel_pedal_fault_state_count++;
			}
			
			if(accel_pedal_fault_state_count >= 20){
			
				accel_pedal_fault_state = no_fault_state;
				ACCEL_PADAL_FAULT = false;
				accel_pedal_fault_state_count = 0;
			}
			else {
				// stay here
			}
		}
		break;
		
	}
	
	
	

	
	return ACCEL_PADAL_FAULT;
}

int buzzer_timer = 0;
 bool safety_output_check(void){
	
	 if (buzzer_timer){
				buzzer_timer--;
			SPI_output_vector.ready_to_drive = ON;
	 }else{
			SPI_output_vector.ready_to_drive = OFF;
			//
	 }
	 
	switch(safety_state){
		
		case safety_init:
		{
			SPI_output_vector.safety = OFF;
			SPI_output_vector.rfg = OFF;
			ready_to_drive_flag = OFF;  //buzzer
			
			if(safety_init_done_flag){
				safety_state = safety_off;
			} else {
				safety_state = safety_init;  //wait for it
			}
		}
		break;
		
		case safety_off:
		{
			SPI_output_vector.safety = OFF;
			SPI_output_vector.rfg = OFF;
			ready_to_drive_flag = OFF;
			
			if
			(		  VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
				// and (VNI_Read.bit.nP0 == !VNI_Read.bit.P0)
				and (!CAN_error_flag))
			{ // low true, double check if this works with John 
				safety_state = safety_one;
				safety_off_count = 0;
			}
			else
			{
				safety_off_count++;
				if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
					safety_off_count = 0;
					safety_state = safety_fault;
				}
				else{
					safety_state = safety_off;
				}
			}
			
				
			
		}
		break;
		
		case safety_one:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = OFF;
			ready_to_drive_flag = OFF;
			
			if
			(			 VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				)
			{ 
				if (car_mode == gas) {
					safety_state = safety_gas;
				}
				else if (input_vector.push_button_2 == ON) // also means not in gas state
				{
					safety_state = safety_two;
				} 
				else 
				{
					safety_state = safety_one;
				}
			} 
			else
			{
				safety_off_count++;
				if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
					safety_off_count = 0;
					safety_state = safety_fault;
				}
				else{
					safety_state = safety_one;
				}

			}
			
			
		}
		break;
		
		case safety_gas:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = OFF;
			ready_to_drive_flag = OFF;			
			
			if
			(	//		CLT_Read.bit.PC1
				//and	CLT_Read.bit.PC2
						VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				)
			{ 
				safety_state = safety_gas;
			} 
			else
			{
				safety_off_count++;
			}
			
			if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
				safety_off_count = 0;
				safety_state = safety_fault;
			}
			else{
				safety_state = safety_gas;
			}
			
		}
		break;
		
		case safety_two:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = OFF;
			// ready_to_drive_flag = OFF; // ***** PUT BACK!******
			
			
			if
			(			 VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				)
			{ 
				if (input_vector.bamocar_dout_1) 
				{
					buzzer_timer = BUZZER_TIMER_LOAD;
					safety_state = safety_three;
				} 
				else 
				{
					safety_state = safety_two;
				}
			} 
			else
			{
				safety_off_count;// safety_off_count++;
				
				if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
				safety_off_count = 0;
				safety_state = safety_fault;
				}
				else{
					safety_state = safety_one;
				}
			}
			
			
			
		}
		break;
		
		case safety_three:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = OFF;
			
			
			// buzzer_done = ready_to_drive_func();
			
			if(
				(car_mode == hybrid) & (!input_vector.ice_mode)
				or
				(!input_vector.bamocar_dout_1) // 
			) {
				SPI_output_vector.ready_to_drive = OFF;
				safety_state = safety_init;
			}
			else if
			(	//		CLT_Read.bit.PC1
				//and	CLT_Read.bit.PC2
						VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
			//	and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				)
			{ 
				if (buzzer_timer == 0)
				{
					SPI_output_vector.ready_to_drive = OFF;
					safety_state = safety_four;
				} 
				else 
				{
					safety_state = safety_three;
				}
			} 
			else
			{
				safety_off_count++;
				if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
					safety_off_count = 0;
					SPI_output_vector.ready_to_drive = OFF;
					safety_state = safety_fault;
				}
				else{
					safety_state = safety_three;
				}
			}
			
			
			
		}
		break;
		
		case safety_four:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = ON;
			ready_to_drive_flag = OFF;
			
			if(
				(car_mode == hybrid) & (!input_vector.ice_mode)
				or
				(!input_vector.bamocar_dout_1) // 
			) {
				safety_state = safety_init;
			}
			else if
			(	//		CLT_Read.bit.PC1
				//and	CLT_Read.bit.PC2
					VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
//				and (input_vector.bamocar_bus_voltage <= 14000 && input_vector.bamocar_bus_voltage >= 11000) // voltage check
				and (input_vector.motor_rpm <= 3000) // rpm limiter for driving
			//and (input_vector.motor_rpm <= 500) // rpm limiter for testing/demo for inspection
				)
			{ 
				
				safety_state = safety_four;
			} 
			else
			{
				safety_off_count++;
			}
			
			if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
				safety_off_count = 0;
				safety_state = safety_fault;
			}
			else{
				safety_state = safety_four;
			}
			
		}
		break;
		
		case safety_fault:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = ON;
			ready_to_drive_flag = OFF;
		}
		break;
	}
	
	return true;
} 

