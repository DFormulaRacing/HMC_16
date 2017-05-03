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
extern volatile car_mode_t mode;
extern volatile	uint32_t		JLM_Debug;

int safety_off_count = 0;
#define SAFETY_OFF_COUNT_LOAD 5

int ready_to_drive_count = 0;
volatile int ready_to_drive_flag = OFF; // make external, use in initialize

bool buzzer_done = false;
bool ready_to_drive_func (void){
	
	if(ready_to_drive_flag){
		if(SPI_output_vector.ready_to_drive == OFF){
			SPI_output_vector.ready_to_drive = ON;
			ready_to_drive_count++;
		} 
		else if(SPI_output_vector.ready_to_drive == ON && ready_to_drive_count >= (20000/50)) { // Nmilliseconds / 50
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


bool pedal_safety_check(void)
{
	tps_weighted = input_vector.accel_rdval/100.0f;
	
	clutch_pot1_weighted = input_vector.clutch_pot1/CLUTCH_POT1_MAX;
	clutch_pot2_weighted = input_vector.clutch_pot2/CLUTCH_POT1_MAX;
	
	accel_pot1_weighted = input_vector.accel_pot1/ACCEL_POT1_MAX;
	accel_pot2_weighted = input_vector.accel_pot2/ACCEL_POT2_MAX;
	
	if(
		(clutch_pot1_weighted <= clutch_pot2_weighted-CLUTCH_TOL) |
		(clutch_pot1_weighted >= clutch_pot2_weighted+CLUTCH_TOL)		) {
			return false;
		}
	else{
		return true;
	}
			
	if(
		(tps_weighted <= accel_pot1_weighted-ACCEL_TOL) |
		(tps_weighted >= accel_pot1_weighted+ACCEL_TOL)		) {
			return false;
		}
	else{
		return true;
	}
	
	
	return true;
}

 bool safety_output_check(void){
	
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
				if (mode == gas) {
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
			(			CLT_Read.bit.PC1
				and	CLT_Read.bit.PC2
				and VNI_Read.bit.PG
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
			
			if(input_vector.push_button_2){
				ready_to_drive_flag = 1;
			}
			else{
				// do nothing
			}
			
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
					safety_state = safety_three;
				} 
				else 
				{
					safety_state = safety_two;
					buzzer_done = ready_to_drive_func();
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
			ready_to_drive_flag = ON;
			
			buzzer_done = ready_to_drive_func();
			
			
			if(
				(mode == hybrid) & (!input_vector.ice_mode)
				or
				(!input_vector.bamocar_dout_1) // 
			) {
				safety_state = safety_init;
			}
			else if
			(			CLT_Read.bit.PC1
				and	CLT_Read.bit.PC2
				and VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				)
			{ 
				if (buzzer_done)
				{
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
			}
			
			if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
				safety_off_count = 0;
				safety_state = safety_fault;
			}
			else{
				safety_state = safety_two;
			}
			
		}
		break;
		
		case safety_four:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = ON;
			ready_to_drive_flag = OFF;
			
			if(
				(mode == hybrid) & (!input_vector.ice_mode)
				or
				(!input_vector.bamocar_dout_1) // 
			) {
				safety_state = safety_init;
			}
			else if
			(			CLT_Read.bit.PC1
				and	CLT_Read.bit.PC2
				and VNI_Read.bit.PG
				and !VNI_Read.bit.TWARN
//				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				and (input_vector.bamocar_bus_voltage <= 14000 && input_vector.bamocar_bus_voltage >= 11000) // voltage check
				and (input_vector.motor_rpm <= 3000) // rpm limiter for driving
			//and (input_vector.motor_rpm <= 500) // rpm limiter for testing/demo for inspection
				)
			{ 
				safety_state = safety_three;
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

