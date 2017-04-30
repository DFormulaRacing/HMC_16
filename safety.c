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
		else if(SPI_output_vector.ready_to_drive == ON && ready_to_drive_count >= (5000/50)) { // Nmilliseconds / 50
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
				and VNI_Read.bit.TWARN
				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag))
			{ // low true, double check if this works with John 
				safety_state = safety_one;
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
				safety_state = safety_off;
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
				and VNI_Read.bit.TWARN
				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
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
			}
			
			if(safety_off_count >= SAFETY_OFF_COUNT_LOAD){
				safety_off_count = 0;
				safety_state = safety_fault;
			}
			else{
				safety_state = safety_one;
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
				and VNI_Read.bit.TWARN
				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
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
			ready_to_drive_flag = ON;
			
			buzzer_done = ready_to_drive_func();
			
			
			if
			(			CLT_Read.bit.PC1
				and	CLT_Read.bit.PC2
				and VNI_Read.bit.PG
				and VNI_Read.bit.TWARN
				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				)
			{ 
				if (buzzer_done)
				{
					safety_state = safety_three;
				} 
				else 
				{
					safety_state = safety_two;
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
		
		case safety_three:
		{
			SPI_output_vector.safety = ON;
			SPI_output_vector.rfg = ON;
			ready_to_drive_flag = OFF;
			
			
			if
			(			CLT_Read.bit.PC1
				and	CLT_Read.bit.PC2
				and VNI_Read.bit.PG
				and VNI_Read.bit.TWARN
				and (VNI_Read.bit.nP0 == ~VNI_Read.bit.P0)
				and (!CAN_error_flag)							// low true, double check if this works with John
				and (!input_vector.bamocar_fault) // fault will fill bit fields, no faults means that 0s in bitfields
				and (input_vector.bamocar_bus_voltage <= 14000 && input_vector.bamocar_bus_voltage >= 10500) // voltage check
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
				safety_state = safety_three;
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

