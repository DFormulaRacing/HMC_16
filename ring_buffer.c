// rename to input_output.h
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f4xx_can.h"
#include "ring_buffer.h" // will need to change
#include <stdbool.h>

// input ring variables

CanRxMsg buffer[BUFFER_SIZE] = {0} ;
volatile int readIdx = 0;
volatile int writeIdx = 0;
volatile int ringCounter = 0;

// output ring variables

volatile int out_read_idx = 0;
volatile int out_write_idx = 0;
volatile int out_ring_count = 0;
volatile bool flag_can_busy;

volatile CanTxMsg output_ring_buff[OUTPUT_RING_SIZE]; // will need to make output ring buff to handle CAN and non-CAN message

/*
{0x180, STD, 0x30, 0, 1,12, column, "BAMO1",0,0}, // BAMOCAR 1 - RPM
	{0x180, STD, 0x5F, 0, 1,13, column, "BAMO2",0,0}, // BAMOCAR 2 - Motor Current // change to reg 27..?
	{0x180, STD, 0xA0, 0, 1,14, column, "BAMO3",0,0}, // BAMOCAR 3 - Motor Torque
	{0x180, STD, 0x8A, 0, 1,15, column, "BAMO4",0,0}, // BAMOCAR 4 - Motor Voltage
	{0x180, STD, 0x49, 0, 1,16, column, "BAMO5",0,0}, // BAMOCAR 5 - Motor Temp
	{0x180, STD, 0x8F, 0, 1,17, column, "BAMO5",0,0}, // BAMOCAR 6 - BAMOCAR_Fault
	{0x180, STD, 0xEB, 0, 1,18, column, "BAMO5",0,0}, // BAMOCAR 7 - BAMOCAR Bus Voltage
*/

#define TRANSMIT_TO_BAMO_ID 0x210
#define BAMO_READ_REGID 0x3D
#define _100_ms_transmission 0x64 // 0x64 = 100

#define MOTOR_RPM_REGID 0x30
#define MOTOR_CURRENT_REGID 0x5F
#define MOTOR_TORQUE_REGID 0xA0
#define MOTOR_VOLTAGE_REGID 0x8A
#define MOTOR_TEMP_REGID 0x49
#define BAMOCAR_FAULT_REGID 0x8F
#define BAMOCAR_BUS_VOLTAGE_REGID 0xFB

#define	INIT_MESSAGE_COUNT 7
CanTxMsg bamocar_init_msg[INIT_MESSAGE_COUNT] = 
{
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						1
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, MOTOR_RPM_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00} 
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						2
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, MOTOR_CURRENT_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						3
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, MOTOR_TORQUE_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						4
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, MOTOR_VOLTAGE_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						5
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, MOTOR_TEMP_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						6
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, BAMOCAR_FAULT_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
	{		TRANSMIT_TO_BAMO_ID,						//  uint32_t StdId;						7
			0,								//  uint32_t ExtId; 
			CAN_Id_Standard,	//  uint8_t IDE; 											
			CAN_RTR_Data,			//  uint8_t RTR;
			8,								//  uint8_t DLC;
												//  uint8_t Data[8];
			{BAMO_READ_REGID, BAMOCAR_BUS_VOLTAGE_REGID, _100_ms_transmission, 0x00, 0x00, 0x00, 0x00, 0x00}
	},
};



// INPUT + INPUT RING BUFFER STUFF
void addToRing (CanRxMsg x) {
	if(ringCounter >= BUFFER_SIZE) {
		//while(1);
		// BUFFER IS FULL!
		printf("Buffer is full\n");
		return;
	}
	
	buffer[writeIdx] = x;
	writeIdx++;
	if (writeIdx >= BUFFER_SIZE){
		writeIdx = 0;
	}
	ringCounter++;	
}

bool readFromRing (volatile CAN_msg_t *msgTable) {
	if (ringCounter ==0) {
		//printf("Buffer is empty\n");
		return false;
	}

	memcpy((void*)&msgTable->data, (void*)&buffer[readIdx].Data, sizeof(msgTable->data));
			// msgTable[i].data = *((uint64_t *)&(My_RX_message.Data[0]));
			// memcpy(&msgTable[i].data, &My_RX_message.Data[0], sizeof(msgTable[i].data));
	msgTable->update = 1;
	msgTable->msg_count++;
//printf("%llu\n\r", msgTable->data);

	ringCounter--;
	if(ringCounter<0)
	{
		ringCounter=0;
	}
	else
	{
		// do nothing
	}
	
	readIdx++;
	if (readIdx >= BUFFER_SIZE)
	{
		readIdx = 0;
	}

	return true;
}

bool isEmpty (void) {
	if (ringCounter == 0){
		return true;
	} 
	else{
		return false;
	}
}

// OUTPUT + OUTPUT RINGBUFF STUFF
volatile CanTxMsg bamocar_msg = 
	{
	0x180,								//  uint32_t StdId;  /*!< Specifies the standard identifier.
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
	{0x26,    0, 0 , 0, 0, 0, 0, 0}
// ^REGID
	};
	


//volatile int out_read_idx = 0;
//volatile int out_write_idx = 3;
//volatile int out_ring_count = 3;
//volatile bool flag_can_busy;

// change name ... don't think we need this anymore... just need add to ring buff

// **** NOT NEEDED****
/* void send_output_msg(output_vector_t output_vector[]){
	int i;
	for(i = 0; i<OUTPUT_VECTOR_SIZE-1; i++){
		if(output_vector[i].update == 1){
			switch(output_vector[i].select){
				case(motor_torque_out): // CAN MESSAGE for Bamocar torque command
					bamocar_msg.StdId = 0x180;
					bamocar_msg.IDE = CAN_Id_Standard;
					bamocar_msg.RTR = CAN_RTR_Data;
					bamocar_msg.DLC = 3; // or 5
					bamocar_msg.Data[0] = output_vector[i].CANID;
					memcpy((void*)&bamocar_msg.Data[1], (void*)&output_vector[i].data, sizeof(uint16_t));
					// port function from other header -> add_to_output_ring(bamocar_msg);
					output_vector[i].update = 0;
					break;
				case(glvs_shutdown): // GLVS SHUTDOWN
					// 
					output_vector[i].update = 0;
					break;
				case(ready_to_drive): // READY TO DRIVE
					//
					output_vector[i].update = 0;
					break;
				case(shift_up): // SHIFT
					//
					output_vector[i].update = 0;
					break;
				case(shift_down):
					//
					output_vector[i].update = 0;
					break;
			}
		}
	}	
};*/



// **** NOT NEEDED****
// extern output_vector_t output_vector[OUTPUT_VECTOR_SIZE];

// **** NOT NEEDED****
/*void update_output_vector(uint16_t data, output_t select){
	int i;
	
	for(i = 0; i<OUTPUT_VECTOR_SIZE-1;i++){
		if(output_vector[i].select == select){
			output_vector[i].data = data;
			output_vector[i].update = 1;
			
		}
		else{
			// do nothing...
		}
	}
	
	
}*/

bool send_from_output_buff (volatile CanTxMsg output_ring_buff[]) {
	if (out_ring_count ==0) {
		//printf("Buffer is empty\n");
		return false;
	}
	
	CAN_Transmit(CAN1, (CanTxMsg*)&output_ring_buff[out_read_idx]);

	out_ring_count--;
	if(out_ring_count<0)
	{
		ringCounter=0;
	}
	else
	{
		// do nothing
	}
	
	out_read_idx++;
	if (out_read_idx >= OUTPUT_RING_SIZE)
	{
		out_read_idx = 0;
	}

	return true;
}

void add_to_output_ring (CanTxMsg x) {
	if(ringCounter >= OUTPUT_RING_SIZE) {
		//while(1);
		// BUFFER IS FULL!
		printf("output Buffer is full\n");
		return;
	}
	
	output_ring_buff[out_write_idx] = x;
	out_write_idx++;
	out_ring_count++;	
	
	if (out_write_idx >= OUTPUT_RING_SIZE){
		out_write_idx = 0;
	}
	
	if(flag_can_busy){
	// do nothing
	}
	else
	{
		send_from_output_buff(output_ring_buff);
	}
}


int init_index = 0;
bool bamocar_init(void){
	
	if(init_index < INIT_MESSAGE_COUNT) {
		add_to_output_ring(bamocar_init_msg[init_index]);
		init_index++;
		return false; // not done
	} else {
		init_index = 0;
		return true; // done
		
	}
	
}


