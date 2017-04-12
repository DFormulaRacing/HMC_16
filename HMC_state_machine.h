#include   <stdint.h>
#include   <string.h>

float desired_kP(void);
int get_gear(void);
float get_engine_torque(void);
void torque_command(float motor_torque);

volatile typedef struct input_vector_s {
	
	uint16_t engine_rpm;
	uint16_t engine_MAP;
	uint16_t engine_temp;
	
	uint16_t motor_rpm;
	uint16_t motor_torque_rdval;
	
} input_vector_t;
