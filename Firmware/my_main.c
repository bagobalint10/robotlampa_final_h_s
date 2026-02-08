/*
 * my_main.c
 *
 *  Created on: Oct 18, 2025
 *      Author: bagob
 */

#include <my_main.h>

#include "main.h"
#include <dmx_usart_s.h>
#include <motor_drive_h_s.h>

#define MAX_POS 100000UL

typedef struct
	{
		int16_t pos;
		float speed;
		uint8_t motor_1_reset_f;

	} Input_parameters;

typedef struct
	{
		Input_parameters input_parameters;
		MOTOR_TypeDef driver_parameters;

	} Motors;

enum  Motor_ids
	{
		MOTOR_1,
		MOTOR_2,
		MOTOR_3,
		MOTOR_4,
		MOTOR_5,
		MOTOR_6,
		MOTOR_7,
		MOTOR_8,
		MOTOR_COUNT
	};

enum Motor_limit_type
	{
		HALL_SENSOR,
		BLOCKING
	};

typedef struct
	{
		char name[20];
		uint8_t limit_type;
		uint8_t pin1, pin2;
		int max_speed;

	} Motor_Config;

static Motor_Config motor_config[MOTOR_COUNT] =
{
	[MOTOR_1] = {.name = "color_wheel", 	.limit_type = HALL_SENSOR, 	.pin1 = 0, 	.pin2 = 1, 	.max_speed = 200},
	[MOTOR_2] = {.name = "shutter_1", 		.limit_type = BLOCKING, 	.pin1 = 2, 	.pin2 = 3, 	.max_speed = 25	},
	[MOTOR_3] = {.name = "prism_rotate", 	.limit_type = BLOCKING,		.pin1 = 4, 	.pin2 = 5, 	.max_speed = 500},
	[MOTOR_4] = {.name = "prism_switch", 	.limit_type = HALL_SENSOR, 	.pin1 = 6, 	.pin2 = 7, 	.max_speed = 50	},
	[MOTOR_5] = {.name = "focus", 			.limit_type = BLOCKING, 	.pin1 = 8, 	.pin2 = 9, 	.max_speed = 100},
	[MOTOR_6] = {.name = "shutter_2", 		.limit_type = BLOCKING, 	.pin1 = 11,	.pin2 = 10, .max_speed = 25	},
	[MOTOR_7] = {.name = "gobo_wheel", 		.limit_type = HALL_SENSOR, 	.pin1 = 12, .pin2 = 13, .max_speed = 200},
	[MOTOR_8] = {.name = "gobo_rotate", 	.limit_type = HALL_SENSOR, 	.pin1 = 14, .pin2 = 15, .max_speed = 200},

};

static Motors motors[MOTOR_COUNT] = {0};

static MOTOR_TypeDef Motor_1 = {0};
static MOTOR_TypeDef Motor_2 = {0};
//static MOTOR_TypeDef Motor_3 = {0};
static MOTOR_TypeDef Motor_4 = {0};
//static MOTOR_TypeDef Motor_5 = {0};
//static MOTOR_TypeDef Motor_6 = {0};
static MOTOR_TypeDef Motor_7 = {0};
static MOTOR_TypeDef Motor_8 = {0};

static int16_t pos_1 = 0;
static int16_t pos_2 = 0;
static int16_t pos_3 = 0;
static int16_t pos_4 = 0;
static int16_t pos_5 = 0;
static int16_t pos_6 = 0;
static int16_t pos_7 = 0;
static int16_t pos_8 = 0;

static float speed_1 = 0;
static float speed_2 = 0;
static float speed_3 = 0;
static float speed_4 = 0;
static float speed_5 = 0;
static float speed_6 = 0;
static float speed_7 = 0;
static float speed_8 = 0;

static uint8_t motor_1_reset_f = 0;
static uint8_t motor_2_reset_f = 0;
static uint8_t motor_4_reset_f = 0;
static uint8_t motor_7_reset_f = 0;
static uint8_t motor_8_reset_f = 0;

static uint8_t reset = 1;

static void dmx_channel_map(void)
{
	static uint16_t colorwheel_array[12] = {128, 261, 394, 527, 660, 793, 926, 1059,1192, 1325, 1458, 1591};
	float tmp;

	speed_1 = 1.5f;
	speed_2 = 5.0f;
	speed_3 = 5.0f;
	speed_4 = 0.5f;
	speed_5 = 1.5f;
	speed_6 = 5.0f;
	speed_7 = 1.5f;
	speed_8 = 1.5f;

	pos_1 = (int16_t)dmx_array[1];
	pos_2 = (int16_t)dmx_array[2];
	pos_3 = (int16_t)dmx_array[3];
	pos_4 = (int16_t)dmx_array[4];
	pos_5 = (int16_t)dmx_array[5];
	pos_6 = (int16_t)dmx_array[6];
	pos_7 = (int16_t)dmx_array[7];
	pos_8 = (int16_t)dmx_array[8];

	// prism on off
	if(pos_4 > 125) pos_4 = 257;
	else 			pos_4 = 0;

	// focus
	tmp = 335.0f/255.0f;
	tmp *= pos_5;
	pos_5 = (int16_t)tmp;

	// strobe 6 pos alapján --> pos 2
	uint8_t tmp_pos;
	uint8_t tmp_pos_6;
	static uint32_t strobe_interval;
	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;
	static uint8_t strobe_f = 0;

	// dimm pos_2 alapján
	tmp_pos_6 = pos_6;
	tmp_pos = pos_2;
	tmp = 100.0f/255.0f;
	tmp *= pos_2;
	pos_2 = (int16_t)tmp;

	tmp = 75.0f/255.0f;
	tmp *= tmp_pos;
	pos_6 = (int16_t)tmp;

	if(tmp_pos_6 >55)
	{
		tmp = 9.0f/201.0f;
		tmp *= ((float)tmp_pos_6-55.0f);
		tmp += 1.0f;

		current_time = HAL_GetTick();
		strobe_interval = (uint32_t)(100.0f * (11.0f - tmp));

		if ((uint32_t)(current_time - prev_time)>= strobe_interval)
		{
			prev_time = current_time;
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			strobe_f = 1;
		}

		if(strobe_f && (Motor_2.current_pos == (pos_2 + 100)))
		{
			strobe_f = 0;
		}

		if(!strobe_f)
		{
			pos_2 = 0;
			pos_6 = 0;
		}
	}
	else
	{

	}

	// prism rotate
	tmp_pos = pos_3;
	if(pos_3 > 55)
	{
		if(pos_3>155)
		{
			pos_3 = 3000;
			// speed szamolas
			tmp = 4.5f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			speed_3 = tmp;
		}
		else
		{
			pos_3 = -3000;
			// speed szamolas
			tmp = 4.5f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			speed_3 = tmp;
		}
	}
	else
	{
		pos_3 = 0;
		speed_3 = 5.0f;
	}

	// prism on off
	tmp_pos = pos_8;
	if(pos_8 > 55)
	{
		if(pos_8>155)
		{
			pos_8 = 3000;
			// speed szamolas
			tmp = 1.0f/100.0f;
			tmp *= ((float)tmp_pos - 155.0f);
			tmp += 0.5f;
			speed_8 = tmp;
		}
		else
		{
			pos_8 = -3000;
			// speed szamolas
			tmp = 1.0f/100.0f;
			tmp *=  ((float)tmp_pos - 55.0f);
			tmp += 0.5f;
			speed_8 = tmp;
		}
	}
	else
	{
		tmp = 800.0f/55.0f;
		tmp *= pos_8;
		pos_8 = (int16_t)tmp;
		speed_8 = 1.5f;
	}

	// color wheel
	if(pos_1 <= 200)
	{
		tmp = 11.0f/201.0f;
		tmp *= pos_1;
		pos_1 = colorwheel_array[(int8_t)tmp];
	}
	else if(pos_1 < 225)
	{
		pos_1 = -3000;
	}
	else
	{
		pos_1 = 3000;
	}

	uint8_t tmp_2 = 0;
	tmp = 7.0f/256.0f;
	tmp *= pos_7;
	tmp_2 = (uint8_t) tmp;

	tmp = tmp_2 * 228.4f;
	tmp += 81.0f;
	pos_7 = (int16_t)tmp;
}

static void reset_fgv(void)
{
	static uint8_t hall_1_edge = 0xff;
	static uint8_t hall_2_edge = 0xff;
	static uint8_t hall_3_edge = 0xff;

	uint8_t hall;

	hall = HAL_GPIO_ReadPin(HALL_1_GPIO_Port, HALL_1_Pin); // gobo
	hall_1_edge = ((hall_1_edge<<1) | hall);

	hall = HAL_GPIO_ReadPin(HALL_2_GPIO_Port, HALL_2_Pin); // prizma
	hall_2_edge = ((hall_2_edge<<1) | hall);

	hall = HAL_GPIO_ReadPin(HALL_3_GPIO_Port, HALL_3_Pin); // color
	hall_3_edge = ((hall_3_edge<<1) | hall);

	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;
	static uint16_t interval = 10000;

	current_time = HAL_GetTick();

	if(((hall_3_edge & 0x03 ) == 0x02) && !motor_1_reset_f)
	{
		motor_1_set_0_pos(&Motor_1);
		pos_1 = 100;
		motor_1_reset_f = 1;
	}
	else if((motor_1_reset_f == 1) && (Motor_1.current_pos == 200))
	{
		pos_1 = -3000;
		motor_1_reset_f = 2;
	}
	else if(((hall_3_edge & 0x03 ) == 0x02) && (motor_1_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_1);
		pos_1 = 0;
		motor_1_reset_f = 3;
	}

	if(((hall_2_edge & 0x03 ) == 0x02) && !motor_4_reset_f)
	{
		motor_1_set_0_pos(&Motor_4);
		pos_4 = 100;
		motor_4_reset_f = 1;
	}
	else if((motor_4_reset_f == 1) && (Motor_4.current_pos == 200))
	{
		pos_4 = -3000;
		motor_4_reset_f = 2;
	}
	else if(((hall_2_edge & 0x03 ) == 0x02) && (motor_4_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_4);
		pos_4 = 0;
		motor_4_reset_f = 3;
	}


	if(((hall_1_edge & 0x03 ) == 0x02) && !motor_7_reset_f)
	{
		motor_1_set_0_pos(&Motor_7);
		pos_7 = 0;
		motor_7_reset_f = 1;

		prev_time = current_time;

	}
	else if(motor_7_reset_f == 1)
	{
		if(hall_1_edge == 0xff)
		{
			motor_7_reset_f = 0;
			pos_7 = -3000;
		}
		else if ((uint32_t)(current_time - prev_time)>= interval)
		{
			pos_7 = 100;
			motor_7_reset_f = 10;
		}
	}
	else if((motor_7_reset_f == 10) && (Motor_7.current_pos == 200))
	{
		pos_7 = -3000;
		motor_7_reset_f = 2;
	}
	else if(((hall_1_edge & 0x03 ) == 0x02)&&(motor_7_reset_f == 2))
	{
		motor_1_set_0_pos(&Motor_7);
		pos_7 = 770;
		motor_7_reset_f = 3;
	}
	else if((motor_7_reset_f == 3)&&(hall_1_edge == 0xff))
	{
		motor_7_reset_f = 4;
	}
	else if((motor_7_reset_f == 4)&&(hall_1_edge == 0x00)&& !motor_8_reset_f && (Motor_7.current_pos == 870))
	{
		motor_1_set_0_pos(&Motor_8);
		pos_8 = 0;
		motor_8_reset_f = 1;
	}

	if((Motor_2.current_pos == 100) && !motor_2_reset_f)
	{
		Motor_2.current_pos = 112;
		motor_2_reset_f = 1;
	}
	if(motor_2_reset_f && motor_8_reset_f && (motor_1_reset_f == 3) &&  (motor_4_reset_f == 3))
	{
		reset = 0;
	}
}

void my_main_init(void)
{
	for(int i = MOTOR_1; i<MOTOR_COUNT; i++)
	{
		// common
		motors[i].input_parameters.speed = 		0.5f;
		motors[i].driver_parameters.interval = 	10;

		// different
		motors[i].driver_parameters.pin_1 = motor_config[i].pin1;
		motors[i].driver_parameters.pin_2 = motor_config[i].pin2;
		motors[i].driver_parameters.n_max = motor_config[i].max_speed;

		if(motor_config[i].limit_type == HALL_SENSOR)
		{
			motors[i].driver_parameters.current_pos = 100;
			motors[i].input_parameters.pos = -3000;
		}
		else
		{
			motors[i].driver_parameters.current_pos = 1500;
			motors[i].input_parameters.pos = 0;
		}
	}
}

void my_main_loop(void)
{
	if(reset)
	{
		reset_fgv();
	}
	else
	{
		dmx_channel_map();
	}

	for (int i = MOTOR_1; i < MOTOR_COUNT; i++)
	{
		motor_1_main(&motors[i].driver_parameters, motors[i].input_parameters.pos, motors[i].input_parameters.speed);
	}
}

void motor_refresh_IT(void)
{
	static uint32_t Motor_Tick = 0;
	Motor_Tick++;

	for (int i = MOTOR_1; i < MOTOR_COUNT; i++)
	{
		motors[i].driver_parameters.current_time = Motor_Tick;
		motor_1_update_timer(&motors[i].driver_parameters);
	}
}
