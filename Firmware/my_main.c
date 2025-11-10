/*
 * my_main.c
 *
 *  Created on: Oct 18, 2025
 *      Author: bagob
 */

//#include <control_board_s.h>
#include <my_main.h>
#include "main.h"
#include <dmx_usart_s.h>
#include <motor_drive_h_s.h>
//#include <motor_2_drive_s.h>
#define MAX_POS 100000UL
MOTOR_TypeDef Motor_1 = {0};
MOTOR_TypeDef Motor_2 = {0};
MOTOR_TypeDef Motor_3 = {0};
MOTOR_TypeDef Motor_4 = {0};
MOTOR_TypeDef Motor_5 = {0};
MOTOR_TypeDef Motor_6 = {0};
MOTOR_TypeDef Motor_7 = {0};
MOTOR_TypeDef Motor_8 = {0};
/*
#define	PAN_CH 11
#define	PAN_F_CH 12
#define	TILT_CH 13
#define	TILT_F_CH 14
#define	PT_SPEED_CH 15

#define SM_TIME 5							// 70ms egy frame
#define SM_COUNT 17							// 85 ms mintavételezés
#define SM_COUNT_REV 0.058824f				// 17 db interpolálás 5ms enként
*/
/*
static uint16_t dmx_pan_value = 0;
static uint16_t dmx_tilt_value = 0;
static uint8_t  dmx_speed_value = 0;
static uint8_t  reset_f = 0; 				// 0--reset,  > 0 fut a program

static uint16_t dmx_pan_sm_value = 0;
static uint16_t dmx_tilt_sm_value = 0;
*/

static uint16_t pos_1 = 0;
static uint16_t pos_2 = 0;
static uint16_t pos_3 = 0;
static uint16_t pos_4 = 0;
static uint16_t pos_5 = 0;
static uint16_t pos_6 = 0;
static uint16_t pos_7 = 0;
static uint16_t pos_8 = 0;

static void dmx_channel_map(void)
{

	pos_1 = (uint16_t)dmx_array[1];
	pos_2 = (uint16_t)dmx_array[2];
	pos_3 = (uint16_t)dmx_array[3];
	pos_4 = (uint16_t)dmx_array[4];
	pos_5 = (uint16_t)dmx_array[5];
	pos_6 = (uint16_t)dmx_array[6];
	pos_7 = (uint16_t)dmx_array[7];
	pos_8 = (uint16_t)dmx_array[8];
}
/*
static void dmx_signal_smoothing(void)
{
	static uint16_t pan_next = 0;
	static uint16_t pan_prev = 0;
	static uint16_t tilt_next = 0;
	static uint16_t tilt_prev = 0;


	static uint32_t current_time = 0;
	static uint32_t prev_time = 0;
	static uint16_t count = 0;

	current_time = HAL_GetTick();

	if ((uint32_t)(current_time - prev_time) >= SM_TIME)	// időzítés
	{
		prev_time = current_time;
		count++;

		if(count >= SM_COUNT )								// mintavételezés
		{
			count = 0;

			tilt_prev = tilt_next;
			tilt_next = dmx_tilt_value;

			pan_prev = pan_next;
			pan_next = dmx_pan_value;
		}

		float step;											// interpolálás

		step = ((float)tilt_next - tilt_prev) * SM_COUNT_REV; 	// lépéskülönbség / interp. szám
		dmx_tilt_sm_value = (uint16_t)((float)tilt_prev + ((float)count * step));

		step = ((float)pan_next - pan_prev) * SM_COUNT_REV; 	// lépéskülönbség / interp. szám
		dmx_pan_sm_value = (uint16_t)((float)pan_prev + ((float)count * step));
	}
}
*/
void my_main_init(void)
{
	Motor_1.current_pos = 100;
	Motor_2.current_pos = 100;
	Motor_3.current_pos = 100;
	Motor_4.current_pos = 100;
	Motor_5.current_pos = 100;
	Motor_6.current_pos = 100;
	Motor_7.current_pos = 100;
	Motor_8.current_pos = 100;

	Motor_1.interval = 10;
	Motor_2.interval = 10;
	Motor_3.interval = 10;
	Motor_4.interval = 10;
	Motor_5.interval = 10;
	Motor_6.interval = 10;
	Motor_7.interval = 10;
	Motor_8.interval = 10;


	Motor_1.n_max = 200;
	Motor_1.pin_1 = 0;
	Motor_1.pin_2 = 1;

	Motor_2.n_max = 25;
	Motor_2.pin_1 = 2;
	Motor_2.pin_2 = 3;

	Motor_3.n_max = 500;
	Motor_3.pin_1 = 4;
	Motor_3.pin_2 = 5;

	Motor_4.n_max = 50;
	Motor_4.pin_1 = 6;
	Motor_4.pin_2 = 7;

	Motor_5.n_max = 100;
	Motor_5.pin_1 = 8;
	Motor_5.pin_2 = 9;

	Motor_6.n_max = 25;
	Motor_6.pin_1 = 10;
	Motor_6.pin_2 = 11;

	Motor_7.n_max = 200;
	Motor_7.pin_1 = 12;
	Motor_7.pin_2 = 13;

	Motor_8.n_max = 200;
	Motor_8.pin_1 = 14;
	Motor_8.pin_2 = 15;



/*	motor mapping
 * 	motor_1		color_w		pin  0 1
 * 	motor_2		shutter_1	pin  2 3
 * 	motor_3		priz_rot	pin  4 5
 * 	motor_4		priz 		pin  6 7
 * 	motor_5		focus 		pin  8 9
 * 	motor_6		shutter_2	pin  10 11
 * 	motor_7		gobo		pin  12 13
 * 	motor_8		gobo_rot	pin  14 15
 */

}

void my_main_loop(void)
{
	dmx_channel_map();

	static uint32_t tmp_current_time = 0;
	tmp_current_time = HAL_GetTick();
	static uint16_t tmp_pos = 0;

	static uint32_t tmp_time_0 = 0;
	static uint16_t tmp_interval_0 = 5000;

	if ((uint32_t)(tmp_current_time - tmp_time_0)>= tmp_interval_0)
	{
		tmp_time_0 = tmp_current_time;

		if(tmp_pos)tmp_pos = 0;
		else tmp_pos = 80;
	}

	motor_1_main(&Motor_1 , pos_1, 1.5);	// colorw
	motor_1_main(&Motor_2 , pos_2, 2.0);	// shutter_1
	motor_1_main(&Motor_3 , pos_3, 5.0);	// prizrot
	motor_1_main(&Motor_4 , pos_4, 0.5);	//priz
	motor_1_main(&Motor_5 , pos_5, 1.5);	//focus
	motor_1_main(&Motor_6 , pos_6, 2.0);	//shutter2
	motor_1_main(&Motor_7 , pos_7, 1.5);	//gobo
	motor_1_main(&Motor_8 , pos_8, 1.5);	//goborot
}

void motor_refresh_IT(void)			// 1 ms időzítés
{
	static uint32_t Motor_Tick = 0;
	Motor_Tick++;

	Motor_1.current_time = Motor_Tick;
	Motor_2.current_time = Motor_Tick;
	Motor_3.current_time = Motor_Tick;
	Motor_4.current_time = Motor_Tick;
	Motor_5.current_time = Motor_Tick;
	Motor_6.current_time = Motor_Tick;
	Motor_7.current_time = Motor_Tick;
	Motor_8.current_time = Motor_Tick;

	motor_1_update_timer(&Motor_1);
	motor_1_update_timer(&Motor_2);
	motor_1_update_timer(&Motor_3);
	motor_1_update_timer(&Motor_4);
	motor_1_update_timer(&Motor_5);
	motor_1_update_timer(&Motor_6);
	motor_1_update_timer(&Motor_7);
	motor_1_update_timer(&Motor_8);

}
