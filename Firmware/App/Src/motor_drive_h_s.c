/*
 * motor_drive.c
 *
 * Created: 2025. 10. 14. 16:33:07
 *  Author: bagob
 */ 

#include <motor_drive_h_s.h>
#include "main.h"
#include "math.h"



#define ZERO_POS 100
#define MAX_POS 100000UL
#define T_MAX 109.54451f
//#define DT_0 840000UL 			// 10ms
#define DT_0 84000UL 				// 1ms
//#define TIMER_CONST 644095.0f		// 1/16 max speed --> dt = 70us
#define TIMER_CONST 322047.0f		// 1/32 max speed --> dt = 70us



#define SYMMETRY_CONST 0.1f 	//0.1-0.5

			
 // private 
static uint32_t  calculate_time(MOTOR_TypeDef *Motor, uint32_t n)
{
	float v_s;

	if(!n) return 0; 					//0 sebességnál 0
	else if(n > Motor->n_max) v_s = Motor->v_max ;	//max sebesség felett = max
	else
	{
	 float k 	= ((float)(Motor->n_max - n) * Motor->k_mul) + 1.0f;				// korrigáció
	 float r_n 	= sqrtf((float)n) * k / Motor->r_max;    					// gyök n + korrigáció
	 v_s 		= ((3.0f*r_n*r_n)-( 2.0f*r_n*r_n*r_n)) * Motor->v_max;		// s görbe számítás
	}

	v_s += Motor->v_start;
	float dt = 1.0f / v_s;

	dt *= TIMER_CONST; 					// timer hez illesztés

	return (uint32_t) dt;
}

static void calculate_speed(MOTOR_TypeDef *Motor, uint8_t speed) 	//0-255
{
	// calculate speed v2
	// speed ből --> n_max, v_max, v_start

	Motor->n_max = 8000 + ((uint32_t)((float)speed/255.0f)*12000); //min 8000 - max 30000
	Motor->v_max = 20.0f + (((float)speed/255.0f)*60.0f); // min 20 - max 80
	Motor->v_start = 0.2f +(((float)speed/255.0f)*0.8f); 	// min 0.2 - max 1.5  --> 10ms

	Motor->k_mul   = (SYMMETRY_CONST / (float)Motor->n_max);
	Motor->r_max   = sqrtf((float)Motor->n_max);  				// hogy a végeredmény 0-1 közé essen
	Motor->max_speed_level = Motor->n_max;

}

//public

void motor_1_update_timer(MOTOR_TypeDef *Motor)
{
	// időzítés

	//Motor->current_time = HAL_GetTick();
	//Motor->interval = 1000;

	if ((uint32_t)(Motor->current_time - Motor->prev_time) < Motor->interval)
	{
		return;
	}

	Motor->prev_time = Motor->current_time;
	HAL_GPIO_TogglePin(TEST_OUT_GPIO_Port, TEST_OUT_Pin);


	// motor sebedseg szabalyzas

	if(Motor->motor_enable && Motor->direction)
	{
		Motor->pos_diff = (Motor->input_pos - Motor->current_pos)-1;		// -1 hogy ne l�pjen 1 el t�bbet !

		if ((Motor->pos_diff > Motor->current_level) && (Motor->current_level < Motor->max_speed_level))	// gyorsítás
		{
			Motor->current_level++;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}
		else if((Motor->pos_diff < Motor->current_level) && (Motor->current_level > 0))
		{
			Motor->current_level--;						// lassítás
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}

		Motor->current_pos++;

		if(!Motor->d_t) Motor->d_t = DT_0;						// mintavételezése idő
		//tim_2_set_period(Motor->d_t);
	}
	else if(Motor->motor_enable && (!Motor->direction))
	{
		Motor->pos_diff = (Motor->current_pos - Motor->input_pos)-1 ;	// -1 hogy ne l�pjen 1 el t�bbet !

		if ((Motor->pos_diff > Motor->current_level) && (Motor->current_level < Motor->max_speed_level))
		{
			Motor->current_level++;
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}
		else if((Motor->pos_diff < Motor->current_level) && (Motor->current_level > 0))
		{
			Motor->current_level--;						// lassítás
			Motor->d_t = calculate_time(Motor, Motor->current_level);
		}

		Motor->current_pos--;

		if(!Motor->d_t) Motor->d_t = DT_0;						// mintavételezése idő
		//tim_2_set_period(Motor->d_t);						// timer kompar�l�si ovf �rt�k --> frek frisst�s
	} 

	if ((!Motor->current_level) && Motor->motor_enable)			// ha lenull�z�dott a sebess�g
	{
		Motor->motor_enable = 0;							// motor tilt�sa -> main b�l lehet �jra ind�tani
	}
}

void motor_1_main(MOTOR_TypeDef *Motor, uint16_t dmx_pos_1, uint8_t dmx_speed)
{
	uint32_t tim_state;								// timer jelenlegi �ll�sa
	uint16_t motor_enable_buf;						// timer-enable, szinkroniz�l�s --> bufferel�s

	Motor->input_pos =((int32_t)dmx_pos_1 + ZERO_POS);		// beolvasott poz�ci�

	motor_enable_buf = Motor->motor_enable;

		//n tim_state = tim_2_get_value(); 					// kiolvasni a timer �rt�k�t

	if((!motor_enable_buf) && (Motor->input_pos != Motor->current_pos) && ( tim_state > PWM_ON_DUTY) && ( tim_state < DT_0))
	{	
		//motor ind�t�s + ir�ny meghat�roz�s
		if(Motor->input_pos>Motor->current_pos)	Motor->direction = 1;	// pozitiv  // ir�ny meghat�roz�s
		else 									Motor->direction = 0;	// negat�v

		Motor->motor_enable = 1;							// enged�lyez�s

		calculate_speed(Motor, dmx_speed);

			//n HAL_GPIO_WritePin(MOTOR_1_DIRECTION_GPIO_Port, MOTOR_1_DIRECTION_Pin, direction);
			//n tim_2_set_duty(1);							// PWM kimenet enged�lyez�se
				
	}else if((!motor_enable_buf) && ( tim_state > PWM_ON_DUTY) && ( tim_state < DT_0) )
	{	  
			// timer kimenet letilt�sa --> motor stop
			//n tim_2_set_duty(0);// PWM lev�laszt�sa
	}
}

void motor_1_set_0_pos(MOTOR_TypeDef *Motor) 	// hall hatására fut le --> resetből!!!!
{
	Motor->current_pos = ZERO_POS;
	Motor->current_level = 50;
}


 
