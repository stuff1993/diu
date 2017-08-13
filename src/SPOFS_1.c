/*
===============================================================================
 Name        : SPOFS_1.c
 Author      : Stuart Gales
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
 */

#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>
#include <stdio.h>

#include "type.h"
#include "iirfilter.h"
#include "inttofloat.h"
#include "timer.h"
#include "lcd.h"
#include "adc.h"
#include "eeprom.h"
#include "struct.h"
#include "dash.h"
#include "inputs.h"
#include "can.h"
#include "menu.h"

MPPT mppt1 =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
MPPT mppt2 =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

MOTORCONTROLLER esc =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

SHUNT shunt =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

BMU bmu =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

CLOCK clock =
{ 0, 0, 0, 0, 0, 0 };

CAN_MSG can_tx1_buf =
{ 0, 0, 0, 0 };
CAN_MSG can_tx2_buf =
{ 0, 0, 0, 0 };

extern CAN_MSG can_rx1_buf;

volatile unsigned char SWITCH_IO  = 0;

uint16_t thr_pos = 0;
uint16_t rgn_pos = 0;


/******************************************************************************
 ** Function:    BOD_IRQHandler
 **
 ** Description: Brown-out detection handler
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void BOD_IRQHandler(void)
{
	HV_OFF
	REVERSE_ON
	NEUTRAL_ON
	REGEN_ON
	DRIVE_ON
	FAULT_ON
	ECO_ON
	SPORTS_ON
}

/******************************************************************************
 ** Function:    SysTick_Handler
 **
 ** Description: System clock event handler. Fires every 10mS
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void SysTick_Handler(void)
{
	clock.t_ms++;
	clock.blink = clock.t_ms / 50;

	// DIU CAN Heart Beat
	// Every 100 mS send heart beat CAN packets
	if ((!(clock.t_ms % 10)) && STATS_ARMED)
	{
		can_tx1_buf.Frame = 0x00080000;
		can_tx1_buf.MsgID = ESC_CONTROL + 1;
		can_tx1_buf.DataA = conv_float_uint(drive.speed_rpm);
		if (drive.current < 0)
		{
			can_tx1_buf.DataB = conv_float_uint(drive.current * -1.0);
		}
		else
		{
			can_tx1_buf.DataB = conv_float_uint(drive.current);
		}
		can1_send_message(&can_tx1_buf);

		// TODO: This may only need to be sent once - investigate
		can_tx1_buf.Frame = 0x00080000;
		can_tx1_buf.MsgID = ESC_CONTROL + 2;
		can_tx1_buf.DataA = 0x0;
		can_tx1_buf.DataB = conv_float_uint(1);
		can1_send_message(&can_tx1_buf);

		// Average power stats
		esc.avg_power += esc.watts;
		mppt1.avg_power += mppt1.watts;
		mppt2.avg_power += mppt2.watts;
		stats.avg_power_counter++;

		// TODO: Test DLC and where data needs to go for 1 byte transmission
		// Light control message
		// If hazards, set both left and right
		can_tx1_buf.Frame = 0x00010000;
		can_tx1_buf.MsgID = DASH_RPLY;
		can_tx1_buf.DataA = STATS_LEFT | (STATS_RIGHT << 1) | (STATS_BRAKE << 2) | STATS_HAZARDS | (STATS_HAZARDS << 1);
		can_tx1_buf.DataB = 0x0;
		can1_send_message(&can_tx1_buf);
	}

	if (stats.buz_tim)
	{
		if (!(--stats.buz_tim))
		{
			BUZZER_OFF
		}
	}

	// Time sensitive Calculations
	esc.watt_hrs += (esc.watts / 360000.0);

	mppt1.watt_hrs += (mppt1.watts / 360000.0);
	mppt2.watt_hrs += (mppt2.watts / 360000.0);

	bmu.watt_hrs += (bmu.watts / 360000.0);

	if (esc.velocity_kmh > 0)
	{
		stats.odometer += esc.velocity_kmh / 360000.0;
		stats.odometer_tr += esc.velocity_kmh / 360000.0;
	}
	else
	{
		stats.odometer -= esc.velocity_kmh / 360000.0;
		stats.odometer_tr -= esc.velocity_kmh / 360000.0;
	}

	// Calculate time
	if (clock.t_ms >= 100)
	{
		clock.t_ms = 0;
		clock.t_s++;

		if ((mppt1.flags & 0x03) > 0)
		{
			// if disconnected for 3 seconds. Then FLAG disconnect.
			mppt1.flags = (mppt1.flags & 0xFC) | ((mppt1.flags & 0x03) - 1);
		}
		if ((mppt2.flags & 0x03) > 0)
		{
			// if disconnected for 3 seconds. Then FLAG disconnect.
			mppt2.flags = (mppt2.flags & 0xFC) | ((mppt2.flags & 0x03) - 1);
		}
		if (shunt.con_tim > 0)
		{
			shunt.con_tim--;
		}

		can_tx1_buf.Frame = 0x00080000;
		can_tx1_buf.MsgID = DASH_RPLY + 3;
		if (stats.avg_power_counter)
		{
			can_tx1_buf.DataA = conv_float_uint(esc.avg_power / stats.avg_power_counter);
		}
		else
		{
			can_tx1_buf.DataA = 0;
		}
		can_tx1_buf.DataB = conv_float_uint((mppt1.watt_hrs + mppt2.watt_hrs));
		can1_send_message(&can_tx1_buf);

		if (stats.avg_power_counter)
		{
			can_tx1_buf.Frame = 0x00080000;
			can_tx1_buf.MsgID = DASH_RPLY + 4;
			can_tx1_buf.DataA = conv_float_uint(mppt1.avg_power / stats.avg_power_counter);
			can_tx1_buf.DataB = conv_float_uint(mppt2.avg_power / stats.avg_power_counter);
			can1_send_message(&can_tx1_buf);
		}

		// Store data in eeprom every second
		persistent_store();

		if (clock.t_s >= 60)
		{
			clock.t_s = 0;
			clock.t_m++;
			if (clock.t_m >= 60)
			{
				clock.t_m = 0;
				clock.t_h++;
				if (clock.t_h >= 24)
				{
					clock.t_h = 0;
					clock.t_d++;
				}
			}
		}
	}
}

/******************************************************************************
 ** Function:    main_driver_check
 **
 ** Description: Checks for input sequence to active test mode
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_driver_check(void)
{
	volatile static uint8_t _seq = 0;
	if(LEFT && _seq == 0){_seq++;}
	else if(LEFT && _seq == 1){;}
	else if(DECREMENT && _seq == 1){_seq++;}
	else if(DECREMENT && _seq == 2){;}
	else if(RIGHT && _seq == 2){_seq++;}
	else if(RIGHT && _seq == 3){;}
	else if(INCREMENT && _seq == 3){_seq++;}
	else if(INCREMENT && _seq == 4){;}
	else if(RIGHT && _seq == 4){_seq++;}
	else if(RIGHT && _seq == 5){;}
	else if(_seq == 5){menu.driver = 9;menu_init();_seq=0;}
	else if(!RIGHT && !INCREMENT && !LEFT && !DECREMENT && !SELECT){;}
	else{_seq = 0;}
}

/******************************************************************************
 ** Function:    main_mppt_poll
 **
 ** Description: Sends request packet to MPPT (125K CAN Bus)
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_mppt_poll(void)
{
	// Check mppt connection timeouts - clear instantaneous data
	if (!(mppt1.flags & 0x03))
	{
		mppt1.v_in = 0;
		mppt1.i_in = 0;
		mppt1.v_out = 0;
		mppt1.watts = 0;
		mppt1.flags = 0;
	}

	if (!(mppt2.flags & 0x03))
	{
		mppt2.v_in = 0;
		mppt2.i_in = 0;
		mppt2.v_out = 0;
		mppt2.watts = 0;
		mppt2.flags = 0;
	}
}

/******************************************************************************
 ** Function:    can1_unpack
 **
 ** Description: Unpacks data received on CAN1
 **
 ** Parameters:  Received CAN message
 ** Return:      None
 **
 ******************************************************************************/
void can1_unpack(CAN_MSG *_msg)
{
	if (_msg->MsgID >= ESC_BASE && _msg->MsgID <= ESC_BASE + 23)
	{
		esc_data_extract(&esc, _msg);
	}
	else if (_msg->MsgID >= DASH_RQST && _msg->MsgID <= DASH_RQST + 1)
	{
		dash_data_extract(_msg);
	}
	else if (_msg->MsgID >= BMU_SHUNT && _msg->MsgID <= BMU_SHUNT + 1)
	{
		shunt_data_extract(&shunt, _msg);
	}
	else if (_msg->MsgID == MPPT1_RPLY)
	{
		mppt_data_extract(&mppt1, _msg);
		//extractMPPT1DATA();
	}
	else if (_msg->MsgID == MPPT2_RPLY)
	{
		mppt_data_extract(&mppt2, _msg);
		//extractMPPT2DATA();
	}
}

/*
void extractMPPT1DATA(void)
{
	uint32_t RawDATA_A = can_rx2_buf.DataA;
	uint32_t RawDATA_B = can_rx2_buf.DataB;

	can_tx1_buf.MsgID = MPPT1_RPLY;
	can_tx1_buf.Frame = 0x00070000;
	can_tx1_buf.DataA = RawDATA_A;
	can_tx1_buf.DataB = RawDATA_B;
	can1_send_message( &can_tx1_buf );

	uint32_t tempVOLT_IN = 0;
	uint32_t tempCURR_IN = 0;
	uint32_t tempVOLT_OUT = 0;

	// Status Flags
	mppt1.flags &= 0xC3;
	mppt1.flags |= ((RawDATA_A & 0xF0) >> 2);

	// Power Variables
	tempVOLT_IN = ((RawDATA_A & 0b11) << 8);  								// Masking and shifting the upper 2 MSB
	tempVOLT_IN = tempVOLT_IN + ((RawDATA_A & 0b1111111100000000) >> 8); 	// Masking and shifting the lower 8 LSB
	tempVOLT_IN = tempVOLT_IN * 1.50;										// SCALING


	RawDATA_A = (RawDATA_A >> 16);
	tempCURR_IN = ((RawDATA_A & 0b11) << 8); 		// Masking and shifting the lower 8 LSB
	tempCURR_IN = tempCURR_IN + ((RawDATA_A & 0b1111111100000000) >> 8);  // Masking and shifting the upper 2 MSB
	tempCURR_IN = tempCURR_IN * 0.87;  										// SCALING

	tempVOLT_OUT = ((RawDATA_B & 0b11) << 8);  									// Masking and shifting the upper 2 MSB
	tempVOLT_OUT = tempVOLT_OUT + ((RawDATA_B & 0b1111111100000000) >> 8); 		// Masking and shifting the lower 8 LSB
	tempVOLT_OUT = tempVOLT_OUT * 2.10;

	mppt1.tmp = (((RawDATA_B & 0b111111110000000000000000) >> 16) + 7 * mppt1.tmp)/8;

	// Update the global variables after IIR filtering
	mppt1.v_in = tempVOLT_IN;	//infinite impulse response filtering
	mppt1.i_in = tempCURR_IN;
	mppt1.v_out = tempVOLT_OUT;
	mppt1.flags |= 0x3;
}

void extractMPPT2DATA(void)
{
	uint32_t RawDATA_A = can_rx2_buf.DataA;
	uint32_t RawDATA_B = can_rx2_buf.DataB;

	can_tx1_buf.MsgID = MPPT2_RPLY;
	can_tx1_buf.Frame = 0x00070000;
	can_tx1_buf.DataA = RawDATA_A;
	can_tx1_buf.DataB = RawDATA_B;
	can1_send_message( &can_tx1_buf );

	uint32_t tempVOLT_IN = 0;
	uint32_t tempCURR_IN = 0;
	uint32_t tempVOLT_OUT = 0;

	// Status Flags
	mppt2.flags &= 0xC3;
	mppt2.flags |= ((RawDATA_A & 0xF0) >> 2);

	// Power Variables
	tempVOLT_IN = ((RawDATA_A & 0b11) << 8);  								// Masking and shifting the upper 2 MSB
	tempVOLT_IN = tempVOLT_IN + ((RawDATA_A & 0b1111111100000000) >> 8); 	// Masking and shifting the lower 8 LSB
	tempVOLT_IN = tempVOLT_IN * 1.50;										// SCALING

	RawDATA_A = (RawDATA_A >> 16);
	tempCURR_IN = ((RawDATA_A & 0b11) << 8); 								// Masking and shifting the lower 8 LSB
	tempCURR_IN = tempCURR_IN + ((RawDATA_A & 0b1111111100000000) >> 8);  	// Masking and shifting the upper 2 MSB
	tempCURR_IN = tempCURR_IN * 0.87;  										// SCALING

	tempVOLT_OUT = ((RawDATA_B & 0b11) << 8);  									// Masking and shifting the upper 2 MSB
	tempVOLT_OUT = tempVOLT_OUT + ((RawDATA_B & 0b1111111100000000) >> 8); 		// Masking and shifting the lower 8 LSB
	tempVOLT_OUT = tempVOLT_OUT * 2.10;

	mppt2.tmp = (((RawDATA_B & 0b111111110000000000000000) >> 16) + 7 * mppt2.tmp)/8;

	// Update the global variables after IIR filtering
	mppt2.v_in = tempVOLT_IN;	//infinite impulse response filtering
	mppt2.i_in = tempCURR_IN;
	mppt2.v_out = tempVOLT_OUT;
	mppt2.flags |= 0x3;
}
 */

/******************************************************************************
 ** Function:    mppt_data_extract
 **
 ** Description: Extracts data from CAN message into MPPT structure.
 ** 			 Uses DriveTek message structure.
 **
 ** Parameters:  1. Address of MPPT to extract to
 ** 			 2. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
void mppt_data_extract(MPPT *_mppt, CAN_MSG *_msg)
{
	// TODO: Test the shit out of this
	uint32_t _v_in = 0;
	uint32_t _i_in = 0;
	uint32_t _v_out = 0;

	uint32_t _data_a = _msg->DataA;
	uint32_t _data_b = _msg->DataB;

	// Status Flags
	_mppt->flags &= 0xC3;
	_mppt->flags |= ((_data_a & 0xF0) >> 2);

	// Power Variables
	_v_in |= ((_data_a & 0x3) << 8);     // Masking and shifting the upper 2 MSB
	_v_in |= ((_data_a & 0xFF00) >> 8);  // Masking and shifting the lower 8 LSB
	_v_in *= 1.50;                        // Scaling

	_data_a = (_data_a >> 16);
	_i_in |= ((_data_a & 0x3) << 8);     // Masking and shifting the lower 8 LSB
	_i_in |= ((_data_a & 0xFF00) >> 8);  // Masking and shifting the upper 2 MSB
	_i_in *= 0.87;                        // Scaling

	_v_out |= ((_data_b & 0x3) << 8);    // Masking and shifting the upper 2 MSB
	_v_out |= ((_data_b & 0xFF00) >> 8); // Masking and shifting the lower 8 LSB
	_v_out *= 2.10;                       // Scaling

	// Update the structure after IIR filtering
	_mppt->tmp = iir_filter_uint(((_data_b & 0xFF0000) >> 16), _mppt->tmp, IIR_GAIN_THERMAL);
	_mppt->v_in = iir_filter_uint(_v_in, _mppt->v_in, IIR_GAIN_ELECTRICAL);
	_mppt->i_in = iir_filter_uint(_i_in, _mppt->i_in, IIR_GAIN_ELECTRICAL);
	_mppt->v_out = iir_filter_uint(_v_out, _mppt->v_out, IIR_GAIN_ELECTRICAL);
	_mppt->flags |= 0x03; // Connection timing bits
}

/******************************************************************************
 ** Function:    esc_data_extract
 **
 ** Description: Extracts data from CAN messages into MOTORCONTROLLER structure.
 **
 ** Parameters:  1. Address of MOTORCONTROLLER to extract to
 ** 			 2. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
void esc_data_extract(MOTORCONTROLLER *_esc, CAN_MSG *_msg)
{
	switch (_msg->MsgID)
	{
	case ESC_BASE + 1:
		_esc->error = (_msg->DataA >> 16);
		if (_esc->error == 0x2)
		{
			NEUTRAL_ON
			REVERSE_ON
			DRIVE_ON
			REGEN_ON
		}
		break;
	case ESC_BASE + 2:
		_esc->bus_v = iir_filter_float(_esc->bus_v, conv_uint_float(_msg->DataA), IIR_GAIN_ELECTRICAL);
		_esc->bus_i = iir_filter_float(_esc->bus_i, conv_uint_float(_msg->DataB), IIR_GAIN_ELECTRICAL);
		break;
	case ESC_BASE + 3:
		_esc->velocity_kmh = conv_uint_float(_msg->DataB) * 3.6;
		break;
	case ESC_BASE + 11:
		_esc->heatsink_tmp = conv_uint_float(_msg->DataB);
		_esc->motor_tmp = conv_uint_float(_msg->DataA);
		break;
	case ESC_BASE + 12:
		_esc->board_tmp = conv_uint_float(_msg->DataA);
		break;
	case ESC_BASE + 14:
		_esc->odometer = conv_uint_float(_msg->DataA);
		break;
	}
}

/******************************************************************************
 ** Function:    dash_data_extract
 **
 ** Description: Extracts data from CAN messages for dash communication.
 **
 ** Parameters:  1. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
void dash_data_extract(CAN_MSG *_msg)
{
	switch (_msg->MsgID)
	{
	case DASH_RQST:
		// Data = KILLDRVE
		if (_msg->DataA == 0x4C4C494B && _msg->DataB == 0x45565244)
		{
			SET_STATS_STOP
		}
		break;
	case DASH_RQST + 1:
		SET_STATS_COMMS
		break;
	}
}

/******************************************************************************
 ** Function:    shunt_data_extract
 **
 ** Description: Extracts data from CAN messages into SHUNT structure.
 **
 ** Parameters:  1. Address of SHUNT to extract to
 ** 			 2. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
void shunt_data_extract(SHUNT *_shunt, CAN_MSG *_msg)
{
	switch (_msg->MsgID)
	{
	case BMU_SHUNT:
		_shunt->bus_v = conv_uint_float(_msg->DataA); // Values filtered on shunt side
		_shunt->bus_i = conv_uint_float(_msg->DataB);
		_shunt->watts = _shunt->bus_i * _shunt->bus_v;
		_shunt->con_tim = 3;
		break;
	case BMU_SHUNT + 1:
		_shunt->watt_hrs = conv_uint_float(_msg->DataA);
		uint32_t _data_b = _msg->DataB;
		if (_data_b & 0x1)
		{
			SET_STATS_ARMED
		}
		else
		{
			CLR_STATS_ARMED
		}
		break;
	}
}

/******************************************************************************
 ** Function:    bmu_data_extract
 **
 ** Description: Extracts data from CAN messages into BMU structure.
 **
 ** Parameters:  1. Address of BMU to extract to
 ** 			 2. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
void bmu_data_extract(BMU *_bmu, CAN_MSG *_msg)
{
	switch (_msg->MsgID)
	{
	case BMU_BASE + BMU_INFO + 4:
		_bmu->min_cell_v = _msg->DataA & 0xFFFF;
		_bmu->max_cell_v = (_msg->DataA >> 16) & 0xFFFF;
		_bmu->cmu_min_v = _msg->DataB & 0xFF;
		_bmu->cmu_max_v = (_msg->DataB >> 16) & 0xFF;
		_bmu->cell_min_v = (_msg->DataB >> 8) & 0xFF;
		_bmu->cell_max_v = (_msg->DataB >> 24) & 0xFF;
		break;
	case BMU_BASE + BMU_INFO + 5:
		_bmu->max_cell_tmp = _msg->DataA & 0xFFFF;
		_bmu->max_cell_tmp = (_msg->DataA >> 16) & 0xFFFF;
		_bmu->cmu_min_tmp = _msg->DataB & 0xFF;
		_bmu->cmu_max_tmp = (_msg->DataB >> 16) & 0xFF;
		break;
	case BMU_BASE + BMU_INFO + 6:
		// Packet is in mV and mA
		_bmu->bus_v = iir_filter_uint(_msg->DataA / 1000, _bmu->bus_v, IIR_GAIN_ELECTRICAL);
		_bmu->bus_i = iir_filter_int(_msg->DataB / 1000, _bmu->bus_i, IIR_GAIN_ELECTRICAL);
		break;
	case BMU_BASE + BMU_INFO + 9:
		_bmu->status = _msg->DataA & 0x7; // Only Voltage and Temperature flags relevant
		break;
	}
}

/******************************************************************************
 ** Function:    main_input_check
 **
 ** Description: Checks majority of inputs (Switches, Left, Right)
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_input_check(void)
{
	unsigned char OLD_IO = SWITCH_IO;
	uint8_t btn_ret = 0;

	SWITCH_IO = 0;
	SWITCH_IO |= (FORWARD << 0);
	SWITCH_IO |= (REVERSE << 1);
	SWITCH_IO |= (SPORTS_MODE << 2);
	SWITCH_IO |= (LEFT_ON << 3);
	SWITCH_IO |= (RIGHT_ON << 4);

	// BEEP if toggle position has changed.
	if(OLD_IO != SWITCH_IO){buzzer(50);}

	if (SWITCH_IO & 0x8)
	{
		SET_STATS_LEFT
	}
	else
	{
		CLR_STATS_LEFT
	}
	if (SWITCH_IO & 0x10)
	{
		SET_STATS_RIGHT
	}
	else
	{
		CLR_STATS_RIGHT
	}


	if((btn_ret = btn_release_left_right()))
	{
		buzzer(2);

		if(btn_ret == 3)     {menu.menu_pos = 0;}
		else if(btn_ret == 1){menu_dec(&menu.menu_pos, menu.menu_items);}
		else if(btn_ret == 2){menu_inc(&menu.menu_pos, menu.menu_items);}

		if(menu.menu_pos==0){buzzer(10);}
		if((esc.error & 0x2) && !STATS_SWOC_ACK){SET_STATS_SWOC_ACK;}
		if((esc.error & 0x1) && !STATS_HWOC_ACK){SET_STATS_HWOC_ACK;BUZZER_OFF}
		if(STATS_COMMS == 1)  // send NO RESPONSE packet
		{
			if((LPC_CAN1->GSR & (1 << 3)))  // Check Global Status Register
			{
				can_tx1_buf.Frame = 0x00010000; // 11-bit, no RTR, DLC is 1 byte
				can_tx1_buf.MsgID = DASH_RPLY + 1;
				can_tx1_buf.DataA = 0x0;
				can_tx1_buf.DataB = 0x0;
				can1_send_message(&can_tx1_buf);
			}
			CLR_STATS_COMMS
		}

		lcd_clear();
		inputs.input_dwn = 0;
		menu.submenu_pos = 0;
		CLR_MENU_SELECTED
	}

	if(SWITCH_IO & 0x4){SET_STATS_DRV_MODE;stats.ramp_speed = SPORTS_RAMP_SPEED;}
	else               {CLR_STATS_DRV_MODE;stats.ramp_speed = ECONOMY_RAMP_SPEED;}

	if (swt_cruise() & 0x0C)
	{
		TOG_STATS_HAZARDS
	}

	if ((MECH_BRAKE || rgn_pos || esc.bus_i < 0))
	{
		SET_STATS_BRAKE
	}
	else
	{
		CLR_STATS_BRAKE
	}
}

/******************************************************************************
 ** Function:    main_fault_check
 **
 ** Description: Checks for faults in car components
 **
 ** Parameters:  None
 ** Return:      Fault status
 **                0 - No fault
 **                1 - Non critical fault
 **                2 - Critical fault - cancel drive
 **
 ******************************************************************************/
int main_fault_check(void)
{
	if (mppt1.i_in == 0 || mppt2.i_in == 0)
	{
		SET_STATS_NO_ARR_HV
	}
	else
	{
		CLR_STATS_NO_ARR_HV
	}
	if (esc.error)
	{
		drive.current = 0;
		drive.speed_rpm = 0;
		return 2;
	}
	if ((mppt1.flags & 0x28) || (mppt2.flags & 0x28) || (bmu.status & 0x7) || (!shunt.con_tim))
	{
		return 1;
	}
	return 0;
}

/******************************************************************************
 ** Function:    main_drive
 **
 ** Description: Reads drive inputs and configures drive packet
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_drive(void)
{
	uint32_t ADC_A;
	uint32_t ADC_B;

	ADC_A = (ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0))/8;

	ADC_B = (ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1))/8;

	main_paddles(ADC_A, ADC_B, &thr_pos, &rgn_pos);

	if((!FORWARD || !menu.driver || STATS_DRV_MODE) && STATS_CR_STS){buzzer(10);CLR_STATS_CR_ACT;CLR_STATS_CR_STS;stats.cruise_speed = 0;} // Must be in forward or not in display mode to use cruise
	if(rgn_pos || thr_pos){CLR_STATS_CR_ACT;}


	// DRIVE LOGIC
	if((!MECH_BRAKE || STATS_DRV_MODE) && (FORWARD || REVERSE)){
		if(STATS_CR_ACT)                                                                                        {drive.current = 1.0;    drive.speed_rpm = stats.cruise_speed / ((60 * 3.14 * WHEEL_D_M) / 1000.0);}
		else if(!thr_pos && !rgn_pos)                                                                           {drive.speed_rpm = 0;    drive.current = 0;}
		else if(rgn_pos)                                                                                        {drive.speed_rpm = 0;    drive.current = -((float)rgn_pos / 1000.0);}
		else if(thr_pos && drive.current < 0)                                                                   {                        drive.current = 0;}
		else if(FORWARD && esc.velocity_kmh > -5.0 && (((drive.current * 1000) + stats.ramp_speed) < thr_pos))  {drive.speed_rpm = 1500; drive.current += (stats.ramp_speed / 1000.0);}
		else if(FORWARD && esc.velocity_kmh > -5.0)                                                             {drive.speed_rpm = 1500; drive.current = (thr_pos / 1000.0);}
		else if(REVERSE && esc.velocity_kmh <  1.0 && (((drive.current * 1000) + stats.ramp_speed) < thr_pos))  {drive.speed_rpm = -200; drive.current += (stats.ramp_speed / 1000.0);}
		else if(REVERSE && esc.velocity_kmh <  1.0)                                                             {drive.speed_rpm = -200; drive.current = (thr_pos / 1000.0);}
		else{drive.speed_rpm = 0; drive.current = 0;}}
	else if(rgn_pos)                                                                                        	{drive.speed_rpm = 0;    drive.current = -((float)rgn_pos / 1000.0);}
	else{drive.speed_rpm = 0; drive.current = 0;CLR_STATS_CR_ACT;}
}

/******************************************************************************
 ** Function:    main_paddles
 **
 ** Description: Takes paddle inputs and returns regen and throttle values
 **              Mode count here must be reflected in lcd_display_options
 **
 ** Parameters:  1. Paddle 1 ADC Read (Right)
 **              2. Paddle 2 ADC Read (Left)
 **              3. Address of throttle variable (Output)
 **              4. Address of regen variable (Output)
 ** Return:      None
 **
 ******************************************************************************/
void main_paddles(uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn)
{
	if((_pad1 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)) || (_pad1 < ((LOW_PAD_V - 0.1) * ADC_POINTS_PER_V))){_pad1 = MID_PAD_V * ADC_POINTS_PER_V;}
	if((_pad2 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)) || (_pad2 < ((LOW_PAD_V - 0.1) * ADC_POINTS_PER_V))){_pad2 = MID_PAD_V * ADC_POINTS_PER_V;}
	switch(stats.paddle_mode)
	{
	case 0:
		// Throttle - Paddle 1
		_pad1 = (_pad1 < ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad1 - ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
		_pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
		if(esc.velocity_kmh < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
		if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
		if(_pad1>1000){_pad1=1000;}
		// Regen - Paddle 2
		_pad2 = (_pad2 < ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad2 - ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V);
		_pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_RGN_DZ) - (MID_PAD_V + MIN_RGN_DZ)) * ADC_POINTS_PER_V);
		if(_pad2>1000){_pad2=1000;}
		_pad2 *= MAX_REGEN / 1000.0;

		*_thr = _pad1;
		*_rgn = _pad2;
		break;
	default:
	case 1:
		if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
		{
			_pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
			_pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
			if(esc.velocity_kmh < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
			if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
			if(_pad1>1000){_pad1=1000;}

			*_thr = _pad1;
			*_rgn = 0;
		}
		else if(_pad1 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 1 Back
		{
			_pad1 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad1;
			_pad1 = (_pad1 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
			if(_pad1>1000){_pad1=1000;}
			_pad1 *= MAX_REGEN / 1000.0;

			*_thr = 0;
			*_rgn = _pad1;
		}
		else{*_thr = 0;*_rgn = 0;} // Centred Paddle
		break;
	case 2:
		if(_pad1 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 1 Back
		{
			_pad1 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad1;
			_pad1 = (_pad1 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
			if(_pad1>1000){_pad1=1000;}
			_pad1 *= MAX_REGEN / 1000.0;

			*_thr = 0;
			*_rgn = _pad1;
		}
		else if(_pad2 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 2 Back
		{
			_pad2 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad2;
			_pad2 = (_pad2 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
			if(_pad2>1000){_pad2=1000;}
			_pad2 *= MAX_REGEN / 1000.0;

			*_thr = 0;
			*_rgn = _pad2;
		}
		else if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
		{
			_pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
			_pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

			if(esc.velocity_kmh < LOWSPD_THRES && _pad1 > MAX_THR_LOWSPD){_pad1 = MAX_THR_LOWSPD;}
			if(!menu.driver && _pad1 > MAX_THR_DISP){_pad1 = MAX_THR_DISP;}
			if(_pad1>1000){_pad1=1000;}

			*_thr = _pad1;
			*_rgn = 0;
		}
		else if(_pad2 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 2 Forward
		{
			_pad2 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
			_pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

			if(esc.velocity_kmh < LOWSPD_THRES && _pad2 > MAX_THR_LOWSPD){_pad2 = MAX_THR_LOWSPD;}
			if(!menu.driver && _pad2 > MAX_THR_DISP){_pad2 = MAX_THR_DISP;}
			if(_pad2>1000){_pad2=1000;}

			*_thr = _pad2;
			*_rgn = 0;
		}
		else{*_thr = 0;*_rgn = 0;} // Centred Paddle
		break;
	}
}

/******************************************************************************
 ** Function:    main_lights
 **
 ** Description: Controls status of on board LEDs
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_lights(void)
{
	if (REVERSE)
	{
		REVERSE_ON
		NEUTRAL_OFF
		DRIVE_OFF
	}
	else if (FORWARD)
	{
		REVERSE_OFF
		NEUTRAL_OFF
		DRIVE_ON
	}
	else
	{
		REVERSE_OFF
		NEUTRAL_ON
		DRIVE_OFF
	}

	if (rgn_pos || esc.bus_i < 0)
	{
		REGEN_ON
	}
	else
	{
		REGEN_OFF
	}

	if ((STATS_LEFT || STATS_HAZARDS) && (clock.blink))
	{
		BLINKER_L_ON
	}
	else
	{
		BLINKER_L_OFF
	}
	if ((STATS_RIGHT || STATS_HAZARDS) && (clock.blink))
	{
		BLINKER_R_ON
	}
	else
	{
		BLINKER_R_OFF
	}

	if (STATS_DRV_MODE)
	{
		SPORTS_ON
		ECO_OFF
	}
	else
	{
		SPORTS_OFF
		ECO_ON
	}

	// TODO: Remove MECH_BRAKE after testing to confirm brake lights. Or leave if worth keeping for driver
	if (STATS_FAULT == 1 || MECH_BRAKE)
	{
		FAULT_ON
	}
	else if (STATS_FAULT == 2 && (clock.blink))
	{
		FAULT_ON
	}
	else
	{
		FAULT_OFF
	}

	if (STATS_ARMED)
	{
		HV_ON
	}
	else
	{
		HV_OFF
	}
}

/******************************************************************************
 ** Function:    main_can_handler
 **
 ** Description: Handles custom CAN packets that aren't handled in can.c
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_can_handler(void)
{
	if(STATS_STOP)
	{
		lcd_clear();

		drive.current = 0;
		drive.speed_rpm = 0;

		if(menu.driver == 3)
		{
			char rot1[20], rot2[20];

			_lcd_putTitle("-GOT DICK-");
			lcd_putstring(1,0, EROW);
			sprintf(rot1, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05, 0x00, 0x02, 0x04, 0x04, 0x05);
			lcd_putstring_custom(2,0, rot1, 20); // does not catch nulls
			sprintf(rot2, "%c%c   %c%c   %c%c   %c%c   ", 0x01, 0x03, 0x01, 0x03, 0x01, 0x03, 0x01, 0x03);
			lcd_putstring(3,0, rot2);
			while(FORWARD || REVERSE)
			{
				_lcd_putTitle("-GOT DICK-");
				_buffer_rotate_right(rot1, 20);
				_buffer_rotate_right(rot2, 20);
				lcd_putstring_custom(2,0, rot1, 20);
				lcd_putstring(3,0, rot2);
				buzzer(5);
				delayMs(1,250);
			}
		}
		else
		{
			while(FORWARD || REVERSE)
			{
				_lcd_putTitle("-KILLDRVE-");
				lcd_putstring(1,0, "--   KILL DRIVE   --");
				lcd_putstring(2,0, "--   KILL DRIVE   --");
				lcd_putstring(3,0, "--   KILL DRIVE   --");
				buzzer(5);
				delayMs(1,250);
			}
		}

		lcd_clear();
	}

	// If only error is SWOC, reset
	if(esc.error == 0x2 && (AUTO_SWOC || menu.driver == 0))
	{
		esc_reset();
		buzzer(50);
		NEUTRAL_OFF
		REVERSE_OFF
		DRIVE_OFF
		REGEN_OFF
	}
}

/******************************************************************************
 ** Function:    main_calc
 **
 ** Description: Calculates instantaneous values and peaks
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void main_calc(void)
{
	// Calculate Power of components
	esc.watts = esc.bus_v * esc.bus_i;

	bmu.watts = bmu.bus_i * bmu.bus_v;

	mppt1.watts = (mppt1.v_in * mppt1.i_in) / 1000.0;
	mppt2.watts = (mppt2.v_in * mppt2.i_in) / 1000.0;

	// Check peaks
	if(esc.watts > esc.max_watts){esc.max_watts = esc.watts;}
	if(esc.bus_i > esc.max_bus_i){esc.max_bus_i = esc.bus_i;}
	if(esc.bus_v > esc.max_bus_v){esc.max_bus_v = esc.bus_v;}
	if(esc.velocity_kmh > stats.max_speed){stats.max_speed = esc.velocity_kmh;}

	if(mppt1.tmp > mppt1.max_tmp){mppt1.max_tmp = mppt1.tmp;}
	if(mppt1.v_in > mppt1.max_v_in){mppt1.max_v_in = mppt1.v_in;}
	if(mppt1.i_in > mppt1.max_i_in){mppt1.max_i_in = mppt1.i_in;}
	if(mppt1.watts > mppt1.max_watts){mppt1.max_watts = mppt1.watts;}

	if(mppt2.tmp > mppt2.max_tmp){mppt2.max_tmp = mppt2.tmp;}
	if(mppt2.v_in > mppt2.max_v_in){mppt2.max_v_in = mppt2.v_in;}
	if(mppt2.i_in > mppt2.max_i_in){mppt2.max_i_in = mppt2.i_in;}
	if(mppt2.watts > mppt2.max_watts){mppt2.max_watts = mppt2.watts;}

	if(bmu.watts > bmu.max_watts){bmu.max_watts = bmu.watts;}
	if(bmu.bus_i > bmu.max_bus_i){bmu.max_bus_i = bmu.bus_i;}
	if(bmu.bus_v > bmu.max_bus_v){bmu.max_bus_v = bmu.bus_v;}

	if(shunt.watts > shunt.max_watts){shunt.max_watts = shunt.watts;}
	if(shunt.bus_i > shunt.max_bus_i){shunt.max_bus_i = shunt.bus_i;}
	if(shunt.bus_v > shunt.max_bus_v){shunt.max_bus_v = shunt.bus_v;}
}

/******************************************************************************
 ** Function:    esc_reset
 **
 ** Description: Resets motorcontroller(s) with CAN packet
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void esc_reset(void)
{
	// RESET MOTOR CONTROLLER(S)
	// see WS22 user manual and Tritium CAN network specs
	// TODO: try MC + 25 (0x19) + msg "RESETWS" (TRI88.004 ver3 doc, July 2013) - 2015
	CAN_MSG reset_msg = { 0x00080000, ESC_CONTROL + 3, 0x0, 0x0 };
	can1_send_message(&reset_msg);
}

/******************************************************************************
 ** Function:    persistent_load
 **
 ** Description: Restores persistent variables from EEPROM
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void persistent_load(void)
{
	stats.flags |= (ee_read(ADD_BUZZ) & 0b1) << 1;
	stats.odometer = conv_uint_float(ee_read(ADD_ODO));
	stats.odometer_tr = conv_uint_float(ee_read(ADD_ODOTR));

	bmu.watt_hrs = conv_uint_float(ee_read(ADD_BMUWHR));
	mppt1.watt_hrs = conv_uint_float(ee_read(ADD_MPPT1WHR));
	mppt2.watt_hrs = conv_uint_float(ee_read(ADD_MPPT2WHR));
}

/******************************************************************************
 ** Function:    persistent_store
 **
 ** Description: Saves persistent variables to EEPROM
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void persistent_store(void)
{
	if (clock.t_s % 2)
	{
		ee_write(ADD_ODO, conv_float_uint(stats.odometer));
		ee_write(ADD_ODOTR, conv_float_uint(stats.odometer_tr));
	}
	else
	{
		ee_write(ADD_BMUWHR, conv_float_uint(bmu.watt_hrs));
		ee_write(ADD_MPPT1WHR, conv_float_uint(mppt1.watt_hrs));
		ee_write(ADD_MPPT2WHR, conv_float_uint(mppt2.watt_hrs));
	}
}

/******************************************************************************
 ** Function:    nonpersistent_load
 **
 ** Description: Loads non-persistent default values
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void nonpersistent_load(void)
{
	menu.flags = 0;
	menu.counter = 0;
	menu.menu_pos = 0;
	menu.submenu_pos = 0;
	menu.submenu_items = 0;
	menu.driver = 255;

	drive.current = 0;
	drive.speed_rpm = 0;

	stats.flags = 0;
	stats.errors = 0;
	stats.max_speed = 0;
	stats.cruise_speed = 0;
	stats.ramp_speed = 5;
	stats.paddle_mode = 1;
}

/******************************************************************************
 ** Function:    gpio_init
 **
 ** Description: Configures pins to be used for GPIO
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void gpio_init(void)
{
	/*
	 * GPIO0:
	 *  PINSEL0:
	 *    0 - IN - RIGHT
	 *    1 - IN - INC/UP
	 *    3 - OUT - Buzzer
	 *    10 - IN - REVERSE
	 *    11 - IN - FORWARD
	 *    15 - OUT - LCD Reset
	 *  PINSEL1:
	 *    16 - OUT - LCD Enable
	 *    25 - IN - Mech Brake
	 *    27 - OUT - Fault LED
	 */
	LPC_GPIO0->FIODIR = (1 << 3) | (1 << 15) | (1 << 16) | (1 << 27);
	LPC_GPIO0->FIOCLR = (1 << 3) | (1 << 15) | (1 << 16) | (1 << 27);

	/*
	 * GPIO1:
	 *  PINSEL2:
	 *    0 - IN - RIGHT_ON
	 *    1 - IN - LEFT_ON
	 *    4 - IN - Power Status
	 *    8 - OUT - Armed Status
	 *  PINSEL3:
	 *    19 - OUT - Blinker R
	 *    20 - OUT - Blinker L
	 *    21 - OUT - Brake Light
	 *    23 - OUT - Reverse LED
	 *    24 - OUT - Neutral LED
	 *    25 - OUT - Regen LED
	 *    26 - OUT - Drive LED
	 *    27 - IN - LEFT
	 *    28 - IN - DEC/DOWN
	 *    29 - IN - SELECT
	 *    30 - OUT - ECO LED
	 *    31 - OUT - SPORTS LED
	 */
	LPC_GPIO1->FIODIR = (1 << 8) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 23)
					| (1 << 24) | (1 << 25) | (1 << 26) | (1 << 30) | (1 << 31);
	LPC_GPIO1->FIOCLR = (1 << 8) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 23)
					| (1 << 24) | (1 << 25) | (1 << 26) | (1 << 30) | (1 << 31);

	/*
	 * GPIO2:
	 *  PINSEL4:
	 *    6 - OUT - LCD D7
	 *    7 - OUT - LCD D6
	 *    8 - OUT - LCD D5
	 *    9 - OUT - LCD D4
	 *    10 - IN - Aux ON (SPORTS MODE)
	 *    11 - IN - Aux OFF
	 *    12 - IN - Hazards
	 *    13 - IN - Hazards
	 */
	LPC_GPIO2->FIODIR = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);
	LPC_GPIO2->FIOCLR = (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);

	/*
	 * GPIO3:
	 *  PINSEL7:
	 *    25 - OUT - Left LED
	 *    26 - OUT - Right LED
	 */
	LPC_GPIO3->FIODIR = (1 << 25) | (1 << 26);
	LPC_GPIO3->FIOCLR = (1 << 25) | (1 << 26);
}

/******************************************************************************
 ** Function:    buzzer
 **
 ** Description: Turns buzzer on for set amount of time
 **
 ** Parameters:  Number of 10mS ticks to sound buzzer
 ** Return:      None
 **
 ******************************************************************************/
void buzzer(uint8_t val)
{
	if (STATS_BUZZER)
	{
		stats.buz_tim = val;
		BUZZER_ON
		;
	}
}

/******************************************************************************
 ** Function:    BOD_Init
 **
 ** Description: Configures BOD Handler
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void BOD_init(void)
{
	// Turn on BOD.
	LPC_SC->PCON &= ~(0x1 << 3);

	// Enable the BOD Interrupt
	NVIC_EnableIRQ(BOD_IRQn);
}

/******************************************************************************
 ** Function:    main
 **
 ** Description: Program entry point. Contains initializations and menu loop
 **
 ** Parameters:  None
 ** Return:      Program exit value
 **
 ******************************************************************************/
int main(void)
{
	SystemInit();
	SystemCoreClockUpdate();

	can1_init(BITRATE500K30MHZ);
	CAN_SetACCF(ACCF_BYPASS);

	I2C1Init();

	nonpersistent_load();
	persistent_load();

	SysTick_Config(SystemCoreClock / 100);  // 10mS Systicker.

	ADCInit(ADC_CLK);

	gpio_init();

	lcd_init();
	lcd_clear();

	BOD_init();

	menu_driver();

	menu_intro();

	menu_init();

	if (FORWARD || REVERSE)
	{
		menu_errOnStart();
	}
	while (FORWARD || REVERSE)
	{
		buzzer(6);
		delayMs(1, 1000);
	}

	// Exiting this loop ends the program
	while (1)
	{
		if ((esc.error & 0x1) && !STATS_HWOC_ACK)
		{
			// on unacknowledged HWOC error, show error screen
			menu.errors[1]();
		}
		else if ((esc.error & 0x2) && !STATS_SWOC_ACK && !AUTO_SWOC && menu.driver)
		{
			// show SWOC screen when auto reset off and not on display mode and error not acknowledged
			menu.errors[0]();
		}
		else if (STATS_COMMS)
		{
			menu.errors[2]();
		}
		else if (bmu.status && !(STATS_BMU_ACK))
		{
			menu.errors[3]();
		}
		else
		{
			if (STATS_SWOC_ACK && !(esc.error & 0x2)) // if acknowledged previous error is reset
			{
				CLR_STATS_SWOC_ACK
			}
			if (STATS_HWOC_ACK && !(esc.error & 0x1)) // if acknowledged previous error is reset
			{
				CLR_STATS_HWOC_ACK
			}
			if (STATS_BMU_ACK && !(bmu.status))
			{
				CLR_STATS_BMU_ACK
			}

			menu.counter++;

			menu.menus[menu.menu_pos]();
		}
		main_mppt_poll();
		main_input_check();
		stats.errors &= 0b11100111;
		stats.errors |= main_fault_check() << 3;
		if (!(stats.errors & (0x2 << 3)))
		{
			// no drive on fault code 2
			main_drive();
		}
		main_lights();
		main_can_handler();
		main_calc();
		main_driver_check();
	}

	return 0; // For compilers sanity
}

