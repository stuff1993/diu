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
#include "i2c.h"
#include "struct.h"
#include "dash.h"
#include "inputs.h"
#include "can.h"
#include "menu.h"

// TODO: MAJOR - naming consistency
// TODO: MAJOR - change I2C to handle multiple words in one write

SHUNT shunt =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

BMU bmu =
{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

CLOCK clock =
{ 0, 0, 0, 0, 0, 0 };

/////////////////////////////   CAN    ////////////////////////////////
CAN_MSG can_tx1_buf, can_tx2_buf; /* TX Buffers for CAN messages */
CAN_MSG can_rx1_buf, can_rx2_buf; /* RX Buffers for CAN messages */
volatile uint32_t can_rx1_done = FALSE, can_rx2_done = FALSE;
///////////////////////////////////////////////////////////////////////

/////////////////////////////   I2C    ////////////////////////////////
extern volatile uint8_t I2CMasterBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CWriteLength[I2C_PORT_NUM];
extern volatile uint32_t I2CReadLength[I2C_PORT_NUM];
extern volatile uint8_t I2CSlaveBuffer[I2C_PORT_NUM][BUFSIZE];
extern volatile uint32_t I2CMasterState[I2C_PORT_NUM]; // TODO: For Timeout test
///////////////////////////////////////////////////////////////////////

volatile unsigned char SWITCH_IO  = 0;

uint16_t thr_pos = 0;
uint16_t rgn_pos = 0;

/// MPPTs
MPPT mppt1;
MPPT mppt2;

/// Motor Controller
MOTORCONTROLLER esc;

/******************************************************************************
 ** Function name:  BOD_IRQHandler
 **
 ** Description:    Brown-out detection handler
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void BOD_IRQHandler (void)
{HV_OFF;REVERSE_ON;NEUTRAL_ON;REGEN_ON;DRIVE_ON;FAULT_ON;ECO_ON;SPORTS_ON;}

/******************************************************************************
 ** Function name:  SysTick_Handler
 **
 ** Description:    System clock event handler. Fires every 10mS
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void SysTick_Handler (void)
{
  clock.T_mS++;

  // MinorSec: DIU CAN Heart Beat
  if((!(clock.T_mS % 10)) && STATS_ARMED) // Every 100 mS send heart beat CAN packets
  {
    can_tx1_buf.Frame = 0x00080000;
    can_tx1_buf.MsgID = ESC_CONTROL + 1;
    can_tx1_buf.DataA = conv_float_uint(drive.speed_rpm);
    if(drive.current < 0){can_tx1_buf.DataB = conv_float_uint(drive.current * -1.0);}
    else{can_tx1_buf.DataB = conv_float_uint(drive.current);}
    can1_send_message( &can_tx1_buf );

    can_tx1_buf.Frame = 0x00080000;
    can_tx1_buf.MsgID = ESC_CONTROL + 2;
    can_tx1_buf.DataA = 0x0;
    can_tx1_buf.DataB = conv_float_uint(1);
    can1_send_message( &can_tx1_buf );

    esc.avg_power += esc.watts;
    mppt1.avg_power += mppt1.watts;
    mppt2.avg_power += mppt2.watts;
    stats.avg_power_counter++;
  }

  if(clock.T_mS / 50)
  {
	  if(!clock.blink)
	  {
		  can_tx2_buf.MsgID = MPPT1_BASE;
		  can2_send_message( &can_tx2_buf );
	  }
	  clock.blink = 1;
  }
  else
  {
	  if(clock.blink)
	  {
		  can_tx2_buf.MsgID = MPPT2_BASE;
		  can2_send_message( &can_tx2_buf );
	  }
	  clock.blink = 0;
  }

  if(stats.buz_tim)
  {
    if(!(--stats.buz_tim)){BUZZER_OFF;}
  }

  if(!stats.strobe_tim){TOG_STATS_STROBE}
  else{CLR_STATS_STROBE}

  // MinorSec:  Time sensitive Calculations
  esc.watt_hrs += (esc.watts/360000.0);

  mppt1.watt_hrs += (mppt1.watts/360000.0);
  mppt2.watt_hrs += (mppt2.watts/360000.0);

  bmu.watt_hrs += (bmu.watts/360000.0);

  if(esc.velocity_kmh > 0){stats.odometer += esc.velocity_kmh/360000.0;stats.odometer_tr += esc.velocity_kmh/360000.0;}
  else{stats.odometer -= esc.velocity_kmh/360000.0;stats.odometer_tr -= esc.velocity_kmh/360000.0;}

  if(clock.T_mS >= 100) // Calculate time
  {
    clock.T_mS = 0;clock.T_S++;

    if((mppt1.flags & 0x03)>0){mppt1.flags = (mppt1.flags & 0xFC) | ((mppt1.flags & 0x03) - 1);} // if disconnected for 3 seconds. Then FLAG disconnect.
    if((mppt2.flags & 0x03)>0){mppt2.flags = (mppt2.flags & 0xFC) | ((mppt2.flags & 0x03) - 1);} // if disconnected for 3 seconds. Then FLAG disconnect.
    if(shunt.con_tim>0){shunt.con_tim--;}
    if(MECH_BRAKE || rgn_pos)
    {
      if(stats.strobe_tim>0){stats.strobe_tim--;}
    }
    else{stats.strobe_tim = 30;}

    can_tx1_buf.Frame = 0x00080000;
    can_tx1_buf.MsgID = DASH_RPLY + 3;
    if(stats.avg_power_counter){can_tx1_buf.DataA = conv_float_uint(esc.avg_power/stats.avg_power_counter);}
    else{can_tx1_buf.DataA = 0;}
    can_tx1_buf.DataB = conv_float_uint((mppt1.watt_hrs + mppt2.watt_hrs));
    can1_send_message( &can_tx1_buf );

    if(stats.avg_power_counter){
    can_tx1_buf.Frame = 0x00080000;
    can_tx1_buf.MsgID = DASH_RPLY + 4;
    can_tx1_buf.DataA = conv_float_uint(mppt1.avg_power/stats.avg_power_counter);
    can_tx1_buf.DataB = conv_float_uint(mppt2.avg_power/stats.avg_power_counter);
    can1_send_message( &can_tx1_buf );}

    persistent_store(); // Store data in eeprom every second

    if(clock.T_S >= 60){clock.T_S = 0;clock.T_M++;
    if(clock.T_M >= 60){clock.T_M = 0;clock.T_H++;
    if(clock.T_H >= 24){clock.T_H = 0;clock.T_D++;}}}
  }
}

/******************************************************************************
 ** Function name:  main_driver_check
 **
 ** Description:    Checks for input sequence to active test mode
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_driver_check (void)
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
  else if(!RIGHT && !INCREMENT && !LEFT && !DECREMENT & !SELECT){;}
  else{_seq = 0;}
}

/******************************************************************************
 ** Function name:  main_mppt_poll
 **
 ** Description:    1. Sends request packet to MPPT (125K CAN Bus)
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_mppt_poll (void)
{
//  TOG_STATS_MPPT_POLL; // Toggle bit. Selects which MPPT to poll this round
//
//  // 1. Sends request packet to MPPT (125K CAN Bus)
//  if((LPC_CAN2->GSR & (1 << 3)) && STATS_MPPT_POLL) // Check Global Status Reg
//  {
//    can_tx2_buf.MsgID = MPPT2_BASE;
//    CAN2_SendMessage( &can_tx2_buf );
//  }
//
//  else if(LPC_CAN2->GSR & (1 << 3)) // Check Global Status Reg
//  {
//    can_tx2_buf.MsgID = MPPT1_BASE;
//    CAN2_SendMessage( &can_tx2_buf );
//  }
//
//  // 2. Sends previous MPPT packet to car (500K CAN Bus)
//  if((LPC_CAN1->GSR & (1 << 3)) && STATS_MPPT_POLL) // Check Global Status Reg
//  {
//    can_tx1_buf.Frame = 0x00070000;  // 11-bit, no RTR, DLC is 7 bytes
//    can_tx1_buf.MsgID = MPPT2_RPLY;
//    if(mppt2.flags & 0x03)
//    {
//      can_tx1_buf.DataA = mppt_relay2.data_a;
//      can_tx1_buf.DataB = mppt_relay2.data_b;
//    }
//    else
//    {
//      can_tx1_buf.DataA = 0x0;
//      can_tx1_buf.DataB = 0x0;
//    }
//    CAN1_SendMessage( &can_tx1_buf );
//  }
//  else if(LPC_CAN1->GSR & (1 << 3)) // Check Global Status Reg
//  {
//    can_tx1_buf.Frame = 0x00070000;  // 11-bit, no RTR, DLC is 7 bytes
//    can_tx1_buf.MsgID = MPPT1_RPLY;
//    if(mppt1.flags & 0x03)
//    {
//      can_tx1_buf.DataA = mppt_relay1.data_a;
//      can_tx1_buf.DataB = mppt_relay1.data_b;
//    }
//    else
//    {
//      can_tx1_buf.DataA = 0x0;
//      can_tx1_buf.DataB = 0x0;
//    }
//    CAN1_SendMessage( &can_tx1_buf );
//  }
//
//  // 3. Receives new packet and extracts data (125K CAN Bus)
//  if ( can_rx2_done == TRUE )
//  {
//    can_rx2_done = FALSE;
//    if      (can_rx2_buf.MsgID == MPPT1_RPLY){mppt_data_extract(&mppt1, &mppt_relay1);}
//    else if (can_rx2_buf.MsgID == MPPT2_RPLY){mppt_data_extract(&mppt2, &mppt_relay2);}
//
//    // Reset buffer to prevent packets being received multiple times
//    can_rx2_buf.Frame = 0x0;
//    can_rx2_buf.MsgID = 0x0;
//    can_rx2_buf.DataA = 0x0;
//    can_rx2_buf.DataB = 0x0;
//  }

  // Check mppt connection timeouts - clear instantaneous data
  if(!(mppt1.flags & 0x03))
  {
    mppt1.v_in = 0;
    mppt1.i_in = 0;
    mppt1.v_out = 0;
    mppt1.watts = 0;
    mppt1.flags = 0;
  }

  if(!(mppt2.flags & 0x03))
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
	else if (_msg->MsgID >= BMU_SHUNT && _msg->MsgID <= BMU_SHUNT + 2)
	{
		shunt_data_extract(&shunt, _msg);
	}
	else if (can_rx2_buf.MsgID == MPPT1_RPLY)
	{
		mppt_data_extract(&mppt1, _msg);
		//extractMPPT1DATA();
	}
	else if (can_rx2_buf.MsgID == MPPT2_RPLY)
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
			can_rx1_done = TRUE;
			NEUTRAL_ON
			;
			REVERSE_ON
			;
			DRIVE_ON
			;
			REGEN_ON
			;
		}
		break;
	case ESC_BASE + 2:
		_esc->bus_v = iir_filter_float(_esc->bus_v,
				conv_uint_float(_msg->DataA), IIR_GAIN_ELECTRICAL);
		_esc->bus_i = iir_filter_float(_esc->bus_i,
				conv_uint_float(_msg->DataB), IIR_GAIN_ELECTRICAL);
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
		can_rx1_done = TRUE;
		break;
	case DASH_RQST + 1:
		SET_STATS_COMMS
		;
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
		shunt.bus_v = conv_uint_float(_msg->DataA); // Values filtered on shunt side
		shunt.bus_i = conv_uint_float(_msg->DataB);
		shunt.watts = shunt.bus_i * shunt.bus_v;
		shunt.con_tim = 3;
		break;
	case BMU_SHUNT + 1:
		shunt.watt_hrs_out = conv_uint_float(_msg->DataA);
		shunt.watt_hrs_in = conv_uint_float(_msg->DataB);
		break;
	case BMU_SHUNT + 2:
		shunt.watt_hrs = conv_uint_float(_msg->DataA);
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
		bmu.min_cell_v = _msg->DataA & 0xFFFF;
		bmu.max_cell_v = (_msg->DataA >> 16) & 0xFFFF;
		bmu.cmu_min_v = _msg->DataB & 0xFF;
		bmu.cmu_max_v = (_msg->DataB >> 16) & 0xFF;
		bmu.cell_min_v = (_msg->DataB >> 8) & 0xFF;
		bmu.cell_max_v = (_msg->DataB >> 24) & 0xFF;
		break;
	case BMU_BASE + BMU_INFO + 5:
		bmu.max_cell_tmp = _msg->DataA & 0xFFFF;
		bmu.max_cell_tmp = (_msg->DataA >> 16) & 0xFFFF;
		bmu.cmu_min_tmp = _msg->DataB & 0xFF;
		bmu.cmu_max_tmp = (_msg->DataB >> 16) & 0xFF;
		break;
	case BMU_BASE + BMU_INFO + 6:
		bmu.bus_v = iir_filter_uint(_msg->DataA / 1000, bmu.bus_v, IIR_GAIN_ELECTRICAL); // Packet is in mV and mA
		bmu.bus_i = iir_filter_int(_msg->DataB / 1000, bmu.bus_i, IIR_GAIN_ELECTRICAL);
		break;
	case BMU_BASE + BMU_INFO + 9:
		bmu.status = _msg->DataA & 0x7; // Only Voltage and Temperature flags relevant
		break;
	}
}

/******************************************************************************
 ** Function name:  main_input_check
 **
 ** Description:    Checks majority of inputs (Switches, Left, Right)
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_input_check (void)
{
  unsigned char OLD_IO = SWITCH_IO;
  uint8_t btn_ret = 0;

  SWITCH_IO = 0;
  SWITCH_IO |= (FORWARD << 0);
  SWITCH_IO |= (REVERSE << 1);
  SWITCH_IO |= (SPORTS_MODE << 2);
  SWITCH_IO |= (LEFT_ON << 3);
  SWITCH_IO |= (RIGHT_ON << 4);

  if(OLD_IO != SWITCH_IO){buzzer(50);}  // BEEP if toggle position has changed.

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
      if((LPC_CAN1->GSR & (1 << 3)))  // Check Global Status Reg
      {
        can_tx1_buf.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
        can_tx1_buf.MsgID = DASH_RPLY + 1;
        can_tx1_buf.DataA = 0x0;
        can_tx1_buf.DataB = 0x0;
        can1_send_message( &can_tx1_buf );
      }
      CLR_STATS_COMMS;
    }

    lcd_clear();
    inputs.input_dwn = 0;
    menu.submenu_pos = 0;
    CLR_MENU_SELECTED;
  }

  if(SWITCH_IO & 0x4) {SET_STATS_DRV_MODE;stats.ramp_speed = SPORTS_RAMP_SPEED;}
  else                {CLR_STATS_DRV_MODE;stats.ramp_speed = ECONOMY_RAMP_SPEED;}

  if(swt_cruise() & 0x0C){TOG_STATS_HAZARDS;}
}

/******************************************************************************
 ** Function name:  main_fault_check
 **
 ** Description:    Checks for faults in car components
 **
 ** Parameters:     None
 ** Returned value: Fault status
 **                   0 - No fault
 **                   1 - Non critical fault
 **                   2 - Critical fault - cancel drive
 **
 ******************************************************************************/
int main_fault_check (void)
{
  if(mppt1.i_in == 0 || mppt2.i_in == 0){SET_STATS_NO_ARR_HV;}
  else{CLR_STATS_NO_ARR_HV;}
  if(esc.error){drive.current = 0;drive.speed_rpm = 0;return 2;}
  if((mppt1.flags & 0x28) || (mppt2.flags & 0x28) || (bmu.status & 0x7) || (!shunt.con_tim)){return 1;}
  return 0;
}

/******************************************************************************
 ** Function name:  main_drive
 **
 ** Description:    Reads drive inputs and configures drive packet
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_drive (void)
{
  uint32_t ADC_A;
  uint32_t ADC_B;

  /// THROTTLE
  ADC_A = (ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0))/8;

  /// REGEN
  ADC_B = (ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1))/8;

  main_paddles(ADC_A, ADC_B, &thr_pos, &rgn_pos);

  if((!FORWARD || !menu.driver || STATS_DRV_MODE) && STATS_CR_STS){buzzer(10);CLR_STATS_CR_ACT;CLR_STATS_CR_STS;stats.cruise_speed = 0;} // Must be in forward or not in display mode to use cruise
  if(rgn_pos || thr_pos){CLR_STATS_CR_ACT;}


  // MinorSec: DRIVE LOGIC
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
  else if(rgn_pos)                                                                                        {drive.speed_rpm = 0;    drive.current = -((float)rgn_pos / 1000.0);}
  else{drive.speed_rpm = 0; drive.current = 0;CLR_STATS_CR_ACT;}
}

/******************************************************************************
 ** Function name:  main_paddles
 **
 ** Description:    Takes paddle inputs and returns regen and throttle values
 **                 Mode count here must be reflected in lcd_display_options
 **
 ** Parameters:     1. Paddle 1 ADC Read (Right)
 **                 2. Paddle 2 ADC Read (Left)
 **                 3. Address of throttle variable (Output)
 **                 4. Address of regen variable (Output)
 ** Returned value: None
 **
 ******************************************************************************/
void main_paddles (uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn)
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
 ** Function name:  main_lights
 **
 ** Description:    Controls status of lights and LEDs
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_lights (void)
{
  if((MECH_BRAKE || rgn_pos || esc.bus_i < 0) && !STATS_STROBE){BRAKELIGHT_ON;}
  else                                        {BRAKELIGHT_OFF;}

  if(!rgn_pos)
  {
    if(REVERSE)     {REVERSE_ON;NEUTRAL_OFF;REGEN_OFF;DRIVE_OFF;}
    else if(FORWARD){REVERSE_OFF;NEUTRAL_OFF;REGEN_OFF;DRIVE_ON;}
    else            {REVERSE_OFF;NEUTRAL_ON;REGEN_OFF;DRIVE_OFF;}
  }
  else
  {
    if(REVERSE)     {REVERSE_ON;NEUTRAL_OFF;REGEN_ON;DRIVE_OFF;}
    else if(FORWARD){REVERSE_OFF;NEUTRAL_OFF;REGEN_ON;DRIVE_ON;}
    else            {REVERSE_OFF;NEUTRAL_ON;REGEN_OFF;DRIVE_OFF;}
  }

  if(((SWITCH_IO & 0x8) || STATS_HAZARDS) && (clock.blink)){BLINKER_L_ON}
  else{BLINKER_L_OFF}
  if(((SWITCH_IO & 0x10) || STATS_HAZARDS) && (clock.blink)){BLINKER_R_ON}
  else{BLINKER_R_OFF}

  if(STATS_DRV_MODE) {SPORTS_ON;ECO_OFF;}
  else                {SPORTS_OFF;ECO_ON;}

  if(STATS_FAULT == 1){FAULT_ON}
  else if(STATS_FAULT == 2 && (clock.blink)){FAULT_ON}
  else{FAULT_OFF}
}

/******************************************************************************
 ** Function name:  main_can_handler
 **
 ** Description:    Handles custom CAN packets that aren't handled in can.c
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_can_handler (void)
{
  /*
   * List of packets handled:
   * Kill drive - 0x520: Use loop to prevent drive logic running. Not done in can.c to release Rx buffer
   * SWOC Error - 0x401: Send MC reset packet
   */
  if(can_rx1_done == TRUE)
  {
    can_rx1_done = FALSE;

    if((can_rx1_buf.MsgID == DASH_RQST) && (can_rx1_buf.DataA == 0x4C4C494B) && (can_rx1_buf.DataB == 0x45565244)) // Data = KILLDRVE
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
    else if(can_rx1_buf.MsgID == ESC_BASE + 1 && esc.error == 0x2 && (AUTO_SWOC || menu.driver == 0))
    {
      if(esc.error == 0x2)
      {
        esc_reset();
        buzzer(50);
        NEUTRAL_OFF;REVERSE_OFF;DRIVE_OFF;REGEN_OFF;
      }
    }

    // Clear Rx Buffer
    can_rx1_buf.Frame = 0x0;
    can_rx1_buf.MsgID = 0x0;
    can_rx1_buf.DataA = 0x0;
    can_rx1_buf.DataB = 0x0;
  }
}

/******************************************************************************
 ** Function name:  main_calc
 **
 ** Description:    Calculates instantaneous values and peaks
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_calc (void)
{
  // Calculate Power of components
  esc.watts = esc.bus_v * esc.bus_i;

  bmu.watts = bmu.bus_i * bmu.bus_v;

  mppt1.watts = (mppt1.v_in * mppt1.i_in) / 1000.0; // TODO: adjust for efficiency?
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
 ** Function name:  main_HV
 **
 ** Description:    Controls HV contactor
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void main_HV (void)
{
  if(esc.bus_v > 90 && esc.bus_v > (0.9 * shunt.bus_v) && esc.bus_v < (1.1 * shunt.bus_v) && stats.hv_counter<1100){stats.hv_counter++;}
  else if(stats.hv_counter){stats.hv_counter--;}

  if(stats.hv_counter>1000 && !STATS_ARMED){buzzer(50);SET_STATS_ARMED;HV_ON;}
  else if(stats.hv_counter<100){CLR_STATS_ARMED;HV_OFF;}
}

/******************************************************************************
 ** Function name:  esc_reset
 **
 ** Description:    Resets motorcontroller(s) with CAN packet
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void esc_reset (void)
{
  // RESET MOTOR CONTROLLERS
  // see WS22 user manual and Tritium CAN network specs
  // try MC + 25 (0x19) + msg "RESETWS" (TRI88.004 ver3 doc, July 2013)
  can_tx1_buf.Frame = 0x00080000;  // 11-bit, no RTR, DLC is 8 bytes
  can_tx1_buf.MsgID = ESC_CONTROL + 3;
  can_tx1_buf.DataB = 0x0;
  can_tx1_buf.DataA = 0x0;
  can1_send_message( &can_tx1_buf );
}

/******************************************************************************
 ** Function name:  EE_Read
 **
 ** Description:    Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:     Address to read from
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t EE_read (uint16_t _EEadd)
{
  uint32_t retDATA = 0;

  retDATA = I2C_read(_EEadd+3);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+2);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+1);
  retDATA = (retDATA << 8) + I2C_read(_EEadd+0);

  return retDATA;
}

/******************************************************************************
 ** Function name:  EE_Seq_Read
 **
 ** Description:    Reads a word from EEPROM (Uses I2CRead)
 **
 ** Parameters:     1. Address to read from
 **                 2. Byte length of read
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t EE_seq_read (uint16_t _EEadd, int _len)
{
  uint32_t _ret = 0;

  I2C_seq_read(_EEadd, _len);
  while(_len--)
  {
    _ret += I2CSlaveBuffer[PORT_USED][_len] << (_len * 8);
  }

  return _ret;
}

/******************************************************************************
 ** Function name:  EE_Write
 **
 ** Description:    Saves a word to EEPROM (Uses I2CWrite)
 **
 ** Parameters:     1. Address to save to
 **                 2. Data to save (convert to uint with converter first)
 ** Returned value: None
 **
 ******************************************************************************/
void EE_write (uint16_t _EEadd, uint32_t _EEdata)
{
  uint8_t temp0 = (_EEdata & 0x000000FF);
  uint8_t temp1 = (_EEdata & 0x0000FF00) >> 8;
  uint8_t temp2 = (_EEdata & 0x00FF0000) >> 16;
  uint8_t temp3 = (_EEdata & 0xFF000000) >> 24;

  I2C_write(_EEadd, temp0,temp1,temp2,temp3);
}

/******************************************************************************
 ** Function name:  I2C_Read
 **
 ** Description:    Reads a byte from EEPROM
 **
 ** Parameters:     Address to read from
 ** Returned value: Data at address
 **
 ******************************************************************************/
uint32_t I2C_read (uint16_t _EEadd)
{
  int i;

  for ( i = 0; i < BUFSIZE; i++ ) // clear buffer
  {
    I2CMasterBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = 1;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);

  // TODO: Test I2C timeouts
  // if(I2CMasterState[PORT_USED] == I2C_TIME_OUT){SET_STATS_SWOC_ACK;}

  return (uint32_t)I2CSlaveBuffer[PORT_USED][0];
}

/******************************************************************************
 ** Function name:  I2C_Seq_Read
 **
 ** Description:    Reads a byte from EEPROM
 **
 ** Parameters:     1. Address to read from
 **                 2. Byte length of read
 ** Returned value: None
 **
 ******************************************************************************/
void I2C_seq_read (uint16_t _EEadd, int read_len)
{
  int i;
  for ( i = 0; i < BUFSIZE; i++ ) // clear buffer
  {
    I2CSlaveBuffer[PORT_USED][i] = 0;
  }

  I2CWriteLength[PORT_USED] = 3;
  I2CReadLength[PORT_USED] = read_len;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = _24LC256_ADDR | RD_BIT;
  I2CEngine( PORT_USED );
  I2CStop(PORT_USED);
}

/******************************************************************************
 ** Function name:  I2C_Write
 **
 ** Description:    Saves a word to EEPROM
 **
 ** Parameters:     1. Address to save to
 **                 2. Data to save
 ** Returned value: None
 **
 ******************************************************************************/
void I2C_write (uint16_t _EEadd, uint8_t data0, uint8_t data1, uint8_t data2, uint8_t data3)
{
  I2CWriteLength[PORT_USED] = 7;
  I2CReadLength[PORT_USED] = 0;
  I2CMasterBuffer[PORT_USED][0] = _24LC256_ADDR;
  I2CMasterBuffer[PORT_USED][1] = (_EEadd & 0x0f00) >> 8; // address
  I2CMasterBuffer[PORT_USED][2] = _EEadd & 0x00ff;        // address
  I2CMasterBuffer[PORT_USED][3] = data0;
  I2CMasterBuffer[PORT_USED][4] = data1;
  I2CMasterBuffer[PORT_USED][5] = data2;
  I2CMasterBuffer[PORT_USED][6] = data3;
  I2CEngine( PORT_USED );

  // TODO: Test I2C timeouts
  // if(I2CMasterState[PORT_USED] == I2C_TIME_OUT){SET_STATS_HWOC_ACK;}

  delayMs(1,2);
}

/******************************************************************************
 ** Function name:  load_persistent
 **
 ** Description:    Restores persistent variables from EEPROM
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void persistent_load (void)
{
  stats.flags |= (EE_read(ADD_BUZZ) & 0b1) << 1;
  stats.odometer = conv_uint_float(EE_read(ADD_ODO));
  stats.odometer_tr = conv_uint_float(EE_read(ADD_ODOTR));

  bmu.watt_hrs = conv_uint_float(EE_read(ADD_BMUWHR));
  mppt1.watt_hrs = conv_uint_float(EE_read(ADD_MPPT1WHR));
  mppt2.watt_hrs = conv_uint_float(EE_read(ADD_MPPT2WHR));
}

/******************************************************************************
 ** Function name:  store_persistent
 **
 ** Description:    Saves persistent variables to EEPROM
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void persistent_store (void)
{
  if(clock.T_S % 2)
  {
    EE_write(ADD_ODO, conv_float_uint(stats.odometer));
    EE_write(ADD_ODOTR, conv_float_uint(stats.odometer_tr));
  }
  else
  {
    EE_write(ADD_BMUWHR, conv_float_uint(bmu.watt_hrs));
    EE_write(ADD_MPPT1WHR, conv_float_uint(mppt1.watt_hrs));
    EE_write(ADD_MPPT2WHR, conv_float_uint(mppt2.watt_hrs));
  }
}

/******************************************************************************
 ** Function name:  load_nonpersistent
 **
 ** Description:    Loads non-persistent default values
 **
 ** Parameters:     None
 ** Returned value: None
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

  clock.T_D = 0;
  clock.T_H = 0;
  clock.T_M = 0;
  clock.T_S = 0;
  clock.T_mS = 0;
  clock.blink = 0;

  drive.current = 0;
  drive.speed_rpm = 0;

  stats.flags = 0;
  stats.errors = 0;
  stats.max_speed = 0;
  stats.cruise_speed = 0;
  stats.ramp_speed = 5;
  stats.strobe_tim = 30;
  stats.paddle_mode = 1;

  bmu.bus_i = 0;
  bmu.bus_v = 0;
  bmu.watts = 0;
  bmu.status = 0;
  bmu.max_bus_i = 0;
  bmu.max_bus_v = 0;
  bmu.max_watts = 0;
  bmu.max_cell_v = 0;
  bmu.min_cell_v = 0;
  bmu.max_cell_tmp = 0;
  bmu.min_cell_tmp = 0;

  esc.bus_i = 0;
  esc.bus_v = 0;
  esc.error = 0;
  esc.watts = 0;
  esc.watt_hrs = 0;
  esc.max_bus_i = 0;
  esc.max_bus_v = 0;
  esc.max_watts = 0;
  esc.velocity_kmh = 0;
  esc.motor_tmp = 0;
  esc.board_tmp = 0;
  esc.avg_power = 0;

  mppt1.tmp = 0;
  mppt1.v_in = 0;
  mppt1.i_in = 0;
  mppt1.v_out = 0;
  mppt1.watts = 0;
  mppt1.flags = 0;
  mppt1.max_tmp = 0;
  mppt1.max_i_in = 0;
  mppt1.max_v_in = 0;
  mppt1.max_v_out = 0;
  mppt1.max_watts = 0;
  mppt1.avg_power = 0;

  mppt2.tmp = 0;
  mppt2.v_in = 0;
  mppt2.i_in = 0;
  mppt2.v_out = 0;
  mppt2.watts = 0;
  mppt2.flags = 0;
  mppt2.max_tmp = 0;
  mppt2.max_i_in = 0;
  mppt2.max_v_in = 0;
  mppt2.max_v_out = 0;
  mppt2.max_watts = 0;
  mppt2.avg_power = 0;

  shunt.bus_i = 0;
  shunt.bus_v = 0;
  shunt.con_tim = 0;
  shunt.watt_hrs = 0;
  shunt.watt_hrs_in = 0;
  shunt.watt_hrs_out = 0;

  can_tx1_buf.Frame = 0x00080000;
  can_tx1_buf.MsgID = 0x0;
  can_tx1_buf.DataA = 0x0;
  can_tx1_buf.DataB = 0x0;

  can_rx1_buf.Frame = 0x0;
  can_rx1_buf.MsgID = 0x0;
  can_rx1_buf.DataA = 0x0;
  can_rx1_buf.DataB = 0x0;

  can_tx2_buf.Frame = 0x00080000;
  can_tx2_buf.MsgID = MPPT1_BASE;
  can_tx2_buf.DataA = 0x0;
  can_tx2_buf.DataB = 0x0;

  can_rx2_buf.Frame = 0x0;
  can_rx2_buf.MsgID = 0x0;
  can_rx2_buf.DataA = 0x0;
  can_rx2_buf.DataB = 0x0;
}

/******************************************************************************
 ** Function name:  init_GPIO
 **
 ** Description:    Configures pins to be used for GPIO
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void gpio_init (void)
{
  /* GPIO0:
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
  LPC_GPIO0->FIODIR = (1<<3)|(1<<15)|(1<<16)|(1<<27);
  LPC_GPIO0->FIOCLR = (1<<3)|(1<<15)|(1<<16)|(1<<27);

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
  LPC_GPIO1->FIODIR = (1<<8)|(1<<19)|(1<<20)|(1<<21)|(1<<23)|(1<<24)|(1<<25)|(1<<26)|(1<<30)|(1<<31);
  LPC_GPIO1->FIOCLR = (1<<8)|(1<<19)|(1<<20)|(1<<21)|(1<<23)|(1<<24)|(1<<25)|(1<<26)|(1<<30)|(1<<31);

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
  LPC_GPIO2->FIODIR = (1<<6)|(1<<7)|(1<<8)|(1<<9);
  LPC_GPIO2->FIOCLR = (1<<6)|(1<<7)|(1<<8)|(1<<9);

  /*
   * GPIO3:
   *  PINSEL7:
   *    25 - OUT - Left LED
   *    26 - OUT - Right LED
   */
  LPC_GPIO3->FIODIR = (1<<25)|(1<<26);
  LPC_GPIO3->FIOCLR = (1<<25)|(1<<26);
}

/******************************************************************************
 ** Function name:  buzzer
 **
 ** Description:    Turns buzzer on for set amount of time
 **
 ** Parameters:     Number of 10mS ticks to sound buzzer
 ** Returned value: None
 **
 ******************************************************************************/
void buzzer (uint8_t val)
{
  if(STATS_BUZZER)
  {
    stats.buz_tim = val;
    BUZZER_ON;
  }
}

/******************************************************************************
 ** Function name:  BOD_Init
 **
 ** Description:    Configures BOD Handler
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void BOD_init ( void )
{
  // Turn on BOD.
  LPC_SC->PCON &= ~(0x1<<3);

  // Enable the BOD Interrupt
  NVIC_EnableIRQ(BOD_IRQn);
}

/******************************************************************************
 ** Function name:  main
 **
 ** Description:    Program entry point. Contains initializations and menu loop
 **
 ** Parameters:     None
 ** Returned value: Program exit value
 **
 ******************************************************************************/
int main (void)
{
  SystemInit();
  SystemCoreClockUpdate();

  can1_init( BITRATE500K30MHZ );
  can2_init( BITRATE125K30MHZ );
  CAN_SetACCF( ACCF_BYPASS );

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

  if(FORWARD || REVERSE){menu_errOnStart();}
  while(FORWARD || REVERSE){buzzer(6);delayMs(1, 1000);}

  while(1){ // Exiting this loop ends the program
    if((esc.error & 0x1) && !STATS_HWOC_ACK) // on unacknowledged HWOC error, show error screen
    {menu.errors[1]();}
    else if((esc.error & 0x2) && !STATS_SWOC_ACK && !AUTO_SWOC && menu.driver) // show SWOC screen when auto reset off and not on display mode and error not acknowledged
    {menu.errors[0]();}
    else if(STATS_COMMS)
    {menu.errors[2]();}
    else if(bmu.status && !(STATS_BMU_ACK))
    {menu.errors[3]();}
    else
    {
      if(STATS_SWOC_ACK && !(esc.error & 0x2)) // if acknowledged previous error is reset
      {CLR_STATS_SWOC_ACK;}
      if(STATS_HWOC_ACK && !(esc.error & 0x1)) // if acknowledged previous error is reset
      {CLR_STATS_HWOC_ACK;}
      if(STATS_BMU_ACK && !(bmu.status))
      {CLR_STATS_BMU_ACK;}

      menu.counter++;

      menu.menus[menu.menu_pos]();
    }
    main_mppt_poll();
    main_input_check();
    stats.errors &= 0b11100111;
    stats.errors |= main_fault_check() << 3;
    if(!(stats.errors & (0x2 << 3))){main_drive();} // no drive on fault code 2
    main_lights();
    main_can_handler();
    main_calc();
    main_HV();
    main_driver_check();
  }

  return 0; // For compilers sanity
}

