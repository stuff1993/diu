#ifdef __USE_CMSIS
#include "LPC17xx.h"
#endif

#include <cr_section_macros.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

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

CAR_CONFIG config = { CAN_ESC,
        CAN_CONTROL,
        CAN_DASH_REPLY,
        CAN_DASH_REQUEST,
        CAN_SHUNT,
        CAN_BMU,
        CAN_MPPT0,
        CAN_MPPT1,
        CAN_MPPT2,
        WHEEL_D,
        MAX_THROTTLE_LOW,
        LOW_SPEED_THRES };

DRIVER_CONFIG drv_config[4] =
    {
        {D0_MAX_THROTTLE, D0_MAX_REGEN, D0_THROTTLE_RAMP, D0_REGEN_RAMP},  // Race
        {D1_MAX_THROTTLE, D1_MAX_REGEN, D1_THROTTLE_RAMP, D1_REGEN_RAMP},  // Hot Lap
        {D2_MAX_THROTTLE, D2_MAX_REGEN, D2_THROTTLE_RAMP, D2_REGEN_RAMP},  // Test
        {D3_MAX_THROTTLE, D3_MAX_REGEN, D3_THROTTLE_RAMP, D3_REGEN_RAMP}   // Display
    };

/*
 * To prevent confusion with previous code
 * new MPPT == mppt[0]
 * MPPT1 == mppt[1]
 * MPPT2 == mppt[2]
 */
MPPT mppt[3] =
    {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

MOTORCONTROLLER esc =
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

SHUNT shunt =
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

BMU bmu =
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

CLOCK clock =
    {0, 0, 0, 0, 0, 0};

// Systick
CAN_MSG can_tx1_buf =
    {0, 0, 0, 0};
// Free loop
CAN_MSG can_tx2_buf =
    {0, 0, 0, 0};
// CAN Interrupts
CAN_MSG can_tx3_buf =
    {0, 0, 0, 0};

LAP_TIMER lap_timer =
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

volatile unsigned char SWITCH_IO = 0;

uint16_t thr_pos = 0;
uint16_t rgn_pos = 0;


/******************************************************************************
 ** Function:    reset_mppt
 **
 ** Description: Reset mppt values
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
__attribute__((always_inline)) inline void
_reset_mppt(MPPT *_mppt) {
    _mppt->v_in = 0;
    _mppt->i_in = 0;
    _mppt->v_out = 0;
    _mppt->watts = 0;
    _mppt->flags = 0;
}

/******************************************************************************
 ** Function:    tick_mppt
 **
 ** Description: Tick connection timer or reset struct values for mppt
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
__attribute__((always_inline)) inline void
_tick_mppt(MPPT *_mppt) {
    if (_mppt->con_tim)
    {
        _mppt->con_tim--;
    }
    else
    {
        _reset_mppt(_mppt);
    }
}

/******************************************************************************
 ** Function:    BOD_IRQHandler
 **
 ** Description: Brown-out detection handler
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
BOD_IRQHandler(void)
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
 ** Description: System clock event handler
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
SysTick_Handler(void)
{
    clock.t_ms++;
    clock.blink = clock.t_ms / (SYSTICK_SEC_COUNT / 2);

    // DIU CAN Heart Beat
    // Every 100 mS send heart beat CAN packets
    const uint8_t second_tenths = clock.t_ms % (SYSTICK_SEC_COUNT / 10);
    if (second_tenths == 2 && STATS_ARMED)
    {
        can_tx1_buf.Frame = 0x00080000;
        can_tx1_buf.MsgID = config.can_control + 1;
        can_tx1_buf.DataA = conv_float_uint(drive.speed_rpm);
        can_tx1_buf.DataB = conv_float_uint(fabsf(drive.current));
        can1_send_message(&can_tx1_buf);

        // Average power stats
        esc.avg_power += esc.watts;
        mppt[0].avg_power += mppt[0].watts;
        mppt[1].avg_power += mppt[1].watts;
        mppt[2].avg_power += mppt[2].watts;
        stats.avg_power_counter++;
    }
    else if (second_tenths == 7)
    {
        // Light control message
        // If hazards, set both left and right
        can_tx1_buf.Frame = 0x00080000;
        can_tx1_buf.MsgID = config.can_dash_reply;
        can_tx1_buf.DataA = STATS_LEFT | (STATS_RIGHT << 1) | (STATS_BRAKE << 2) | (MECH_BRAKE << 3) | STATS_HAZARDS | (STATS_HAZARDS << 1);
        can_tx1_buf.DataB = thr_pos << 16 | rgn_pos;
        can1_send_message(&can_tx1_buf);
    }

    if (stats.buz_tim)
    {
        if (!(--stats.buz_tim))
        {
            BUZZER_OFF
        }
    }

    if (bmu.status & 0x107 || !bmu.con_tim || (STATS_RGN_CAP && bmu.bus_i > 19))
    {
        disengage_contactors();
    }

    // Time sensitive Calculations
    esc.watt_hrs += (esc.watts / SYSTICK_HOUR_DIV);

    const float mppt0_wh = (mppt[0].watts / SYSTICK_HOUR_DIV);
    mppt[0].watt_hrs += mppt0_wh;
    const float mppt1_wh = (mppt[1].watts / SYSTICK_HOUR_DIV);
    mppt[1].watt_hrs += mppt1_wh;
    const float mppt2_wh = (mppt[2].watts / SYSTICK_HOUR_DIV);
    mppt[2].watt_hrs += mppt2_wh;

    const float bmu_wh = (bmu.watts / SYSTICK_HOUR_DIV);
    bmu.watt_hrs += bmu_wh;

    stats.odometer += fabsf(esc.velocity_kmh / SYSTICK_HOUR_DIV);
    stats.odometer_tr += fabsf(esc.velocity_kmh / SYSTICK_HOUR_DIV);

    if (lap_timer.track)
    {
        // Don't use esc consumption as excludes other drains on system (ie the DIU)
        const float mppt_t_wh = mppt0_wh + mppt1_wh + mppt2_wh;
        lap_timer.current_ms += SYSTICK_INT_MS;
        lap_timer.current_power_in += mppt_t_wh;
        lap_timer.current_power_out += bmu_wh - mppt_t_wh;
    }

    // Calculate time
    if (clock.t_ms >= SYSTICK_SEC_COUNT)
    {
        clock.t_ms = 0;
        clock.t_s++;

        _tick_mppt(&(mppt[0]));
        _tick_mppt(&(mppt[1]));
        _tick_mppt(&(mppt[2]));
        if (shunt.con_tim)
        {
            shunt.con_tim--;
        }
        if (esc.con_tim)
        {
            esc.con_tim--;
        }
        if (bmu.con_tim)
        {
        	bmu.con_tim--;
        }

        // CAN transceiver seems to struggle to send these and the drive packets above
        // so only send one at a time.
        can_tx1_buf.Frame = 0x00080000;
        const uint8_t third_second = clock.t_s % 3;
        if (!third_second) {
            can_tx1_buf.MsgID = config.can_dash_reply + 3;
            if (stats.avg_power_counter)
            {
                can_tx1_buf.DataA = conv_float_uint(esc.avg_power / stats.avg_power_counter);
            }
            else
            {
                can_tx1_buf.DataA = 0;
            }
            can_tx1_buf.DataB = conv_float_uint(mppt[0].watt_hrs + mppt[1].watt_hrs + mppt[2].watt_hrs);
            can1_send_message(&can_tx1_buf);
        }
        else if (third_second == 1)
        {
            if (stats.avg_power_counter)
            {
                can_tx1_buf.MsgID = config.can_dash_reply + 4;
                can_tx1_buf.DataA = conv_float_uint(mppt[1].avg_power / stats.avg_power_counter);
                can_tx1_buf.DataB = conv_float_uint(mppt[2].avg_power / stats.avg_power_counter);
                can1_send_message(&can_tx1_buf);
            }
        }
        else
        {
            if (mppt[0].con_tim && stats.avg_power_counter)
            {
                can_tx1_buf.MsgID = config.can_dash_reply + 5;
                can_tx1_buf.DataA = conv_float_uint(mppt[0].avg_power / stats.avg_power_counter);
                can_tx1_buf.DataB = 0;
                can1_send_message(&can_tx1_buf);
            }
        }

        // Store data in eeprom every second
        persistent_store();

        can_tx1_buf.Frame = 0x00000000;
        can_tx1_buf.MsgID = config.can_mppt0;
        can2_send_message(&can_tx1_buf);

        can_tx1_buf.Frame = 0x00000000;
        can_tx1_buf.MsgID = config.can_mppt1;
        can2_send_message(&can_tx1_buf);

        can_tx1_buf.Frame = 0x00000000;
        can_tx1_buf.MsgID = config.can_mppt2;
        can2_send_message(&can_tx1_buf);

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
void
main_driver_check(void)
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
 ** Function:    can1_unpack
 **
 ** Description: Unpacks data received on CAN1
 **
 ** Parameters:  Received CAN message
 ** Return:      None
 **
 ******************************************************************************/
void
can1_unpack(CAN_MSG *_msg)
{
    if (_msg->MsgID >= config.can_esc && _msg->MsgID <= config.can_esc + 23)
    {
        esc_data_extract(&esc, _msg);
    }
    else if (_msg->MsgID >= config.can_dash_request && _msg->MsgID <= config.can_dash_request + 1)
    {
        dash_data_extract(_msg);
    }
    else if (_msg->MsgID >= config.can_shunt && _msg->MsgID <= config.can_shunt + 1)
    {
        shunt_data_extract(&shunt, _msg);
    }
    else if (_msg->MsgID >= (config.can_bmu + BMU_INFO + 4) && _msg->MsgID <= (config.can_bmu + BMU_INFO + 9))
    {
        bmu_data_extract(&bmu, _msg);
    }
}

/******************************************************************************
 ** Function:    can2_unpack
 **
 ** Description: Unpacks data received on CAN2
 **
 ** Parameters:  Received CAN message
 ** Return:      None
 **
 ******************************************************************************/
void
can2_unpack(CAN_MSG *_msg)
{
    if (_msg->MsgID == config.can_mppt0 + MPPT_RPLY)
    {
        mppt_data_transfer(_msg);
        mppt_data_extract(&(mppt[0]), _msg);
    }
    else if (_msg->MsgID == config.can_mppt1 + MPPT_RPLY)
    {
        mppt_data_transfer(_msg);
        mppt_data_extract(&(mppt[1]), _msg);
    }
    else if (_msg->MsgID == config.can_mppt2 + MPPT_RPLY)
    {
        mppt_data_transfer(_msg);
        mppt_data_extract(&(mppt[2]), _msg);
    }
}

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
__attribute__((always_inline)) inline void
mppt_data_extract(MPPT *_mppt, CAN_MSG *_msg)
{
    uint32_t _data_a = _msg->DataA;
    uint32_t _data_b = _msg->DataB;

    uint32_t _v_in = ((_data_a & 0x3) << 8);     // Masking and shifting the upper 2 MSB
    _v_in |= ((_data_a & 0xFF00) >> 8);          // Masking and shifting the lower 8 LSB
    _v_in *= 1.50;                               // Scaling

    uint32_t _i_in = ((_data_a & 0x30000) >> 8); // Masking and shifting the lower 8 LSB
    _i_in |= ((_data_a & 0xFF000000) >> 24);     // Masking and shifting the upper 2 MSB
    _i_in *= 0.87;                               // Scaling

    uint32_t _v_out = ((_data_b & 0x3) << 8);    // Masking and shifting the upper 2 MSB
    _v_out |= ((_data_b & 0xFF00) >> 8);         // Masking and shifting the lower 8 LSB
    _v_out *= 2.10;                              // Scaling

    _mppt->flags = (_data_a & 0xF0) >> 4;
    _mppt->tmp = ((_data_b & 0xFF0000) >> 16);
    _mppt->v_in = _v_in;
    _mppt->i_in = _i_in;
    _mppt->v_out = _v_out;
    _mppt->con_tim = 3;
    _mppt->watts = (_mppt->v_in * _mppt->i_in) / 1000.0;

    if (_mppt->tmp > _mppt->max_tmp)
    {
        _mppt->max_tmp = _mppt->tmp;
    }
    if (_mppt->v_in > _mppt->max_v_in)
    {
        _mppt->max_v_in = _mppt->v_in;
    }
    if (_mppt->i_in > _mppt->max_i_in)
    {
        _mppt->max_i_in = _mppt->i_in;
    }
    if (_mppt->watts > _mppt->max_watts)
    {
        _mppt->max_watts = _mppt->watts;
    }
}

/******************************************************************************
 ** Function:    mppt_data_transfer
 **
 ** Description: Extracts data from CAN message into MPPT structure.
 ** 			 Uses DriveTek message structure.
 **
 ** Parameters:  1. Address of MPPT to extract to
 ** 			 2. CAN message to extract from
 ** Return:      None
 **
 ******************************************************************************/
__attribute__((always_inline)) inline void
mppt_data_transfer(CAN_MSG *_msg)
{
    can_tx3_buf = *_msg;
    can1_send_message(&can_tx3_buf);
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
__attribute__((always_inline)) inline void
esc_data_extract(MOTORCONTROLLER *_esc, CAN_MSG *_msg)
{
    uint16_t id_offset = _msg->MsgID - config.can_esc;

    switch (id_offset)
    {
    case 1:
        _esc->con_tim = 3;
        _esc->error = (_msg->DataA >> 16);
        if (_esc->error == 0x2)
        {
            NEUTRAL_ON
            REVERSE_ON
            DRIVE_ON
            REGEN_ON
        }
        break;
    case 2:
        _esc->bus_v = iir_filter_float(_esc->bus_v, conv_uint_float(_msg->DataA), IIR_GAIN_ELECTRICAL);
        _esc->bus_i = iir_filter_float(_esc->bus_i, conv_uint_float(_msg->DataB), IIR_GAIN_ELECTRICAL);

        _esc->watts = _esc->bus_v * _esc->bus_i;
        if (_esc->watts > _esc->max_watts)
        {
            _esc->max_watts = _esc->watts;
        }
        if (_esc->bus_i > _esc->max_bus_i)
        {
            _esc->max_bus_i = _esc->bus_i;
        }
        if (_esc->bus_v > _esc->max_bus_v)
        {
            _esc->max_bus_v = _esc->bus_v;
        }
        break;
    case 3:
        _esc->velocity_kmh = conv_uint_float(_msg->DataB) * 3.6;

        if (esc.velocity_kmh > stats.max_speed)
        {
            stats.max_speed = _esc->velocity_kmh;
        }
        break;
    case 11:
        _esc->heatsink_tmp = conv_uint_float(_msg->DataB);
        _esc->motor_tmp = conv_uint_float(_msg->DataA);
        break;
    case 12:
        _esc->board_tmp = conv_uint_float(_msg->DataA);
        break;
    case 14:
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
__attribute__((always_inline)) inline void
dash_data_extract(CAN_MSG *_msg)
{
    uint16_t id_offset = _msg->MsgID - config.can_dash_request;

    switch (id_offset)
    {
    case 0:
        // Data = KILLDRVE
        if (_msg->DataA == 0x4C4C494B && _msg->DataB == 0x45565244)
        {
            SET_STATS_STOP
        }
        break;
    case 1:
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
__attribute__((always_inline)) inline void
shunt_data_extract(SHUNT *_shunt, CAN_MSG *_msg)
{
    uint16_t id_offset = _msg->MsgID - config.can_shunt;

    switch (id_offset)
    {
    case 0:
        _shunt->bus_v = conv_uint_float(_msg->DataA); // Values filtered on shunt side
        _shunt->bus_i = conv_uint_float(_msg->DataB);
        _shunt->bus_watts = _shunt->bus_i * _shunt->bus_v;
        _shunt->con_tim = 3;

        if (_shunt->bus_watts > _shunt->max_bus_watts)
        {
            _shunt->max_bus_watts = _shunt->bus_watts;
        }

        if (_shunt->bus_i > _shunt->max_bat_i) {
            _shunt->max_bat_i = _shunt->bus_i;
        }
        if (_shunt->bus_v > _shunt->max_bat_v) {
            _shunt->max_bat_v = _shunt->bus_v;
        }
        break;
    case 1:
        _shunt->watt_hrs = conv_uint_float(_msg->DataA);
        float _data_b = conv_uint_float(_msg->DataB);

        /*
         * See MegaBoard.c for packing of this packet.
         * Negative value here indicates HV not armed,
         * positive indicates armed.
         * 100.0 is arbitrarily added by the MegaBoard
         * to the MPPT current to guarantee that the polarity
         * of the float is correct, so must be removed here.
         */
        if (_data_b < 0)
        {
            CLR_STATS_ARMED
            _data_b *= -1.0;
        }
        else
        {
            SET_STATS_ARMED
        }
        _shunt->mppt_i = _data_b - 100.0;
        _shunt->mppt_watts = _shunt->mppt_i * _shunt->bus_v;
        if (_shunt->mppt_i > _shunt->max_mppt_i)
        {
            _shunt->max_mppt_i = _shunt->mppt_i;
        }
        if (_shunt->mppt_watts > _shunt->max_mppt_watts)
        {
            _shunt->max_mppt_watts = _shunt->mppt_watts;
        }
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
__attribute__((always_inline)) inline void
bmu_data_extract(BMU *_bmu, CAN_MSG *_msg)
{
	_bmu->con_tim = 3;
    uint16_t id_offset = _msg->MsgID - config.can_bmu;

    switch (id_offset)
    {
    case BMU_INFO + 4:
        _bmu->min_cell_v = _msg->DataA & 0xFFFF;
        _bmu->max_cell_v = (_msg->DataA >> 16) & 0xFFFF;
        _bmu->cmu_min_v = _msg->DataB & 0xFF;
        _bmu->cmu_max_v = (_msg->DataB >> 16) & 0xFF;
        _bmu->cell_min_v = (_msg->DataB >> 8) & 0xFF;
        _bmu->cell_max_v = (_msg->DataB >> 24) & 0xFF;
        break;
    case BMU_INFO + 5:
        _bmu->max_cell_tmp = _msg->DataA & 0xFFFF;
        _bmu->max_cell_tmp = (_msg->DataA >> 16) & 0xFFFF;
        _bmu->cmu_min_tmp = _msg->DataB & 0xFF;
        _bmu->cmu_max_tmp = (_msg->DataB >> 16) & 0xFF;
        break;
    case BMU_INFO + 6:
        // Packet is in mV and mA
        _bmu->bus_v = iir_filter_uint(_msg->DataA / 1000, _bmu->bus_v, IIR_GAIN_ELECTRICAL);
        _bmu->bus_i = iir_filter_int(_msg->DataB / 1000, _bmu->bus_i, IIR_GAIN_ELECTRICAL);

        _bmu->watts = _bmu->bus_i * _bmu->bus_v;
        if (_bmu->watts > _bmu->max_watts)
        {
            _bmu->max_watts = _bmu->watts;
        }
        if (_bmu->bus_i > _bmu->max_bus_i)
        {
            _bmu->max_bus_i = _bmu->bus_i;
        }
        if (_bmu->bus_v > _bmu->max_bus_v)
        {
            _bmu->max_bus_v = _bmu->bus_v;
        }
        break;
    case BMU_INFO + 9:
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
void
main_input_check(void)
{
    unsigned char OLD_IO = SWITCH_IO;
    uint8_t btn_ret = 0;

    SWITCH_IO = 0;
    SWITCH_IO |= (FORWARD << 0);
    SWITCH_IO |= (REVERSE << 1);
    SWITCH_IO |= (LEFT_ON << 2);
    SWITCH_IO |= (RIGHT_ON << 3);

    // BEEP if toggle position has changed.
    if (OLD_IO != SWITCH_IO)
    {
        buzzer(50);
    }

    if (SWITCH_IO & 0x4)
    {
        SET_STATS_LEFT
    }
    else
    {
        CLR_STATS_LEFT
    }
    if (SWITCH_IO & 0x8)
    {
        SET_STATS_RIGHT
    }
    else
    {
        CLR_STATS_RIGHT
    }


    if ((btn_ret = btn_release_left_right()))
    {
        buzzer(2);

        if (btn_ret == 3)      {menu.menu_pos = 0;}
        else if (btn_ret == 1) {menu_dec(&menu.menu_pos, menu.menu_items);}
        else if (btn_ret == 2) {menu_inc(&menu.menu_pos, menu.menu_items);}

        if (menu.menu_pos==0) {buzzer(10);}
        if ((esc.error & 0x2) && !STATS_SWOC_ACK) {SET_STATS_SWOC_ACK;}
        if ((esc.error & 0x1) && !STATS_HWOC_ACK) {SET_STATS_HWOC_ACK;}
        if (STATS_COMMS == 1)  // send NO RESPONSE packet
        {
            if((LPC_CAN1->GSR & (1 << 3)))  // Check Global Status Register
            {
                can_tx2_buf.Frame = 0x00010000; // 11-bit, no RTR, DLC is 1 byte
                can_tx2_buf.MsgID = config.can_dash_reply + 1;
                can_tx2_buf.DataA = 0x0;
                can_tx2_buf.DataB = 0x0;
                can1_send_message(&can_tx2_buf);
            }
            CLR_STATS_COMMS
        }

        lcd_clear();
        inputs.input_dwn = 0;
        menu.submenu_pos = 0;
        CLR_MENU_SELECTED
    }

    if (swt_cruise() & 0x0C)
    {
        TOG_STATS_HAZARDS
    }

    if ((MECH_BRAKE || rgn_pos/* || (esc.con_tim && esc.bus_i < 0)*/)) // Disabled brake lights while in cruise
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
int
main_fault_check(void)
{
    // Buzz for under voltage and over temp flags
    if (bmu.status & 0x6)
    {
        force_buzzer(100);
    }

    if (!(mppt[1].i_in && mppt[2].i_in))
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
    if ((mppt[0].flags & 0xF) || (mppt[1].flags & 0xF) || (mppt[2].flags & 0xF)
            || (bmu.status & 0x117) || (!shunt.con_tim))
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
void
main_drive(void)
{
    uint32_t ADC_A;
    uint32_t ADC_B;

    ADC_A = (ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0) + ADCRead(0))/8;

    ADC_B = (ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1) + ADCRead(1))/8;

    main_paddles(ADC_A, ADC_B, &thr_pos, &rgn_pos);

    if((!FORWARD || menu.driver == 4 || STATS_DRV_MODE) && STATS_CR_STS) // Must be in forward or not in display mode to use cruise
    {
        buzzer(10);
        CLR_STATS_CR_ACT
        CLR_STATS_CR_STS
        stats.cruise_speed = 0;
    }
    if(rgn_pos || thr_pos)
    {
        CLR_STATS_CR_ACT
    }


    // DRIVE LOGIC
    if ((!MECH_BRAKE || STATS_DRV_MODE) && (FORWARD || REVERSE)) {
        if(STATS_CR_ACT)                                                                                        {drive.current = 1.0;    drive.speed_rpm = stats.cruise_speed / ((60 * 3.14 * config.wheel_d) / 1000.0);}
        else if(!thr_pos && !rgn_pos)                                                                           {drive.speed_rpm = 0;    drive.current = 0;}
        else if(rgn_pos)                                                                                        {drive.speed_rpm = 0;    drive.current = -((float)rgn_pos / 1000.0);}
        else if(thr_pos && drive.current < 0)                                                                   {                        drive.current = 0;}
        else if(FORWARD && esc.velocity_kmh > -5.0 && (((drive.current * 1000) + drv_config[menu.driver].throttle_ramp_rate) < thr_pos))  {drive.speed_rpm = 1500; drive.current += (drv_config[menu.driver].throttle_ramp_rate / 1000.0);}
        else if(FORWARD && esc.velocity_kmh > -5.0)                                                             {drive.speed_rpm = 1500; drive.current = (thr_pos / 1000.0);}
        else if(REVERSE && esc.velocity_kmh <  1.0 && (((drive.current * 1000) + drv_config[menu.driver].throttle_ramp_rate) < thr_pos))  {drive.speed_rpm = -200; drive.current += (drv_config[menu.driver].throttle_ramp_rate / 1000.0);}
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
void
main_paddles(uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn)
{
    if((_pad1 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)) || (_pad1 < ((LOW_PAD_V - 0.1) * ADC_POINTS_PER_V))){_pad1 = MID_PAD_V * ADC_POINTS_PER_V;}
    if((_pad2 > ((HGH_PAD_V + 0.1) * ADC_POINTS_PER_V)) || (_pad2 < ((LOW_PAD_V - 0.1) * ADC_POINTS_PER_V))){_pad2 = MID_PAD_V * ADC_POINTS_PER_V;}
    switch(stats.paddle_mode)
    {
    case 0:
        // Throttle - Paddle 1
        _pad1 = (_pad1 < ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad1 - ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
        _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
        if(esc.velocity_kmh < config.low_spd_threshold && _pad1 > config.max_thr_lowspd){_pad1 = config.max_thr_lowspd;}
        if(_pad1 > drv_config[menu.driver].max_throttle){_pad1 = drv_config[menu.driver].max_throttle;}
        if(_pad1>1000){_pad1=1000;}
        // Regen - Paddle 2
        _pad2 = (_pad2 < ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V)) ? 0 : _pad2 - ((MID_PAD_V + MIN_RGN_DZ) * ADC_POINTS_PER_V);
        _pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_RGN_DZ) - (MID_PAD_V + MIN_RGN_DZ)) * ADC_POINTS_PER_V);
        if(_pad2>1000){_pad2=1000;}
        _pad2 *= drv_config[menu.driver].max_regen / 1000.0;

        *_thr = _pad1;
        *_rgn = _pad2;
        break;
    default:
    case 1:
        if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
        {
            _pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
            _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);
            if(esc.velocity_kmh < config.low_spd_threshold && _pad1 > config.max_thr_lowspd){_pad1 = config.max_thr_lowspd;}
            if(_pad1 > drv_config[menu.driver].max_throttle){_pad1 = drv_config[menu.driver].max_throttle;}
            if(_pad1>1000){_pad1=1000;}

            *_thr = _pad1;
            *_rgn = 0;
        }
        else if(_pad1 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 1 Back
        {
            _pad1 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad1;
            _pad1 = (_pad1 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
            if(_pad1>1000){_pad1=1000;}
            _pad1 *= drv_config[menu.driver].max_regen / 1000.0;

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
            _pad1 *= drv_config[menu.driver].max_regen / 1000.0;

            *_thr = 0;
            *_rgn = _pad1;
        }
        else if(_pad2 < ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V)) // Regen - Paddle 2 Back
        {
            _pad2 = ((MID_PAD_V - MIN_RGN_DZ) * ADC_POINTS_PER_V) - _pad2;
            _pad2 = (_pad2 * 1000) / (((MID_PAD_V - MIN_RGN_DZ) - (LOW_PAD_V + MAX_THR_DZ)) * ADC_POINTS_PER_V);
            if(_pad2>1000){_pad2=1000;}
            _pad2 *= drv_config[menu.driver].max_regen / 1000.0;

            *_thr = 0;
            *_rgn = _pad2;
        }
        else if(_pad1 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 1 Forward
        {
            _pad1 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
            _pad1 = (_pad1 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

            if(esc.velocity_kmh < config.low_spd_threshold && _pad1 > config.max_thr_lowspd){_pad1 = config.max_thr_lowspd;}
            if(_pad1 > drv_config[menu.driver].max_throttle){_pad1 = drv_config[menu.driver].max_throttle;}
            if(_pad1>1000){_pad1=1000;}

            *_thr = _pad1;
            *_rgn = 0;
        }
        else if(_pad2 > ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V)) // Throttle - Paddle 2 Forward
        {
            _pad2 -= ((MID_PAD_V + MIN_THR_DZ) * ADC_POINTS_PER_V);
            _pad2 = (_pad2 * 1000) / (((HGH_PAD_V - MAX_THR_DZ) - (MID_PAD_V + MIN_THR_DZ)) * ADC_POINTS_PER_V);

            if(esc.velocity_kmh < config.low_spd_threshold && _pad2 > config.max_thr_lowspd){_pad2 = config.max_thr_lowspd;}
            if(_pad2 > drv_config[menu.driver].max_throttle){_pad2 = drv_config[menu.driver].max_throttle;}
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
void
main_lights(void)
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
void
main_can_handler(void)
{
    if(STATS_STOP)
    {
        CLR_STATS_STOP
        lcd_clear();

        drive.current = 0;
        drive.speed_rpm = 0;

        if(menu.driver == 2)
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
 ** Function:    esc_reset
 **
 ** Description: Resets motorcontroller(s) with CAN packet
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
esc_reset(void)
{
    // RESET MOTOR CONTROLLER(S)
    // see WS22 user manual and Tritium CAN network specs
    // TODO: try MC + 25 (0x19) + msg "RESETWS" (TRI88.004 ver3 doc, July 2013) - 2015
    can_tx2_buf.Frame = 0x00080000;  // 11-bit, no RTR, DLC is 1 byte
    can_tx2_buf.MsgID = config.can_control + 3;
    can_tx2_buf.DataA = 0x0;
    can_tx2_buf.DataB = 0x0;
    can1_send_message(&can_tx2_buf);
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
void
persistent_load(void)
{
    stats.flags |= (ee_read(ADD_BUZZ) & 0x2002);
    stats.odometer = conv_uint_float(ee_read(ADD_ODO));
    stats.odometer_tr = conv_uint_float(ee_read(ADD_ODOTR));

    bmu.watt_hrs = conv_uint_float(ee_read(ADD_BMUWHR));
    mppt[0].watt_hrs = conv_uint_float(ee_read(ADD_MPPT0WHR));
    mppt[1].watt_hrs = conv_uint_float(ee_read(ADD_MPPT1WHR));
    mppt[2].watt_hrs = conv_uint_float(ee_read(ADD_MPPT2WHR));

    uint32_t *conf_add = (uint32_t *)(&config);
    *conf_add++ = ee_read(ADD_CONF1);
    *conf_add++ = ee_read(ADD_CONF2);
    *conf_add++ = ee_read(ADD_CONF3);
    *conf_add++ = ee_read(ADD_CONF4);
    *conf_add++ = ee_read(ADD_CONF5);
    *conf_add = ee_read(ADD_CONF6);

    conf_add = (uint32_t *)(&drv_config);
    *conf_add++ = ee_read(ADD_DRV0_CONF1);
    *conf_add++ = ee_read(ADD_DRV0_CONF2);
    *conf_add++ = ee_read(ADD_DRV1_CONF1);
    *conf_add++ = ee_read(ADD_DRV1_CONF2);
    *conf_add++ = ee_read(ADD_DRV2_CONF1);
    *conf_add++ = ee_read(ADD_DRV2_CONF2);
    *conf_add++ = ee_read(ADD_DRV3_CONF1);
    *conf_add = ee_read(ADD_DRV3_CONF2);

    if (isnan(stats.odometer))
    {
        stats.odometer = 0.0;
    }
    if (isnan(stats.odometer_tr))
    {
        stats.odometer_tr = 0.0;
    }
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
void
persistent_store(void)
{
    uint16_t eestatus = 0;
    if (isnan(stats.odometer))
    {
        stats.odometer = 0.0;
    }
    if (isnan(stats.odometer_tr))
    {
        stats.odometer_tr = 0.0;
    }

    if (STATS_CONF_CHANGED)
    {
        CLR_STATS_CONF_CHANGED
        uint32_t *conf_add = (uint32_t *)(&(config));
        eestatus |= ee_write(ADD_CONF1, *conf_add++);
        eestatus |= ee_write(ADD_CONF2, *conf_add++);
        eestatus |= ee_write(ADD_CONF3, *conf_add++);
        eestatus |= ee_write(ADD_CONF4, *conf_add++);
        eestatus |= ee_write(ADD_CONF5, *conf_add++);
        eestatus |= ee_write(ADD_CONF6, *conf_add);

        conf_add = (uint32_t *)(&(drv_config));
        eestatus |= ee_write(ADD_DRV0_CONF1, *conf_add++);
        eestatus |= ee_write(ADD_DRV0_CONF2, *conf_add++);
        eestatus |= ee_write(ADD_DRV1_CONF1, *conf_add++);
        eestatus |= ee_write(ADD_DRV1_CONF2, *conf_add++);
        eestatus |= ee_write(ADD_DRV2_CONF1, *conf_add++);
        eestatus |= ee_write(ADD_DRV2_CONF2, *conf_add++);
        eestatus |= ee_write(ADD_DRV3_CONF1, *conf_add++);
        eestatus |= ee_write(ADD_DRV3_CONF2, *conf_add++);

        return;
    }

    if (clock.t_s % 2)
    {
        eestatus |= ee_write(ADD_ODO, conv_float_uint(stats.odometer));
        eestatus |= ee_write(ADD_ODOTR, conv_float_uint(stats.odometer_tr));
        eestatus |= ee_write(ADD_MPPT0WHR, conv_float_uint(mppt[0].watt_hrs));
    }
    else
    {
        eestatus |= ee_write(ADD_BMUWHR, conv_float_uint(bmu.watt_hrs));
        eestatus |= ee_write(ADD_MPPT1WHR, conv_float_uint(mppt[1].watt_hrs));
        eestatus |= ee_write(ADD_MPPT2WHR, conv_float_uint(mppt[2].watt_hrs));
    }
    if (eestatus & 0x800)
    {
        SET_STATS_EEPROM_TO
    }
    else
    {
        CLR_STATS_EEPROM_TO
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
void
nonpersistent_load(void)
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
void
gpio_init(void)
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
     *    19 - OUT - C_2_3
     *    20 - OUT - C_1
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
    LPC_GPIO1->FIODIR = (1 << 8) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 23) | (1 << 24) | (1 << 25) | (1 << 26) | (1 << 30) | (1 << 31);
    LPC_GPIO1->FIOCLR = (1 << 8) | (1 << 19) | (1 << 20) | (1 << 21) | (1 << 23) | (1 << 24) | (1 << 25) | (1 << 26) | (1 << 30) | (1 << 31);

    /*
     * GPIO2:
     *  PINSEL4:
     *    0 - OUT - FAULT HELPER
     *    6 - OUT - LCD D7
     *    7 - OUT - LCD D6
     *    8 - OUT - LCD D5
     *    9 - OUT - LCD D4
     *    10 - IN - Aux ON (SPORTS MODE)
     *    11 - IN - Aux OFF
     *    12 - IN - Hazards
     *    13 - IN - Hazards
     */
    LPC_GPIO2->FIODIR = (1 << 0) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);
    LPC_GPIO2->FIOCLR = (1 << 0) | (1 << 6) | (1 << 7) | (1 << 8) | (1 << 9);

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
 **
 ** Function:    led_test
 **
 ** Description: Activates all LEDs in sequence
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
led_test(void)
{
    const uint16_t wait_time = 500;
    REVERSE_ON
    delayMs(1, wait_time);
    REVERSE_OFF
    NEUTRAL_ON
    delayMs(1, wait_time);
    NEUTRAL_OFF
    REGEN_ON
    delayMs(1, wait_time);
    REGEN_OFF
    SPORTS_ON
    delayMs(1, wait_time);
    SPORTS_OFF
    ECO_ON
    delayMs(1, wait_time);
    ECO_OFF
    BLINKER_L_ON
    delayMs(1, wait_time);
    BLINKER_L_OFF
    FAULT_ON
    delayMs(1, wait_time);
    FAULT_OFF
    HV_ON
    delayMs(1, wait_time);
    HV_OFF
    BLINKER_R_ON
    delayMs(1, wait_time);
    BLINKER_R_OFF
}

/******************************************************************************
 **
 ** Function:    engage_contactors
 **
 ** Description: Engage contactors on car start
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
engage_contactors(void)
{
    C_1_ON
    SET_STATS_C_1
    C_2_3_ON
    SET_STATS_C_2_3
}

/******************************************************************************
 **
 ** Function:    disengage_contactors
 **
 ** Description: Disengage contactors on bmu OverV, UnderV or OverT
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
disengage_contactors(void)
{
    C_1_OFF
    CLR_STATS_C_1
    C_2_3_OFF
    CLR_STATS_C_2_3
}

/******************************************************************************
 **
 ** Function:    motorcontroller_init
 **
 ** Description: Waits for heartbeat from motorcontroller before sending 0x502 packet
 **
 ** Parameters:  None
 ** Return:      None
 **
 ******************************************************************************/
void
motorcontroller_init(void)
{
    menu_esc_wait();
    while (esc.con_tim == 0 && !btn_release_select())
    {

    }
    can_tx2_buf.Frame = 0x00080000;
    can_tx2_buf.MsgID = config.can_control + 2;
    can_tx2_buf.DataA = 0x0;
    can_tx2_buf.DataB = conv_float_uint(1);
    force_buzzer(20);
    while (!can1_send_message(&can_tx2_buf))
    {

    }
    delayMs(1, 1000);
}

/******************************************************************************
 ** Function:    buzzer
 **
 ** Description: Turns buzzer on for set amount of time if buzzer on in options
 **
 ** Parameters:  Number of 10mS ticks to sound buzzer
 ** Return:      None
 **
 ******************************************************************************/
void
buzzer(uint8_t val)
{
    if (STATS_BUZZER)
    {
        force_buzzer(val);
    }
}

/******************************************************************************
 ** Function:    force_buzzer
 **
 ** Description: Turns buzzer on for set amount of time regardless of options
 **
 ** Parameters:  Number of 10mS ticks to sound buzzer
 ** Return:      None
 **
 ******************************************************************************/
void
force_buzzer(uint8_t val)
{
    stats.buz_tim = val;
    BUZZER_ON
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
void
BOD_init(void)
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
__attribute__((noreturn)) int
main(void)
{
    SystemInit();
    SystemCoreClockUpdate();

    can1_init(BITRATE500K30MHZ);
    can2_init(BITRATE125K30MHZ);
    CAN_SetACCF(ACCF_BYPASS);

    ee_init();

    nonpersistent_load();
    persistent_load();

    ADCInit(ADC_CLK);

    gpio_init();

    engage_contactors();

    lcd_init();
    lcd_clear();

    led_test();

    motorcontroller_init();

    SysTick_Config(SystemCoreClock / SYSTICK_SEC_COUNT);

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
        main_input_check();
        CLR_STATS_FAULT;
        stats.errors |= main_fault_check() << 3;
        if (!(stats.errors & (0x2 << 3)))
        {
            // no drive on fault code 2
            main_drive();
        }
        main_lights();
        main_can_handler();
    }
}
