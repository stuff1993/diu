/*
 * struct.h
 *
 *  Created on: 29 May 2015
 *      Author: Stuart G
 */

#ifndef STRUCT_H_
#define STRUCT_H_

/// MPPT
#define _MPPT_POWER 1
#define _MPPT_PEAKS 1

/// MTR CONTROLLER
#define _MC_ERR 1
#define _MC_LIM 0
#define _MC_PHASE 0
#define _MC_VECTORS 0
#define _MC_RAILS 0
#define _MC_TMP 1
#define _MC_AMPHRS 0
#define _MC_ODO 1
#define _MC_SLIP 0
#define _MC_VELOCITY 1    // 1 = KMH, 2 = RPM, 3 = BOTH
#define _MC_POWER 1
#define _MC_PEAKS 1

/// BMU
#define _BMU_SOC 0        // 1 = SOC, 2 = SOC%, 3 = BOTH
#define _BMU_BAL_SOC 0    // 1 = Balance AmpHrs, 2 = Balance as %, 3 = BOTH
#define _BMU_THRES 0
#define _BMU_CAP 0
#define _BMU_PRECHARGE 0
#define _BMU_CELL_V 1
#define _BMU_CMU_TMP 1
#define _BMU_BAL_THRES 0
#define _BMU_CMU_CNT 0
#define _BMU_VER 0
#define _BMU_FAN 0        // 1 = Fan0, 2 = Fan1, 3 = BOTH
#define _BMU_12V_CONSUM 0
#define _BMU_POWER 1
#define _BMU_PEAKS 1

typedef struct MPPT_STRUCT
{
	/// From CAN Bus
	uint32_t v_in;        // Input Voltage
	uint32_t v_out;       // Output Voltage
	uint32_t i_in;        // Input Current
	uint32_t tmp;         // Temperature in degrees
	uint8_t flags;
	uint32_t avg_power;	// Average Power

#if _MPPT_POWER
	uint32_t watts;       // Watts into MPPT
	float watt_hrs;       // Watt Hours
#if _MPPT_PEAKS
	uint32_t max_watts;   // Peak watts into MPPT
#endif // _MPPT_PEAKS
#endif // _MPPT_POWER
#if _MPPT_PEAKS
	uint32_t max_v_in;     // Peak Input Voltage
	uint32_t max_v_out;    // Peak Output Voltage
	uint32_t max_i_in;     // Peak Input Current
	uint32_t max_tmp;     // Peak Temperature in degrees
#endif // _MPPT_PEAKS
} MPPT;

#if _MC_VECTORS
typedef struct VECTORS_MTRCONT_STRUCT
{
	float v_real;
	float v_imag;
	float i_real;
	float i_imag;
	float bemf_real;
	float bemf_imag;
}VECTORS_MTRCONT;
#endif // _MC_VECTORS

typedef struct MOTORCONTROLLER_STRUCT
{
	/// From CAN Bus
	float bus_i;          // Bus Current
	float bus_v;          // Bus Voltage
	float avg_power;		// Average Power
#if _MC_ERR
	uint16_t error;       // Error Flags
#endif // _MC_ERR
#if _MC_LIM
	uint16_t limit;       // Limit Flags
#endif // _MC_LIM
#if _MC_PHASE
	float phase_c_i;       // Motor Phase C Current
	float phase_b_i;// Motor Phase B Current
#endif // _MC_PHASE
#if _MC_VECTORS
	VECTORS_MTRCONT *vectors; // Motor Vectors
#endif // _MC_VECTORS
#if _MC_RAILS
	float rail_15v;       // 15V Rail Actual Voltage
	float rail_3300mv;// 3.3V Rail Actual Voltage
	float rail_1900mv;// 1.9V Rail Actual Voltage
#endif // _MC_RAILS
#if _MC_TMP
	float heatsink_tmp;   // Heatsink Temperature
	float motor_tmp;      // Motor Temperature
	float board_tmp;      // Board Temperature
#if _MC_PEAKS // && _MC_TMP
	float max_heathsink_tmp;
	float max_motor_tmp;
	float max_board_tmp;
#endif // _MC_PEAKS
#endif // _MC_TMP
#if _MC_AMPHRS
	float dc_amp_hrs;      // DC Bus AmpHrs
#endif // _MC_AMPHRS
#if _MC_ODO
	float odometer;       // Distance traveled since reset (m)
#endif // _MC_ODO
#if _MC_SLIP
	float slip_speed;     // Motor Slip Speed (Hz)
#endif // _MC_SLIP
#if _MC_VELOCITY == 1
	float velocity_kmh;
#elif _MC_VELOCITY == 2
	float velocity_rpm;
#elif _MC_VELOCITY == 3
	float velocity_kmh;
	float velocity_rpm;
#endif // _MC_VELOCITY

	/// Calculated Values & Peaks
#if _MC_POWER
	float watts;          // Bus_I * Bus_V
	float watt_hrs;        // Calculated every 10mS
#if _MC_PEAKS // && _MC_POWER
	float max_watts;
#endif // _MC_PEAKS
#endif // _MC_POWER
#if _MC_PEAKS
	float max_bus_i;
	float max_bus_v;
#endif // MC_PEAKS
} MOTORCONTROLLER;

typedef struct BMU_STRUCT
{
	/// From CAN Bus
	uint32_t bus_v;   // Battery Voltage
	int32_t bus_i;    // Battery Output Current
	uint32_t status;      // status Flags
#if _BMU_SOC == 1
	float soc;            // Battery State of Charge (0 = Full)
#elif _BMU_SOC == 2
	float soc_per;        // Battery State of Charge as % (1 = Full)
#elif _BMU_SOC ==3
	float soc;
	float soc_per;
#endif // _BMU_SOC
#if _BMU_BAL_SOC == 1
	float bal_soc; // Balance State of Charge. Ah supplied to pack since first cell began balancing
#elif _BMU_BAL_SOC == 2
	float bal_soc_per;    // Balance State of Charge as %
#elif _BMU_BAL_SOC == 3
	float bal_soc;
	float bal_soc_per;
#endif // _BMU_BAL_SOC
#if _BMU_THRES
	int16_t charge_cell_v_err;
	int16_t cell_tmp_margin;
	int16_t discharge_cell_v_err;
#endif // _BMU_THRES
#if _BMU_CAP
	uint16_t pack_capacity;
#endif // _BMU_CAP
#if _BMU_PRECHARGE
	char driver_status;
	char precharge_state;
	uint8_t precharge_time_elapsed;
	uint8_t precharge_timer;
#endif // _BMU_PRECHARGE
#if _BMU_CELL_V
	uint16_t min_cell_v;    // Minimum Cell Voltage
	uint16_t max_cell_v;    // Maximum Cell Voltage
	uint8_t cmu_min_v;      // CMU number with minimum cell voltage
	uint8_t cmu_max_v;      // CMU number with maximum cell voltage
	uint8_t cell_min_v;     // Cell number with minimum cell voltage
	uint8_t cell_max_v;     // Cell number with maximum cell voltage
#endif // _BMU_CELL_V
#if _BMU_CMU_TMP
	uint16_t min_cell_tmp;  // Minimum Cell Temperature
	uint16_t max_cell_tmp;  // Maximum Cell Temperature
	uint8_t cmu_min_tmp;    // CMU number with minimum cell temperature
	uint8_t cmu_max_tmp;    // CMU number with maximum cell temperature
#endif // _BMU_CMU_TMP
#if _BMU_BAL_THRES
	uint16_t bal_thres_rising;
	uint16_t bal_thres_falling;
#endif // _BMU_BAL_THRES
#if _BMU_CMU_CNT
	uint8_t cmu_count;
#endif // _BMU_CMU_CNT
#if _BMU_VER
	uint16_t bmu_fw_ver;
	uint8_t bmu_hw_ver;
	uint8_t bmu_model_id;
#endif // _BMU_VER
#if _BMU_FAN == 1
	uint16_t fan0_spd;
#elif _BMU_FAN == 2
	uint16_t fan1_spd;
#elif _BMU_FAN == 3
	uint16_t fan0_spd;
	uint16_t fan1_spd;
#endif // _BMU_FAN
#if _BMU_12V_CONSUM
	uint16_t fan_contactor_12v_ma;
	uint16_t cmu_12v_ma;
#endif // _BMU_12V_CONSUM

	/// Calculated & Peaks
#if _BMU_POWER
	uint32_t watts;
	float watt_hrs;
#if _BMU_PEAKS // && _BMU_POWER
	uint32_t max_watts;
#endif // _BMU_PEAKS
#endif // _BMU_POWER
#if _BMU_PEAKS
	uint32_t max_bus_v;
	int32_t max_bus_i;
#endif // _BMU_PEKAS
} BMU;

struct STATS_STRUCT
{
	unsigned int ramp_speed;  // .1%/cycle
	float odometer;           // km
	float odometer_tr;        // km
	float max_speed;          // kmh
	float cruise_speed;       // kmh
	uint8_t buz_tim;          // 10mS ticks to sound buzzer
	uint8_t paddle_mode;
	uint32_t avg_power_counter;
	volatile uint16_t flags;
	volatile uint8_t errors;
} stats;

/// stats.flags
#define STATS_DRV_MODE		((stats.flags & 0x0001) >> 0)
#define STATS_BUZZER		((stats.flags & 0x0002) >> 1)
#define STATS_ARMED			((stats.flags & 0x0004) >> 2)
#define STATS_CR_ACT		((stats.flags & 0x0008) >> 3)
#define STATS_CR_STS		((stats.flags & 0x0010) >> 4)
#define STATS_HAZARDS		((stats.flags & 0x0020) >> 5)
#define STATS_LEFT			((stats.flags & 0x0040) >> 6)
#define STATS_RIGHT			((stats.flags & 0x0080) >> 7)
#define STATS_BRAKE			((stats.flags & 0x0100) >> 8)
#define STATS_UNUSED1		((stats.flags & 0x0200) >> 9)
#define STATS_UNUSED2		((stats.flags & 0x0400) >> 10)
#define STATS_UNUSED3		((stats.flags & 0x0800) >> 11)
#define STATS_UNUSED4		((stats.flags & 0x1000) >> 12)
#define STATS_UNUSED5		((stats.flags & 0x2000) >> 13)
#define STATS_UNUSED6		((stats.flags & 0x4000) >> 14)
#define STATS_UNUSED7		((stats.flags & 0x8000) >> 15)

#define SET_STATS_DRV_MODE	stats.flags |= 0x0001;	// Sports Flagged
#define SET_STATS_BUZZER	stats.flags |= 0x0002;
#define SET_STATS_ARMED		stats.flags |= 0x0004;
#define SET_STATS_CR_ACT	stats.flags |= 0x0008;
#define SET_STATS_CR_STS	stats.flags |= 0x0010;
#define SET_STATS_HAZARDS	stats.flags |= 0x0020;
#define SET_STATS_LEFT		stats.flags |= 0x0040;
#define SET_STATS_RIGHT		stats.flags |= 0x0080;
#define SET_STATS_BRAKE		stats.flags |= 0x0100;
#define SET_STATS_UNUSED1	stats.flags |= 0x0200;
#define SET_STATS_UNUSED2	stats.flags |= 0x0400;
#define SET_STATS_UNUSED3	stats.flags |= 0x0800;
#define SET_STATS_UNUSED4	stats.flags |= 0x1000;
#define SET_STATS_UNUSED5	stats.flags |= 0x2000;
#define SET_STATS_UNUSED6	stats.flags |= 0x4000;
#define SET_STATS_UNUSED7	stats.flags |= 0x8000;

#define CLR_STATS_DRV_MODE	stats.flags &= 0xFFFE;	// Economy Flagged
#define CLR_STATS_BUZZER	stats.flags &= 0xFFFD;
#define CLR_STATS_ARMED		stats.flags &= 0xFFFB;
#define CLR_STATS_CR_ACT	stats.flags &= 0xFFF7;
#define CLR_STATS_CR_STS	stats.flags &= 0xFFEF;
#define CLR_STATS_HAZARDS	stats.flags &= 0xFFDF;
#define CLR_STATS_LEFT		stats.flags &= 0xFFBF;
#define CLR_STATS_RIGHT		stats.flags &= 0xFF7F;
#define CLR_STATS_BRAKE		stats.flags &= 0xFEFF;
#define CLR_STATS_UNUSED1	stats.flags &= 0xFDFF;
#define CLR_STATS_UNUSED2	stats.flags &= 0xFBFF;
#define CLR_STATS_UNUSED3	stats.flags &= 0xF7FF;
#define CLR_STATS_UNUSED4	stats.flags &= 0xEFFF;
#define CLR_STATS_UNUSED5	stats.flags &= 0xDFFF;
#define CLR_STATS_UNUSED6	stats.flags &= 0xBFFF;
#define CLR_STATS_UNUSED7	stats.flags &= 0x7FFF;

#define TOG_STATS_DRV_MODE	stats.flags ^= 0x0001;
#define TOG_STATS_BUZZER	stats.flags ^= 0x0002;
#define TOG_STATS_ARMED		stats.flags ^= 0x0004;
#define TOG_STATS_CR_ACT	stats.flags ^= 0x0008;
#define TOG_STATS_CR_STS	stats.flags ^= 0x0010;
#define TOG_STATS_HAZARDS	stats.flags ^= 0x0020;
#define TOG_STATS_LEFT		stats.flags ^= 0x0040;
#define TOG_STATS_RIGHT		stats.flags ^= 0x0080;
#define TOG_STATS_BRAKE		stats.flags ^= 0x0100;
#define TOG_STATS_UNUSED1	stats.flags ^= 0x0200;
#define TOG_STATS_UNUSED2	stats.flags ^= 0x0400;
#define TOG_STATS_UNUSED3	stats.flags ^= 0x0800;
#define TOG_STATS_UNUSED4	stats.flags ^= 0x1000;
#define TOG_STATS_UNUSED5	stats.flags ^= 0x2000;
#define TOG_STATS_UNUSED6	stats.flags ^= 0x4000;
#define TOG_STATS_UNUSED7	stats.flags ^= 0x8000;

/// stats.errors
#define STATS_SWOC_ACK		((stats.errors & 0x01) >> 0)
#define STATS_HWOC_ACK		((stats.errors & 0x02) >> 1)
#define STATS_COMMS			((stats.errors & 0x04) >> 2)
#define STATS_FAULT			((stats.errors & 0x18) >> 3)
#define STATS_NO_ARR_HV		((stats.errors & 0x20) >> 5)
#define STATS_BMU_ACK		((stats.errors & 0x40) >> 6)
#define STATS_STOP			((stats.errors & 0x80) >> 7)

#define SET_STATS_SWOC_ACK	stats.errors |= 0x01;
#define SET_STATS_HWOC_ACK	stats.errors |= 0x02;
#define SET_STATS_COMMS		stats.errors |= 0x04;
#define SET_STATS_NO_ARR_HV	stats.errors |= 0x20;
#define SET_STATS_BMU_ACK	stats.errors |= 0x40;
#define SET_STATS_STOP		stats.errors |= 0x80;

#define CLR_STATS_SWOC_ACK	stats.errors &= 0xFE;
#define CLR_STATS_HWOC_ACK	stats.errors &= 0xFD;
#define CLR_STATS_COMMS		stats.errors &= 0xFB;
#define CLR_STATS_NO_ARR_HV	stats.errors &= 0xDF;
#define CLR_STATS_BMU_ACK	stats.errors &= 0xBF;
#define CLR_STATS_STOP		stats.errors &= 0x7F;

#define TOG_STATS_SWOC_ACK	stats.errors ^= 0x01;
#define TOG_STATS_HWOC_ACK	stats.errors ^= 0x02;
#define TOG_STATS_COMMS		stats.errors ^= 0x04;
#define TOG_STATS_NO_ARR_HV	stats.errors ^= 0x20;
#define TOG_STATS_BMU_ACK	stats.errors ^= 0x40;
#define TOG_STATS_STOP		stats.errors ^= 0x80;

struct DRIVE_STRUCT
{
	float speed_rpm;
	float current;
} drive;

typedef struct CLOCK_STRUCT
{
	uint8_t t_ms;   // (mS / 10)
	uint8_t t_s;
	uint8_t t_m;
	uint8_t t_h;
	uint32_t t_d;
	uint8_t blink;  // half second toggle bit
} CLOCK;

typedef struct SHUNT_STRUCT
{
	float bat_v;
	float bat_i;
	float mppt_i;
	float watt_hrs;
	float watts;
	float max_bat_v;
	float max_bat_i;
	float max_mppt_i;
	float max_watts;
	uint8_t con_tim;
} SHUNT;

#endif /* STRUCT_H_ */
