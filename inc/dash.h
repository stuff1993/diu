#ifndef DASH_H_
#define DASH_H_

#include "struct.h"
#include "can.h"

#ifndef AUTO_SWOC
#define AUTO_SWOC 1 // auto reset SWOCs
#endif

#define SYSTICK_INT_MS    10
#define SYSTICK_SEC_COUNT (1000 / SYSTICK_INT_MS) // Ticks in a second
#define SYSTICK_HOUR_DIV  3600000.0f / SYSTICK_INT_MS // */s hourly divisor (W -> Whr)

#define MAX_RGN_DZ        0.15  // V at pin
#define MIN_RGN_DZ        0.15
#define MAX_THR_DZ        0.15
#define MIN_THR_DZ        0.15
#define LOW_PAD_V         0.25
#define MID_PAD_V         1.25
#define HGH_PAD_V         2.25
#define ADC_POINTS_PER_V  1240.909091 // 0-3.3V on pin

#define MAX_ESC_CUR         65.0  // Amps

#define IIR_GAIN_ELECTRICAL 1000
#define IIR_GAIN_THERMAL    10
#define MIN_CELL_THRESHOLD  2500

// CAN offsets
#define BMU_INFO            0xF4
#define MPPT_RPLY			0x60

// Configuration Defaults
#define CAN_ESC				0x400
#define CAN_CONTROL			0x500
#define CAN_DASH_REPLY		0x510
#define CAN_DASH_REQUEST	0x520
#define CAN_SHUNT			0x530
#define CAN_BMU				0x600
#define CAN_MPPT0           0x71F
#define CAN_MPPT1			0x716
#define CAN_MPPT2			0x719
#define WHEEL_D				0.557f
#define MAX_THROTTLE_LOW	750
#define LOW_SPEED_THRES		20

// Driver Mode defaults
// Race
#define D0_MAX_THROTTLE		1000
#define D0_MAX_REGEN		1000
#define D0_THROTTLE_RAMP	10
#define D0_REGEN_RAMP		30
// Hot Lap
#define D1_MAX_THROTTLE		1000
#define D1_MAX_REGEN		1000
#define D1_THROTTLE_RAMP	50
#define D1_REGEN_RAMP		50
// Test
#define D2_MAX_THROTTLE		1000
#define D2_MAX_REGEN		1000
#define D2_THROTTLE_RAMP	10
#define D2_REGEN_RAMP		30
// Display
#define D3_MAX_THROTTLE		750
#define D3_MAX_REGEN		750
#define D3_THROTTLE_RAMP	5
#define D3_REGEN_RAMP		30

// OUTPUTS
#define BUZZER_ON       LPC_GPIO0->FIOSET |= (1<<3);
#define BUZZER_OFF      LPC_GPIO0->FIOCLR |= (1<<3);

#define FAULT_ON        LPC_GPIO2->FIOSET |= (1<<0);LPC_GPIO0->FIOSET |= (1<<27);
#define FAULT_OFF       LPC_GPIO2->FIOCLR |= (1<<0);LPC_GPIO0->FIOCLR |= (1<<27);

#define HV_ON           LPC_GPIO1->FIOSET |= (1<<8);
#define HV_OFF          LPC_GPIO1->FIOCLR |= (1<<8);

#define BLINKER_R_ON    LPC_GPIO3->FIOSET |= (1<<26);
#define BLINKER_R_OFF   LPC_GPIO3->FIOCLR |= (1<<26);

#define BLINKER_L_ON    LPC_GPIO3->FIOSET |= (1<<25);
#define BLINKER_L_OFF   LPC_GPIO3->FIOCLR |= (1<<25);

// Contactor drivers
#define C_2_3_ON        LPC_GPIO1->FIOSET |= (1<<19);
#define C_2_3_OFF       LPC_GPIO1->FIOCLR |= (1<<19);

#define C_1_ON          LPC_GPIO1->FIOSET |= (1<<20);
#define C_1_OFF         LPC_GPIO1->FIOCLR |= (1<<20);

#define REVERSE_ON      LPC_GPIO1->FIOSET |= (1<<26);
#define REVERSE_OFF     LPC_GPIO1->FIOCLR |= (1<<26);

#define REGEN_ON        LPC_GPIO1->FIOSET |= (1<<24);
#define REGEN_OFF       LPC_GPIO1->FIOCLR |= (1<<24);

#define NEUTRAL_ON      LPC_GPIO1->FIOSET |= (1<<25);
#define NEUTRAL_OFF     LPC_GPIO1->FIOCLR |= (1<<25);

#define DRIVE_ON        LPC_GPIO1->FIOSET |= (1<<23);
#define DRIVE_OFF       LPC_GPIO1->FIOCLR |= (1<<23);

#define ECO_ON          LPC_GPIO1->FIOSET |= (1<<30);
#define ECO_OFF         LPC_GPIO1->FIOCLR |= (1<<30);

#define SPORTS_ON       LPC_GPIO1->FIOSET |= (1<<31);
#define SPORTS_OFF      LPC_GPIO1->FIOCLR |= (1<<31);

#define HAZARDS_ON      BLINKER_R_ON;BLINKER_L_ON;
#define HAZARDS_OFF     BLINKER_R_OFF;BLINKER_L_OFF;

// INPUTS
#define MECH_BRAKE      !(LPC_GPIO0->FIOPIN & (1<<25))

#define LEFT            !(LPC_GPIO1->FIOPIN & (1<<27))
#define INCREMENT       !(LPC_GPIO0->FIOPIN & (1<<1))
#define DECREMENT       !(LPC_GPIO1->FIOPIN & (1<<28))
#define RIGHT           !(LPC_GPIO0->FIOPIN & (1<<0))
#define SELECT          !(LPC_GPIO1->FIOPIN & (1<<29))

#define FORWARD         !(LPC_GPIO0->FIOPIN & (1<<10))
#define REVERSE         !(LPC_GPIO0->FIOPIN & (1<<11))

#define SPORTS_MODE     !(LPC_GPIO2->FIOPIN & (1<<10)) // Aux_ON
#define ECONOMY_MODE    !(LPC_GPIO2->FIOPIN & (1<<11)) // Aux_OFF

#define CC_ON           !(LPC_GPIO2->FIOPIN & (1<<12))
#define CC_OFF          !(LPC_GPIO2->FIOPIN & (1<<13))

#define LEFT_ON         !(LPC_GPIO1->FIOPIN & (1<<1))
#define RIGHT_ON        !(LPC_GPIO1->FIOPIN & (1<<0))

// TOGGLES
#define ECONOMY (0)
#define SPORTS  (1)

#define ON      (1)
#define OFF     (0)

// EEPROM Addresses
// These values only define the order of storage in the EEPROM
// Logic in eeprom.c handles the actual EEPROM memory addressing
#define ADD_BUZZ		0
#define ADD_ODO			1
#define ADD_ODOTR		2
#define ADD_MPPT1WHR	3
#define ADD_BMUWHR		4
#define ADD_MPPT2WHR	5
#define ADD_CONF1		6
#define ADD_CONF2		7
#define ADD_CONF3		8
#define ADD_CONF4		9
#define ADD_CONF5		10
#define ADD_CONF6		11
#define ADD_DRV0_CONF1	12
#define ADD_DRV0_CONF2	13
#define ADD_DRV1_CONF1	14
#define ADD_DRV1_CONF2	15
#define ADD_DRV2_CONF1	16
#define ADD_DRV2_CONF2	17
#define ADD_DRV3_CONF1	18
#define ADD_DRV3_CONF2	19
#define ADD_MPPT0WHR	20

// Function Prototypes
void main_driver_check(void);
__attribute__((always_inline)) void mppt_data_extract(MPPT *_mppt, CAN_MSG *_msg);
__attribute__((always_inline)) void mppt_data_transfer(CAN_MSG *_msg);
__attribute__((always_inline)) void esc_data_extract(MOTORCONTROLLER *_esc, CAN_MSG *_msg);
__attribute__((always_inline)) void dash_data_extract(CAN_MSG *_msg);
__attribute__((always_inline)) void shunt_data_extract(SHUNT *_shunt, CAN_MSG *_msg);
__attribute__((always_inline)) void bmu_data_extract(BMU *_shunt, CAN_MSG *_msg);
void main_input_check(void);
int main_fault_check(void);
void main_drive(void);
void main_paddles(uint32_t _pad1, uint32_t _pad2, uint16_t *_thr, uint16_t *_rgn);
void main_lights(void);
void main_can_handler(void);
void esc_reset(void);
void nonpersistent_load(void);
void persistent_load(void);
void persistent_store(void);
void gpio_init(void);
void led_test(void);
void motorcontroller_init(void);
void buzzer(uint8_t val);
void force_buzzer(uint8_t val);
void BOD_init(void);
void engage_contactors(void);
void disengage_contactors(void);

#endif /* DASH_H_ */
