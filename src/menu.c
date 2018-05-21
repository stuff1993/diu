/*
 * menu.c
 *
 *  Created on: 29 May 2015
 *      Author: Stuart G
 */
 
/*
 * Sequential menu loop logic
 * Next item:     SELECTOR = (SELECTOR + 1) % NUM_ITEMS;
 * Previous item: SELECTOR = (SELECTOR + NUM_ITEMS - 1) % NUM_ITEMS;
 */

#include "lpc17xx.h"

#include <stdio.h>
#include <stdint.h>
#include "struct.h"
#include "inputs.h"
#include "can.h"
#include "lcd.h"
#include "menu.h"
#include "dash.h"
#include "timer.h"
#include "eeprom.h"

extern CLOCK clock;
extern BMU bmu;
extern SHUNT shunt;
extern MOTORCONTROLLER esc;
extern MPPT mppt[];
extern CAN_MSG can_tx2_buf;
extern uint16_t thr_pos;
extern uint16_t rgn_pos;
extern CAR_CONFIG config;
extern DRIVER_CONFIG drv_config[];

#define NUM_CONFIG 28

CONFIG_DISPLAY options[NUM_CONFIG] =
    {
        {"ESC          %#05x", 1, &(config.can_esc), 0, 0x7FF},
        {"CONTROL      %#05x", 1, &(config.can_control), 0, 0x7FF},
        {"REPLY        %#05x", 1, &(config.can_dash_reply), 0, 0x7FF},
        {"REQUEST      %#05x", 1, &(config.can_dash_request), 0, 0x7FF},
        {"SHUNT        %#05x", 1, &(config.can_shunt), 0, 0x7FF},
        {"BMU          %#05x", 1, &(config.can_bmu), 0, 0x7FF},
        {"MPPTX        %#05x", 1, &(config.can_mppt0), 0, 0x7FF},
        {"MPPT1        %#05x", 1, &(config.can_mppt1), 0, 0x7FF},
        {"MPPT2        %#05x", 1, &(config.can_mppt2), 0, 0x7FF},
        {"WHEEL d (m)  %5.3f", 0x80 | 3, &(config.wheel_d), 0, 0},
        {"MAX THR LOW  %04u", 1, &(config.max_thr_lowspd), 0, 1000},
        {"LOW SPD      %03u", 0x80 | 0, &(config.low_spd_threshold), 0, 0},
        {"DRV0 MAX THR %04u", 1, &(drv_config[0].max_throttle), 0, 1000},
        {"DRV0 MAX RGN %04u", 1, &(drv_config[0].max_regen), 0, 1000},
        {"DRV0 THR RMP %04u", 1, &(drv_config[0].throttle_ramp_rate), 0, 1000},
        {"DRV0 RGN RMP %04u", 1, &(drv_config[0].regen_ramp_rate), 0, 1000},
        {"DRV1 MAX THR %04u", 1, &(drv_config[1].max_throttle), 0, 1000},
        {"DRV1 MAX RGN %04u", 1, &(drv_config[1].max_regen), 0, 1000},
        {"DRV1 THR RMP %04u", 1, &(drv_config[1].throttle_ramp_rate), 0, 1000},
        {"DRV1 RGN RMP %04u", 1, &(drv_config[1].regen_ramp_rate), 0, 1000},
        {"DRV2 MAX THR %04u", 1, &(drv_config[2].max_throttle), 0, 1000},
        {"DRV2 MAX RGN %04u", 1, &(drv_config[2].max_regen), 0, 1000},
        {"DRV2 THR RMP %04u", 1, &(drv_config[2].throttle_ramp_rate), 0, 1000},
        {"DRV2 RGN RMP %04u", 1, &(drv_config[2].regen_ramp_rate), 0, 1000},
        {"DRV3 MAX THR %04u", 1, &(drv_config[3].max_throttle), 0, 1000},
        {"DRV3 MAX RGN %04u", 1, &(drv_config[3].max_regen), 0, 1000},
        {"DRV3 THR RMP %04u", 1, &(drv_config[3].throttle_ramp_rate), 0, 1000},
        {"DRV3 RGN RMP %04u", 1, &(drv_config[3].regen_ramp_rate), 0, 1000}
    };


// Not in array, reference manually

/******************************************************************************
 ** Function name:  menu_errOnStart
 **
 ** Description:    Error screen on boot
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_errOnStart (void)
{
  lcd_putstring(0, 0, "**    CAUTION!    **");
  lcd_putstring(1, 0, EROW);
  lcd_putstring(2, 0, "   GEARS ENGAGED!   ");
  lcd_putstring(3, 0, EROW);
}

/******************************************************************************
 ** Function name:  menu_esc_wait
 **
 ** Description:    Screen to display while waiting for ESC response
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_esc_wait(void)
{
  lcd_putstring(0, 0, "**  WSU ASC 2018  **");
  lcd_putstring(1, 0, " WAITING FOR ESC... ");
  lcd_putstring(2, 0, "   SELECT TO SKIP   ");
  lcd_putstring(3, 0, EROW);
}

/******************************************************************************
 ** Function name:  menu_driver
 **
 ** Description:    Select driver type. Determines menus available.
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_driver (void)
{
  menu.driver = 255;
  menu.submenu_pos = 0;
  CLR_MENU_SELECTED;

  _lcd_putTitle("-DRIVER-");
  lcd_putstring(1, 0, "   RACE      HOT LAP");
  lcd_putstring(2, 0, "   TEST      DISPLAY");

  while(menu.driver == 255)
  {
    _lcd_putTitle("-DRIVER-");
    switch(menu.submenu_pos)
    {
      default:
      case 0:
        lcd_putstring(1, 0, SELECTOR);
        lcd_putstring(1, 10, DESELECTOR);
        lcd_putstring(2, 0, DESELECTOR);
        lcd_putstring(2, 10, DESELECTOR);
        break;
      case 1:
        lcd_putstring(1, 0, DESELECTOR);
        lcd_putstring(1, 10, SELECTOR);
        lcd_putstring(2, 0, DESELECTOR);
        lcd_putstring(2, 10, DESELECTOR);
        break;
      case 2:
        lcd_putstring(1, 0, DESELECTOR);
        lcd_putstring(1, 10, DESELECTOR);
        lcd_putstring(2, 0, SELECTOR);
        lcd_putstring(2, 10, DESELECTOR);
        break;
      case 3:
        lcd_putstring(1, 0, DESELECTOR);
        lcd_putstring(1, 10, DESELECTOR);
        lcd_putstring(2, 0, DESELECTOR);
        lcd_putstring(2, 10, SELECTOR);
        break;
    }

    if (btn_release_select())
    {
      menu.driver = menu.submenu_pos;
    }

    if (btn_release_increment())
    {
      // (pos + width) % total
      menu.submenu_pos = (menu.submenu_pos + 2) % 4;
    }

    if (btn_release_decrement())
    {
      // (pos + total - width) % total
      menu.submenu_pos = (menu.submenu_pos + 2) % 4;
    }

    if (btn_release_left())
    {
      // ((pos / width) * width) + ((pos + 1) % width) -- Get row number, get item at start of row number, get next width looping on width
      menu.submenu_pos = ((menu.submenu_pos / 2) * 2) + ((menu.submenu_pos + 1) % 2);
    }

    if (btn_release_right())
    {
      // ((pos / width) * width) + ((pos + width - 1) % width) -- Get row number, get item at start of row number, get next width looping on width
      menu.submenu_pos = ((menu.submenu_pos / 2) * 2) + ((menu.submenu_pos + 1) % 2);
    }
  }
}

/******************************************************************************
 ** Function name:  menu_intro
 **
 ** Description:    Boot intro screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_intro (void)
{
  lcd_putstring(0, 0, "**  WSU ASC 2018  **");
  lcd_putstring(1, 0, EROW);
  lcd_putstring(2, 0, "   CRIMSON Driver   ");
  lcd_putstring(3, 0, "   Interface v4.0   ");
  delayMs(1, 3500);

  lcd_putstring(0, 0, "**  WSU ASC 2018  **");
  lcd_putstring(1, 0, EROW);
  lcd_putstring(2, 0, "    BUZZER Test..   ");
  lcd_putstring(3, 0, EROW);
  BUZZER_ON
  delayMs(1, 1000);
  BUZZER_OFF

  lcd_clear();
}
///////////////////////////////////////////////

///////////////////////////////////////////////
/// menus array

/******************************************************************************
 ** Function name:  menu_info
 **
 ** Description:    Car information screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_info (void)
{
  _lcd_putTitle("-INFO-");
  lcd_putstring(1, 0, "CRIMSON Dash 4.0    ");
  lcd_putstring(2, 0, "HW Version: 3.0     ");
  lcd_putstring(3, 0, "SW Version: 4.0     ");
}

/******************************************************************************
 ** Function name:  menu_escBus
 **
 ** Description:    Data screen for precharge
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_escBus (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-ESC BUS-");

  len = sprintf(buffer, "Bus Voltage: %05.1fV", esc.bus_v);
  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "Bat Voltage: %05.1fV", shunt.bus_v);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "CON1: %d   CON23: %d", STATS_C_1, STATS_C_2_3);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)
}

/******************************************************************************
 ** Function name:  menu_home
 **
 ** Description:    Speed, drive, array power, basic errors
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_home (void)
{
  char buffer[20];

  _lcd_putTitle("-HOME-");

  sprintf(buffer, "MPPT: %3luW ", mppt[0].watts + mppt[1].watts + mppt[2].watts);
  sprintf(buffer + 11, "MCV:%4dV", bmu.min_cell_v);
  lcd_putstring(1, 0, buffer);

  sprintf(buffer, "Bat:  %3.0fW Thr:", shunt.bus_watts);
  if (STATS_CR_ACT){sprintf(buffer + 15, "%3.0f%% ", esc.bus_i * (100 / MAX_ESC_CUR));}
  else if (rgn_pos){sprintf(buffer + 11, "Brk:%3d%% ", rgn_pos/10);}
  else if (FORWARD){sprintf(buffer + 15, "%3d%% ", thr_pos/10);}
  else if (REVERSE){sprintf(buffer + 15, "%3d%% ", thr_pos/10);}
  else{sprintf(buffer + 15, "---%% ");}
  lcd_putstring(2, 0, buffer);

  sprintf(buffer, "Motor:%3.0fW ", esc.watts);
  if (esc.error)                                  {sprintf(buffer + 11, "Err:ESC  ");}
  else if (mppt[0].flags & 0xF)                   {sprintf(buffer + 11, "Err:MPPTX");}
  else if (mppt[1].flags & 0xF)                   {sprintf(buffer + 11, "Err:MPPT1");}
  else if (mppt[2].flags & 0xF)                   {sprintf(buffer + 11, "Err:MPPT2");}
  else if (bmu.status & 0x00001FBF)               {sprintf(buffer + 11, "Err:BMU  ");}
  else if (STATS_NO_ARR_HV)                       {sprintf(buffer + 11, "Err:ARRHV");}
  else if (!(mppt[1].con_tim || mppt[2].con_tim)) {sprintf(buffer + 11, "Err:NoARR");}
  else if (!(shunt.con_tim))                      {sprintf(buffer + 11, "Err:NoSHT");}
  else                                            {sprintf(buffer + 11, "V: %05.1fV", shunt.bus_v);}
  lcd_putstring(3, 0, buffer);
}

/******************************************************************************
 ** Function name:  menu_controls
 **
 ** Description:    Drive details screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_controls (void)
{
  char buffer[20];

  _lcd_putTitle("-CONTROLS-");

  if (FORWARD)      {lcd_putstring(1,0, "MODE:          DRIVE");}
  else if (REVERSE) {lcd_putstring(1,0, "MODE:        REVERSE");}
  else              {lcd_putstring(1,0, "MODE:        NEUTRAL");}

  if (!rgn_pos)
  {
    sprintf(buffer, "OUTPUT:       %5.1f%%", drive.current*100);
    lcd_putstring(2, 0, buffer);

    sprintf(buffer, "THROTTLE:     %3d.%d%%", thr_pos/10,thr_pos%10);
    lcd_putstring(3, 0, buffer);
  }
  else
  {
    sprintf(buffer, "OUTPUT:       %5.1f%%", drive.current*100);
    lcd_putstring(2, 0, buffer);

    sprintf(buffer, "REGEN:        %3d.%d%%", rgn_pos/10,rgn_pos%10);
    lcd_putstring(3, 0, buffer);
  }
}

/******************************************************************************
 ** Function name:  menu_cruise
 **
 ** Description:    Cruise control screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_cruise(void)
{
  _lcd_putTitle("-CRUISE-");

  if (!REVERSE)
  {
    char buffer[20];

    if (STATS_CR_STS && STATS_CR_ACT)
    {
      lcd_putstring(1, 0, " STS:  ON  ACT:  ON ");
      sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", stats.cruise_speed, esc.velocity_kmh);
      lcd_putstring(2, 0, buffer);
      sprintf(buffer, " THR: %3.0f%% Pow:%4.0fW", esc.bus_i * (100 / MAX_ESC_CUR), shunt.bus_watts);
      lcd_putstring(3, 0, buffer);

      // Button presses
      if (btn_release_select())
      {
        CLR_STATS_CR_ACT;
      }
      if (btn_release_increment())
      {
        stats.cruise_speed += 1;
      }
      if ((stats.cruise_speed > 1) && btn_release_decrement())
      {
        stats.cruise_speed -= 1;
      }
    }
    else if (STATS_CR_STS && !STATS_CR_ACT)
    {
      lcd_putstring(1, 0, " STS:  ON  ACT: OFF ");
      sprintf(buffer, " SET: %3.0f  SPD: %3.0f ", stats.cruise_speed, esc.velocity_kmh);
      lcd_putstring(2, 0, buffer);
      lcd_putstring(3, 0, EROW);

      // Button presses
      if (btn_release_select())
      {
        CLR_STATS_CR_STS;
        stats.cruise_speed = 0;
      }
      if ((stats.cruise_speed > 1) && btn_release_increment())
      {
        SET_STATS_CR_ACT;
      }
      if (btn_release_decrement())
      {
        stats.cruise_speed = esc.velocity_kmh;
        SET_STATS_CR_ACT;
      }
    }
    else if (STATS_CR_ACT && !STATS_CR_STS) // Should never trip, but just in case
    {
      lcd_putstring(1, 0, " STS: OFF  ACT:  ON ");
      lcd_putstring(2, 0, "    CRUISE ERROR    ");
      lcd_putstring(3, 0, "     RESETTING      ");

      CLR_STATS_CR_ACT;
      CLR_STATS_CR_STS;
      stats.cruise_speed = 0;
    }
    else
    {
      lcd_putstring(1, 0, " STS: OFF  ACT: OFF ");
      sprintf(buffer, " SET:      SPD: %3.0f ", esc.velocity_kmh);
      lcd_putstring(2, 0, buffer);
      lcd_putstring(3, 0, EROW);

      // Button presses
      if (btn_release_select())
      {
        stats.cruise_speed = 0;
        SET_STATS_CR_STS;
      }
    }
  }
  else // no cruise in reverse
  {
    lcd_putstring(1, 0, " STS: OFF  ACT: OFF ");
    lcd_putstring(2, 0, "  REVERSE ENGAGED!  ");
    lcd_putstring(3, 0, EROW);

    CLR_STATS_CR_ACT;
    CLR_STATS_CR_STS;
    stats.cruise_speed = 0;
  }
}

/******************************************************************************
 ** Function name:  _menu_MPPT
 **
 ** Description:    MPPT1 information screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
_menu_MPPT(MPPT *_mppt)
{
  if (_mppt->con_tim)
  {
    char buffer[20];
    int len;

    len = sprintf(buffer, "IN: %3lu.%luV @ %2lu.%02luA", _mppt->v_in/10, _mppt->v_in%10, _mppt->i_in/100, _mppt->i_in%100);
    lcd_putstring(1, 0, buffer);
    PAD_ROW(1, len)

    len = sprintf(buffer, "OUT:%3lu.%luV @ %4luW", _mppt->v_out/10, _mppt->v_out%10, _mppt->watts);
    lcd_putstring(2, 0, buffer);
    PAD_ROW(2, len)

    sprintf(buffer, "%2lu%cC", _mppt->tmp, (char)0xD2);
    lcd_putstring(3,16, buffer);

    if (clock.blink)
    {
      if (_mppt->flags & 0x2)      {sprintf(buffer, "OVER TEMP       ");}
      else if (_mppt->flags & 0x8) {sprintf(buffer, "LOW IN VOLTAGE  ");}
      else if (_mppt->flags & 0x1) {sprintf(buffer, "BATTERY FULL    ");}
      else if (_mppt->flags & 0x4) {sprintf(buffer, "NO BATTERY      ");}
      else                         {sprintf(buffer, "                ");}
      lcd_putstring(3, 0, buffer);
    }
    else{lcd_putstring(3, 0, "                ");}
  }
  else // No connection
  {
    lcd_putstring(1, 0, EROW);
    lcd_putstring(3, 0, EROW);

    if (clock.blink)
    {
      lcd_putstring(2, 0, "**CONNECTION ERROR**");
    }
    else
    {
      lcd_putstring(2, 0, EROW);
    }
  }
}

void
menu_MPPT0(void)
{
  _lcd_putTitle("-MPPTXTRA-");
  _menu_MPPT(&(mppt[0]));
}

void
menu_MPPT1(void)
{
  _lcd_putTitle("-MPPT 1-");
  _menu_MPPT(&(mppt[1]));
}

void
menu_MPPT2(void)
{
  _lcd_putTitle("-MPPT 2-");
  _menu_MPPT(&(mppt[2]));
}

/******************************************************************************
 ** Function name:  menu_MPPTPower
 **
 ** Description:    Total power from MPPTs
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_MPPTPower (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-POWER IN-");

  len = sprintf(buffer, "MPPT1: %.2fWh", mppt[1].watt_hrs);
  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "MPPT2: %.2fWh", mppt[2].watt_hrs);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "TOTAL: %.2fWh", mppt[0].watt_hrs + mppt[1].watt_hrs + mppt[2].watt_hrs);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)

  if (btn_release_inc_sel() == 3)
  {
      mppt[0].watt_hrs = 0;
      mppt[1].watt_hrs = 0;
      mppt[2].watt_hrs = 0;
      buzzer(50);
  }
}

/******************************************************************************
 ** Function name:  menu_motor
 **
 ** Description:    Motor stats screens
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_motor (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 3;

  switch(menu.submenu_pos)
  {
    default:
      menu.submenu_pos = 0;
    case 0:
      _lcd_putTitle("-MTR PWR-");

      len = sprintf(buffer, "%5.3fV @ %5.3fA", esc.bus_v, esc.bus_i);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Power: %.3fW",  esc.watts);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)
      break;
    case 1:
      _lcd_putTitle("-MTR WHR-");

      len = sprintf(buffer, "%.3f W/hrs", esc.watt_hrs);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      lcd_putstring(2, 0, EROW);
      break;
    case 2:
      _lcd_putTitle("-MTR PKS-");

      len = sprintf(buffer, "%5.3fV @ %5.3fA", esc.max_bus_v, esc.max_bus_i);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Power: %.3fW",  esc.max_watts);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      if (btn_release_select())
      {
        esc.max_bus_i = 0;
        esc.max_bus_v = 0;
        esc.max_watts = 0;
        buzzer(50);
      }
      break;
  }
  len = sprintf(buffer, "ERROR: 0x%02x", esc.error);
  lcd_putstring(3,0, buffer);
  PAD_ROW(3, len)

  // BUTTONS
  if (btn_release_increment())
  {
    menu_inc(&menu.submenu_pos, menu.submenu_items);
  }

  if (btn_release_decrement())
  {
    menu_dec(&menu.submenu_pos, menu.submenu_items);
  }
}

/******************************************************************************
 ** Function name:  menu_battery
 **
 ** Description:    Battery stats screens
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_battery (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 4;

  switch(menu.submenu_pos)
  {
    default:
      menu.submenu_pos = 0;
    case 0:
      _lcd_putTitle("-BAT PWR-");

      len = sprintf(buffer, "%5.3fV @ %5.3fA", shunt.bus_v, shunt.bus_i);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Power: %.3fW",  shunt.bus_watts);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      len = sprintf(buffer, "Status: 0x%01lx", bmu.status);
      lcd_putstring(3, 0, buffer);
      PAD_ROW(3, len)
      break;
    case 1:
      _lcd_putTitle("-BAT WHR-");

      len = sprintf(buffer, "Total: %.3f W/hrs", shunt.watt_hrs);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      lcd_putstring(2, 0, EROW);

      lcd_putstring(3, 0, EROW);
      break;
    case 2:
      _lcd_putTitle("-BAT PKS-");

      len = sprintf(buffer, "Voltage: %.3fV", shunt.max_bat_v);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Current: %.3fA",  shunt.max_bat_i);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      len = sprintf(buffer, "Power:   %.3fW", shunt.max_bus_watts);
      lcd_putstring(3, 0, buffer);
      PAD_ROW(3, len)

      if (btn_release_select())
      {
        shunt.max_bat_i = 0;
        shunt.max_bat_v = 0;
        shunt.max_bus_watts = 0;
        buzzer(50);
      }
      break;
    case 3:
      _lcd_putTitle("-BAT CELL-");

      len = sprintf(buffer, "Min Cell V: %4dV", bmu.min_cell_v);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Max Cell V: %4dV",  bmu.max_cell_v);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      len = sprintf(buffer, "Max Cell T: %3d.%d%cC", bmu.max_cell_tmp/10, bmu.max_cell_tmp%10, 0xD2);
      lcd_putstring(3, 0, buffer);
      PAD_ROW(3, len)
      break;
  }

  // BUTTONS
  if (btn_release_increment())
  {
    menu_inc(&menu.submenu_pos, menu.submenu_items);
  }

  if (btn_release_decrement())
  {
    menu_dec(&menu.submenu_pos, menu.submenu_items);
  }
}

/******************************************************************************
 ** Function name:  menu_temperature
 **
 ** Description:    Temperatures screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_average_power (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-POWER-");

  len = sprintf(buffer, "ESC:   %.3fW", esc.avg_power/stats.avg_power_counter);
  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "MPPT1: %.3fW",  ((float)mppt[1].avg_power)/stats.avg_power_counter);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "MPPT2: %.3fW", ((float)mppt[2].avg_power)/stats.avg_power_counter);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)

  if (btn_release_select())
  {
    esc.avg_power = 0;
    mppt[0].avg_power = 0;
    mppt[1].avg_power = 0;
    mppt[2].avg_power = 0;
    stats.avg_power_counter = 0;
    buzzer(50);
  }
}

/******************************************************************************
 ** Function name:  menu_temperature
 **
 ** Description:    Temperatures screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_temperature (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 2;

  _lcd_putTitle("-TEMPS-");

  switch(menu.submenu_pos)
  {
    default:
      menu.submenu_pos = 0;
    case 0:
      len = sprintf(buffer, "Motor:   %.1f%cC", esc.motor_tmp, (char)0xD2);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "Mtr Con: %.1f%cC",  esc.board_tmp, (char)0xD2);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      len = sprintf(buffer, "Battery: %d.%d%cC", bmu.max_cell_tmp/10, bmu.max_cell_tmp%10, (char)0xD2);
      lcd_putstring(3, 0, buffer);
      PAD_ROW(3, len)
      break;
    case 1:
      len = sprintf(buffer, "MPPTX: %lu%cC", mppt[0].tmp, (char)0xD2);
      lcd_putstring(1, 0, buffer);
      PAD_ROW(1, len)

      len = sprintf(buffer, "MPPT1: %lu%cC", mppt[1].tmp, (char)0xD2);
      lcd_putstring(2, 0, buffer);
      PAD_ROW(2, len)

      len = sprintf(buffer, "MPPT2: %lu%cC", mppt[2].tmp, (char)0xD2);
      lcd_putstring(3, 0, buffer);
      PAD_ROW(3, len)
      break;
  }

  // BUTTONS
  if (btn_release_increment())
  {
    menu_inc(&menu.submenu_pos, menu.submenu_items);
  }

  if (btn_release_decrement())
  {
    menu_dec(&menu.submenu_pos, menu.submenu_items);
  }
}

/******************************************************************************
 ** Function name:  menu_debug
 **
 ** Description:    Bus debug screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_debug (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-DEBUG-");

  len = sprintf(buffer, "MPPT I %5.1fA", shunt.mppt_i);

  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "BUS I  %5.1fA", shunt.bus_i);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "BUS V  %5.1fV", shunt.bus_v);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)
}

/******************************************************************************
 ** Function name:  menu_config
 **
 ** Description:    Compiler configurations screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_config (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = NUM_CONFIG;

  _lcd_putTitle("-CONFIG-");

  uint8_t opt_index = (menu.submenu_pos + menu.submenu_items - 2) % menu.submenu_items;
  int i;
  for (i = 1; i < 4; i++)
  {
    menu_inc(&opt_index, menu.submenu_items);
    switch (options[opt_index].type & 0x3F)
    {
    case 0:
      len = sprintf(buffer, options[opt_index].format, *((uint8_t*)options[opt_index].value));
      break;
    case 1:
      len = sprintf(buffer, options[opt_index].format, *((uint16_t*)options[opt_index].value));
      break;
    case 2:
      len = sprintf(buffer, options[opt_index].format, *((uint32_t*)options[opt_index].value));
      break;
    case 3:
      len = sprintf(buffer, options[opt_index].format, *((float*)options[opt_index].value));
      break;
    default:
      len = sprintf(buffer, "?????");
    }

    lcd_putstring(i, 0, buffer);
    PAD_ROW(i, len)
  }

  if (MENU_SELECTED)
  {
    if (btn_release_increment())
    {
      switch (options[menu.submenu_pos].type & 0xBF)
      {
      case 0:
        if ((*((uint8_t*)options[menu.submenu_pos].value)) >= options[menu.submenu_pos].upper_bound)
        {
          break;
        }
      case 128:
        (*((uint8_t*)options[menu.submenu_pos].value))++;
        break;
      case 1:
        if ((*((uint16_t*)options[menu.submenu_pos].value)) >= options[menu.submenu_pos].upper_bound)
        {
          break;
        }
      case 129:
        (*((uint16_t*)options[menu.submenu_pos].value))++;
        break;
      case 2:
        if ((*((uint32_t*)options[menu.submenu_pos].value)) >= options[menu.submenu_pos].upper_bound)
        {
          break;
        }
      case 130:
        (*((uint32_t*)options[menu.submenu_pos].value))++;
        break;
      case 3:
        if ((*((float*)options[menu.submenu_pos].value)) >= options[menu.submenu_pos].upper_bound)
        {
          break;
        }
      case 131:
        (*((float*)options[menu.submenu_pos].value)) += 0.001;
        break;
      }
    }
    else if (btn_release_decrement())
    {
      switch (options[menu.submenu_pos].type & 0x7F)
      {
      case 0:
        if ((*((uint8_t*)options[menu.submenu_pos].value)) <= options[menu.submenu_pos].lower_bound)
        {
          break;
        }
      case 64:
        (*((uint8_t*)options[menu.submenu_pos].value))--;
        break;
      case 1:
        if ((*((uint16_t*)options[menu.submenu_pos].value)) <= options[menu.submenu_pos].lower_bound)
        {
          break;
        }
      case 65:
        (*((uint16_t*)options[menu.submenu_pos].value))--;
        break;
      case 2:
        if ((*((uint32_t*)options[menu.submenu_pos].value)) <= options[menu.submenu_pos].lower_bound)
        {
          break;
        }
      case 66:
        (*((uint32_t*)options[menu.submenu_pos].value))--;
        break;
      case 3:
        if ((*((float*)options[menu.submenu_pos].value)) <= options[menu.submenu_pos].lower_bound)
        {
          break;
        }
      case 67:
        (*((float*)options[menu.submenu_pos].value)) -= 0.001;
        break;
      }
    }
    else if (btn_release_select())
    {
      SET_STATS_CONF_CHANGED
      CLR_MENU_SELECTED
    }
  }
  else
  {
    uint8_t inc_dec = btn_release_inc_dec();
    if (inc_dec == 1)
    {
      menu_inc(&menu.submenu_pos, menu.submenu_items);
    }
    else if (inc_dec == 2)
    {
      menu_dec(&menu.submenu_pos, menu.submenu_items);
    }
    else if (inc_dec == 3)
    {
      config.can_esc = CAN_ESC;
      config.can_control = CAN_CONTROL;
      config.can_dash_reply = CAN_DASH_REPLY;
      config.can_dash_request = CAN_DASH_REQUEST;
      config.can_shunt = CAN_SHUNT;
      config.can_bmu = CAN_BMU;
      config.can_mppt0 = CAN_MPPT0;
      config.can_mppt1 = CAN_MPPT1;
      config.can_mppt2 = CAN_MPPT2;
      config.wheel_d = WHEEL_D;
      config.max_thr_lowspd = MAX_THROTTLE_LOW;
      config.low_spd_threshold = LOW_SPEED_THRES;

      drv_config[0].max_throttle = D0_MAX_THROTTLE;
      drv_config[0].max_regen = D0_MAX_REGEN;
      drv_config[0].throttle_ramp_rate = D0_THROTTLE_RAMP;
      drv_config[0].regen_ramp_rate = D0_REGEN_RAMP;

      drv_config[1].max_throttle = D1_MAX_THROTTLE;
      drv_config[1].max_regen = D1_MAX_REGEN;
      drv_config[1].throttle_ramp_rate = D1_THROTTLE_RAMP;
      drv_config[1].regen_ramp_rate = D1_REGEN_RAMP;

      drv_config[2].max_throttle = D2_MAX_THROTTLE;
      drv_config[2].max_regen = D2_MAX_REGEN;
      drv_config[2].throttle_ramp_rate = D2_THROTTLE_RAMP;
      drv_config[2].regen_ramp_rate = D2_REGEN_RAMP;

      drv_config[3].max_throttle = D3_MAX_THROTTLE;
      drv_config[3].max_regen = D3_MAX_REGEN;
      drv_config[3].throttle_ramp_rate = D3_THROTTLE_RAMP;
      drv_config[3].regen_ramp_rate = D3_REGEN_RAMP;

      SET_STATS_CONF_CHANGED
    }
    else if (btn_release_select())
    {
      SET_MENU_SELECTED
    }
  }
}


/******************************************************************************
 ** Function name:  menu_errors
 **
 ** Description:    Error display screen
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_errors (void)
{
    char buffer[20];
    int len;

    _lcd_putTitle("-FAULTS-");

    len = sprintf(buffer, "ESC: 0x%02x PT0: 0x%02x", esc.error,
            (mppt[0].con_tim << 4) | (mppt[0].flags & 0xF));
    lcd_putstring(1, 0, buffer);
    PAD_ROW(1, len)

    len = sprintf(buffer, "PT1: 0x%02x PT2: 0x%02x",
            (mppt[1].con_tim << 4) | (mppt[1].flags & 0xF),
            (mppt[2].con_tim << 4) | (mppt[2].flags & 0xF));
    lcd_putstring(2, 0, buffer);
    PAD_ROW(2, len)

    len = sprintf(buffer, "BMU: 0x%04x", bmu.status);
    lcd_putstring(3, 0, buffer);
    PAD_ROW(3, len)

    if (btn_release_select() && esc.error) {
        sprintf(buffer, "RESET MOTOR CONTROLS");
        lcd_putstring(1, 0, buffer);
        lcd_putstring(2, 0, buffer);
        lcd_putstring(3, 0, buffer);

        if ((LPC_CAN1->GSR & (1 << 3))) // If previous transmission is complete, send message;
        {
            esc_reset();
            buzzer(50);
        }
    }
}

/******************************************************************************
 ** Function name:  menu_options
 **
 ** Description:    Other options on this screen.
 **                 Buzzer and driver settings
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_options (void)
{
  char buffer[20];
  int len;
  menu.submenu_items = 3;

  _lcd_putTitle("-OPTIONS-");

  if (STATS_BUZZER)
  {
    lcd_putstring(1, 2, " BUZZER: ON       ");
  }
  else
  {
    lcd_putstring(1, 2, " BUZZER: OFF      ");
  }
  len = sprintf(buffer, " DRIVER: %d", menu.driver);
  lcd_putstring(2, 2, buffer);
  PAD_ROW(2, len + 2)
  len = sprintf(buffer, " PADDLES: %d", stats.paddle_mode);
  lcd_putstring(3, 2, buffer);
  PAD_ROW(3, len + 2)

  int i;
  for (i = 0; i < 3; i++)
  {
    lcd_putstring(i + 1, 0, (menu.submenu_pos == i && (MENU_SELECTED || (clock.blink))) ? SELECTOR : DESELECTOR);
  }

  // BUTTONS
  if (btn_release_select())
  {
    if (MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
      case 0:
        ee_write(ADD_BUZZ, STATS_BUZZER);
        break;
      case 1:
        menu_init();
        break;
      }
      CLR_MENU_SELECTED;
    }
    else
    {
      SET_MENU_SELECTED;
    }
  }

  if (btn_release_increment())
  {
    if (MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
      case 0:
        TOG_STATS_BUZZER;
        break;
      case 1:
        menu.driver = (menu.driver + 1) % 4;
        break;
      case 2:
        stats.paddle_mode = (stats.paddle_mode + 1) % 3;
        break;
      default:
        break;
      }
    }
    else
    {
      menu_dec(&menu.submenu_pos, menu.submenu_items);
    }
  }

  if (btn_release_decrement())
  {
    if (MENU_SELECTED)
    {
      switch(menu.submenu_pos)
      {
      case 0:
        TOG_STATS_BUZZER;
        break;
      case 1:
        menu.driver = (menu.driver + 3) % 4;
        break;
      case 2:
        stats.paddle_mode = (stats.paddle_mode + 2) % 3;
        break;
      default:
        break;
      }
    }
    else
    {
      menu_inc(&menu.submenu_pos, menu.submenu_items);
    }
  }
}


/******************************************************************************
 ** Function name:  menu_peaks
 **
 ** Description:    Car peaks screen
 **                 1. Array power
 **                 2. Top speed
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_peaks (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-DATA PKS-");

  len = sprintf(buffer, "ARRAY: %4lu Watts", mppt[1].max_watts + mppt[2].max_watts);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "TOP SPD: %3.1f kmh", stats.max_speed);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)

  if (btn_release_select())
  {
    mppt[0].max_watts = 0;
    mppt[1].max_watts = 0;
    mppt[2].max_watts = 0;
    stats.max_speed = 0;
    buzzer(50);
  }
}


/******************************************************************************
 ** Function name:  menu_runtime
 **
 ** Description:    Displays car's current runtime
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_runtime (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-RUNTIME-");

  len = sprintf(buffer, "Counter: %lu", menu.counter);
  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "%luD %02dhr", clock.t_d, clock.t_h);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "%02dm %02d.%01ds", clock.t_m, clock.t_s, clock.t_ms/10);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)
}

/******************************************************************************
 ** Function name:  menu_odometer
 **
 ** Description:    Displays odometer
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_odometer (void)
{
  char buffer[20];
  int len;

  _lcd_putTitle("-ODOMETER-");

  len = sprintf(buffer, "CAR: %.3f km", stats.odometer);
  lcd_putstring(1, 0, buffer);
  PAD_ROW(1, len)

  len = sprintf(buffer, "ESC: %.3f km", esc.odometer/1000);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)

  len = sprintf(buffer, "TRP: %.3f km", stats.odometer_tr);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)

  if (btn_release_select())
  {
    stats.odometer_tr = 0;
    buzzer(50);
  }

  if (btn_release_inc_dec() == 3)
  {
      stats.odometer = 0;
      stats.odometer_tr = 0;
      buzzer(50);
  }
}
///////////////////////////////////////////////

///////////////////////////////////////////////
/// errors array

/******************************************************************************
 ** Function name:  menu_SWOC
 **
 ** Description:    Display screen for SWOC error
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_swoc (void) // errors[0]
{
  _lcd_putTitle("-SWOC ERR-");
  lcd_putstring(1,0, "*******ERROR!*******");
  lcd_putstring(2,0, "PRESS SELECT 2 RESET");
  lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

  // BUTTONS
  if (btn_release_select())
  {
    if ((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      esc_reset();
      buzzer(20);
    }
  }
  if (btn_release_inc_dec())
  {
    SET_STATS_SWOC_ACK;
  }
}

/******************************************************************************
 ** Function name:  menu_HWOC
 **
 ** Description:    Display screen for HWOC error
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_hwoc (void) // errors[1]
{
  _lcd_putTitle("-HWOC ERR-");
  lcd_putstring(1,0, "*******ERROR!*******");
  lcd_putstring(2,0, "PRESS SELECT 2 RESET");
  lcd_putstring(3,0, "PRESS OTHER 2 CANCEL");

  // BUTTONS
  if (btn_release_select())
  {
    if ((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      esc_reset();
    }
  }
  if (btn_release_inc_dec())
  {
    SET_STATS_HWOC_ACK;
  }
}

/******************************************************************************
 ** Function name:  menu_COMMS
 **
 ** Description:    Display screen for COMMs check
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_comms (void) // errors[2]
{
  _lcd_putTitle("-COMMS-");
  lcd_putstring(1,0, "    CHECK  COMMS    ");
  lcd_putstring(2,0, "SELECT: RADIO WORKS ");
  lcd_putstring(3,0, "OTHER:  NO RESPONSE ");

  if (btn_release_select())
  {
    if ((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      can_tx2_buf.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
      can_tx2_buf.MsgID = config.can_dash_reply + 1;
      can_tx2_buf.DataA = 0xFF;
      can_tx2_buf.DataB = 0x0;
      can1_send_message(&can_tx2_buf);
    }
    CLR_STATS_COMMS;
  }

  if (btn_release_inc_dec())
  {
    if ((LPC_CAN1->GSR & (1 << 3)))  // If previous transmission is complete, send message;
    {
      can_tx2_buf.Frame = 0x00010000;  // 11-bit, no RTR, DLC is 1 byte
      can_tx2_buf.MsgID = config.can_dash_reply + 1;
      can_tx2_buf.DataA = 0x0;
      can_tx2_buf.DataB = 0x0;
      can1_send_message(&can_tx2_buf);
    }
    CLR_STATS_COMMS;
  }
}

/******************************************************************************
 ** Function name:  menu_battery_error
 **
 ** Description:    Display when battery overheating
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_battery_error (void) // errors[3]
{
  char buffer[20];
  int len;
  _lcd_putTitle("-BAT OVT-");
  lcd_putstring(1, 0, "    **WARNING!**    ");
  len = sprintf(buffer, "TMP: %d.%d%cC", bmu.max_cell_tmp/10, bmu.max_cell_tmp%10, (char)0xD2);
  lcd_putstring(2, 0, buffer);
  PAD_ROW(2, len)
  len = sprintf(buffer, "MIN:%4dV MAX:%4dV ", bmu.cell_min_v, bmu.cell_max_v);
  lcd_putstring(3, 0, buffer);
  PAD_ROW(3, len)

  if (btn_release_select())
  {
    SET_STATS_BMU_ACK;
  }
}
///////////////////////////////////////////////

/******************************************************************************
 ** Function name:  _lcd_putTitle
 **
 ** Description:    Used to place the screen title and current car speed on
 **                 top line of LCD. Will truncate titles with more than
 **                 10 characters.
 **
 ** Parameters:     Address of char array with title string (10 character max)
 ** Returned value: None
 **
 ******************************************************************************/
void
_lcd_putTitle (char *_title)
{
  char buffer[20];
  char spd[11];
  char *bufadd;
  char *spdadd;

  bufadd = buffer;
  spdadd = spd;

  sprintf(buffer, _title);
  while ((*(++bufadd) != '\0') && (bufadd < buffer + 10))
  {
    ;
  }

  for (; bufadd != buffer + 10; bufadd++)
  {
    *bufadd = ' ';
  }

  sprintf(spd, " %5.1fkmh ", esc.velocity_kmh);

  for (; bufadd != buffer + 20; bufadd++)
  {
    *bufadd = *spdadd;
    spdadd++;
  }
  lcd_putstring(0, 0, buffer);
}

/******************************************************************************
 ** Function name:  _lcd_padding
 **
 ** Description:    Places blank chars at a location
 **
 ** Parameters:     1. Row (0-3)
 **                 2. Position (0-19)
 **                 3. Length to clear
 ** Returned value: None
 **
 ******************************************************************************/
void
_lcd_padding (int row, int pos, int len)
{
  char buffer[len];
  sprintf(buffer, "%*s", len, "");
  lcd_putstring(row, pos, buffer);
}


/******************************************************************************
 ** Function name:  _buffer_rotate_right
 **
 ** Description:    Rotates characters in buffer to the right
 **
 ** Parameters:     1. Address of buffer/string
 **                 2. Length of buffer
 ** Returned value: None
 **
 ******************************************************************************/
void
_buffer_rotate_right (char *_buf, int _len)
{
  char _last = *(_buf + _len - 1);
  char* _cur = (_buf + _len - 1);
  while (_cur != _buf)
  {
    *_cur = *(_cur - 1);
    _cur--;
  }
  *_buf = _last;
}

/******************************************************************************
 ** Function name:  _buffer_rotate_left
 **
 ** Description:    Rotates characters in buffer to the left
 **
 ** Parameters:     1. Address of buffer/string
 **                 2. Length of buffer
 ** Returned value: None
 **
 ******************************************************************************/
void
_buffer_rotate_left (char *_buf, int _len)
{
  char _first = *_buf;
  char* _cur = _buf;
  while (_cur != (_buf + _len - 1))
  {
    *_cur = *(_cur + 1);
    _cur++;
  }
  *(_buf + _len - 1) = _first;
}

/******************************************************************************
 ** Function name:  menu_inc
 **
 ** Description:    Increments menu selection by 1. Will loop to first item
 **
 ** Parameters:     1. Address of menu to increment
 **                 2. Total items in menu
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_inc(uint8_t *_pos, uint8_t _total)
{
  *_pos = (*_pos + 1) % _total;
}

/******************************************************************************
 ** Function name:  menu_dec
 **
 ** Description:    Decrements menu selection by 1. Will loop to last item
 **
 ** Parameters:     1. Address of menu to decrement
 **                 2. Total items in menu
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_dec(uint8_t *_pos, uint8_t _total)
{
  *_pos = (*_pos + _total - 1) % _total;
}

/******************************************************************************
 ** Function name:  menu_init
 **
 ** Description:    Initialize menu arrays
 **
 ** Parameters:     None
 ** Returned value: None
 **
 ******************************************************************************/
void
menu_init (void)
{
  menu.errors[0] = menu_swoc;
  menu.errors[1] = menu_hwoc;
  menu.errors[2] = menu_comms;
  menu.errors[3] = menu_battery_error;

  switch (menu.driver)
  {
  case 0: // Race
    menu.menu_items = 13;
    menu.menus[0] = menu_home;
    menu.menus[1] = menu_cruise;
    menu.menus[2] = menu_average_power;
    menu.menus[3] = menu_MPPT0;
    menu.menus[4] = menu_MPPT1;
    menu.menus[5] = menu_MPPT2;
    menu.menus[6] = menu_MPPTPower;
    menu.menus[7] = menu_motor;
    menu.menus[8] = menu_battery;
    menu.menus[9] = menu_temperature;
    menu.menus[10] = menu_options;
    menu.menus[11] = menu_runtime;
    menu.menus[12] = menu_odometer;
    break;
  case 1: // Hot Lap
    menu.menu_items = 7;
    menu.menus[0] = menu_home;
    menu.menus[1] = menu_motor;
    menu.menus[2] = menu_temperature;
    menu.menus[3] = menu_errors;
    menu.menus[4] = menu_options;
    menu.menus[5] = menu_runtime;
    menu.menus[6] = menu_odometer;
    break;
  case 2: // Test/Setup
    menu.menu_items = 20;
    menu.menus[0] = menu_home;
    menu.menus[1] = menu_controls;
    menu.menus[2] = menu_cruise;
    menu.menus[3] = menu_MPPT0;
    menu.menus[4] = menu_MPPT1;
    menu.menus[5] = menu_MPPT2;
    menu.menus[6] = menu_MPPTPower;
    menu.menus[7] = menu_motor;
    menu.menus[8] = menu_battery;
    menu.menus[9] = menu_average_power;
    menu.menus[10] = menu_temperature;
    menu.menus[11] = menu_debug;
    menu.menus[12] = menu_errors;
    menu.menus[13] = menu_options;
    menu.menus[14] = menu_config;
    menu.menus[15] = menu_peaks;
    menu.menus[16] = menu_runtime;
    menu.menus[17] = menu_odometer;
    menu.menus[18] = menu_info;
    menu.menus[19] = menu_escBus;
    break;
  case 3: // Display
    menu.menu_items = 11;
    menu.menus[0] = menu_home;
    menu.menus[1] = menu_MPPT0;
    menu.menus[2] = menu_MPPT1;
    menu.menus[3] = menu_MPPT2;
    menu.menus[4] = menu_MPPTPower;
    menu.menus[5] = menu_motor;
    menu.menus[6] = menu_battery;
    menu.menus[7] = menu_options;
    menu.menus[8] = menu_peaks;
    menu.menus[9] = menu_runtime;
    menu.menus[10] = menu_odometer;
    break;
  }

  CLR_MENU_SELECTED;
  menu.submenu_pos = 0;
  menu.menu_pos = 0; // Initial menu screen
}
