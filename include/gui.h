#ifndef __GUI_H__
#define __GUI_H__

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ioe.h"

//void gui_start();
void gui_start(void *pvParameters);
void gui_play(void *pvParameters);
void next_song ();
void gui_init();
void display_normal_line(uint16_t line, char * str);
#endif