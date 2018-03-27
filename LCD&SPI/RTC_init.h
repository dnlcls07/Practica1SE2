/*
 * RTC-init.h
 *
 *  Created on: Mar 18, 2018
 *      Author: mango
 */

#ifndef RTC_INIT_H_
#define RTC_INIT_H_

void RTC_Init_t();
void GetTime_default_t();
void LCD_SetTime();
void Decodificador_LCD (uint8_t *data1, uint8_t *data2, uint8_t *data3);
void Decodificador_LCD_year (uint16_t data1, uint8_t *data2, uint8_t *data3, uint8_t *data4, uint8_t *data5);
void ConfigYear();

#endif /* RTC_INIT_H_ */
