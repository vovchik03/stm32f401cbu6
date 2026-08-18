/*
 * abp2.h
 *
 *  Created on: 17 серп. 2026 р.
 *      Author: Volodymyr Marchuk
 */

#ifndef INC_ABP2_H_
#define INC_ABP2_H_

#ifdef __cplusplus
extern "C" {
}
#endif

#include "i2c_task.h"
#include <stdbool.h>

#define ABP2_ADDR   0x28  // = 0x50 семибітна адреса

/*
 *  Команда виміру: 0xAA 0x00 0x00.
 * НЕ адреса регістра. У ABP2 регістрів немає взагалі.
 */
#define ABP2_CMD_MEASURE		0xAA

// Біти статус-байта (байт 0 у відповіді)
#define ABP2_ST_POWERED			(1u << 6)	/* 1 = живлення в нормі */
#define ABP2_ST_BUSY			(1u << 5)	/* 1 = дані ще не готові */
#define ABP2_ST_MEM_ERR			(1u << 2)	/* 1 = помилка цілісності пам'яті */
#define ABP2_ST_SATURATED		(1u << 0)	/* 1 = математичне насичення */

/* Передавальна функція 10 % .. 90 % від 2^24 counts.
 * Якщо у твоєї деталі варіант 30 % .. 70 % — ці числа інші! */
#define ABP2_OUT_MIN			1677722L	/* 10 % від 2^24 */
#define ABP2_OUT_MAX			15099494L	/* 90 % від 2^24 */

#define ABP2_PMAX_MMHG_X100		30002L
#define ABP2_PMIN_MMHG_X100		0L

#define ABP2_CONV_TIME_MS		5

#define ABP2_TARE_LIMIT_X100	2000L

#define ABP2_I2C_TIMEOUT_MS		5

Sensor_t* ABP2_Create(void);
void      ABP2_InitCallback(Sensor_t* sensor);
uint8_t   ABP2_UpdateCallback(Sensor_t* sensor);
uint8_t   ABP2_CommandCallback(Sensor_t* sensor, char* name, char* args);

#endif /* INC_ABP2_H_ */
