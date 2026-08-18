/*
 * abp2.c
 *
 *  Created on: 17 серп. 2026 р.
 *      Author: Volodymyr Marchuk
 */


/*
 *  Honeywell ABP2 Series digital pressure sensor (I2C variant)
 *
 *  Протокол виміру (даташит, розділ 6.6.1):
 *    1. Записати 3 байти: 0xAA 0x00 0x00
 *    2. Зачекати >= 5 мс (або доки не скинеться busy-біт)
 *    3. Прочитати 4 байти: [status][press23:16][press15:8][press7:0]
 *
 *  Драйвер ніколи не блокує. Update_Callback викликається кожні
 *  TASK_PERIOD (10 мс), і за один виклик робиться рівно один крок.
 */
#include "abp2.h"
#include <string.h>
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

Sensor_func_t abp2 = {
		.Create_Sensor = ABP2_Create,
		.Init_Callback = ABP2_InitCallback,
		.Update_Callback = ABP2_UpdateCallback,
		.Command_Callback = ABP2_CommandCallback

};

typedef enum {
	ABP2_IDLE = 0,
	ABP2_WAIT_CONV

} abp2_state_t;

typedef struct {
	Sensor_t* owner;
	abp2_state_t state;
	uint32_t cmd_tick;
	int32_t last_raw_x100;
	int32_t tare_x100;
	uint8_t last_status;

}abp2_ctx_t;


static abp2_ctx_t ctx_pool[I2C_CHANNELS];

static abp2_ctx_t* ABP2_GetCtx(Sensor_t* sensor){

	for(uint8_t i=0; i<I2C_CHANNELS;i++){

		if(ctx_pool[i].owner == sensor) return &ctx_pool[i];
	}


	// перший виклик для цього екземпляра — займаємо вільний слот
	for(uint8_t i = 0; i<I2C_CHANNELS;i++){
		if(ctx_pool[i].owner == NULL){

			ctx_pool[i].owner = sensor;
			ctx_pool[i].state = ABP2_IDLE;
			ctx_pool[i].cmd_tick = 0;
			ctx_pool[i].last_raw_x100 = 0;
			ctx_pool[i].tare_x100 = 0;
			ctx_pool[i].last_status = 0;
			return &ctx_pool[i];

		}
	}

	return NULL;
}

// HELPERS

static bool ABP2_SendMesureCmd(Sensor_t* sensor){

	uint8_t cmd[3]={ABP2_CMD_MEASURE,0x00,0x00};
	return (HAL_I2C_Master_Transmit(&hi2c1,(uint16_t)(sensor->address << 1),cmd,3,ABP2_I2C_TIMEOUT_MS) == HAL_OK);
}

/*
 *
 * Рівняння 2 з даташита:
 *   P = (Output - Outmin) * (Pmax - Pmin) / (Outmax - Outmin) + Pmin
 *
 * counts до 16.7 млн, помножені на 30002 — це вилазить за int32.
 * Тому чисельник рахуємо в int64, і лише результат звужуємо назад.
 *
 */

static int32_t ABP2_CountsToMnHgX100(uint32_t counts){
	int64_t num = ((int64_t)counts - (int64_t)ABP2_OUT_MIN)*((int64_t)ABP2_PMAX_MMHG_X100 - (int64_t)ABP2_PMIN_MMHG_X100);
	int64_t result = num / ((int64_t)ABP2_OUT_MAX - (int64_t)ABP2_OUT_MIN);
	return (int32_t)(result + ABP2_PMIN_MMHG_X100);
}

//CALLBLACKS

Sensor_t* ABP2_Create(void){
	return Create_General_Sensor(ABP2_ADDR,"0x%02X:%.2f", 1, "abp2");
}

/*
 * ABP2 не має регістрів конфігурації тому просто скидаєм власний стан у відоме положення
 *
 */
void ABP2_InitCallback(Sensor_t* sensor){

	if(sensor == NULL) return;

	abp2_ctx_t* ctx = ABP2_GetCtx(sensor);

	if(ctx == NULL) return;

	ctx->state = ABP2_IDLE;
	ctx->last_raw_x100 = 0;
	ctx->last_status = 0;
	// ctx->tare_x100 = 0; перепідключення на шині НЕ має скидати калібровку нуля, зроблену оператором.
}

uint8_t ABP2_UpdateCallback(Sensor_t* sensor){

	if(sensor == NULL) return;

	abp2_ctx_t* ctx = ABP2_GetCtx(sensor);
	if(ctx == NULL) return;

	printf(sensor->format_str,sensor->address,sensor->values[0]);

	switch(ctx->state){

	case ABP2_IDLE:{
		if(!ABP2_SendMesureCmd(sensor)) return SENSOR_ERR;

		ctx->cmd_tick = HAL_GetTick();
		ctx->state = ABP2_WAIT_CONV;

		return SENSOR_BUSY;
	}
	case ABP2_WAIT_CONV:{

		uint8_t rx[4];

		// Ми в цей іф фізично не можем попасти з таск період 10мс. але нехай буде
		if((HAL_GetTick() - ctx->cmd_tick) < ABP2_CONV_TIME_MS) return SENSOR_BUSY;

		if(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)(sensor->address << 1), rx, 4, ABP2_I2C_TIMEOUT_MS) != HAL_OK){
			ctx->state = ABP2_IDLE;
			return SENSOR_ERR;
		}

		ctx->last_status = rx[0];

		if(rx[0] & ABP2_ST_BUSY) return SENSOR_BUSY;

		if(rx[0] & ABP2_ST_MEM_ERR){
			ctx->state = ABP2_IDLE;
			return SENSOR_ERR;
		}
		uint32_t counts = ((uint32_t)rx[1] << 16) | ((uint32_t)rx[2] << 8) | (uint32_t)rx[3];

		ctx->last_raw_x100 = ABP2_CountsToMnHgX100(counts);

		sensor->values[0] = (float)(ctx->last_raw_x100 - ctx->tare_x100) / 100.f;

		// Конвеєр, одразу запускаєм наступний вимір не повитаючись в IDLE. Один семпл на кожен виклик колбека, а не на кожен другий

		if(ABP2_SendMesureCmd(sensor)){

			ctx->cmd_tick = HAL_GetTick();
			//СТЕЙТ НЕ ТРОГАТЬ
		}else{
			ctx->state=ABP2_IDLE;
		}

		return SENSOR_OK;
	}
	default:{
		ctx->state = ABP2_IDLE;
		return SENSOR_ERR;
	}
	}
}

/*
 *
 * zero - поточнний тиск 0 (спущена манжета)
 * zero reset - скинути калібровку
 * status - скажи олстанній статус байт бро
 *
 *
 */

uint8_t ABP2_CommandCallback(Sensor_t* sensor, char* name, char* args){
	if((sensor == NULL)||(name == NULL)) return SENSOR_ERR;

	abp2_ctx_t* ctx = ABP2_GetCtx(sensor);

	if(ctx == NULL) return SENSOR_ERR;

	if(strcmp(name,"zero") == 0){

		if((args != NULL ) && (strcmp(args,"reset") == 0)){
			ctx->tare_x100 = 0;
			printf("ABP2: zero reset\r\n");
			return SENSOR_OK;
		}


		// Щоб упаси боже не обнулили при включеній манжеті

		if((ctx->last_raw_x100 > ABP2_TARE_LIMIT_X100) || (ctx->last_raw_x100 < -ABP2_TARE_LIMIT_X100)){

			printf("ABP2: zero refused, invalid, cuff reads %.2f mmHg\r\n", (float)ctx->last_raw_x100 / 100.0f);
			return SENSOR_OK;
		}

		if(strcmp(name,"status") == 0){
			printf("ABP2: status=0x%02X pwr=%d busy=%d mem_err=%d sat=%d\r\n",
					ctx->last_status,
					(ctx->last_status & ABP2_ST_POWERED)		? 1 : 0,
					(ctx->last_status & ABP2_ST_BUSY)			? 1 : 0,
					(ctx->last_status & ABP2_ST_MEM_ERR)		? 1 : 0,
					(ctx->last_status & ABP2_ST_SATURATED)		? 1 : 0);
			return SENSOR_OK;
		}

		printf("ABP2: unknown comand'%s'\r\n",name);
		return SENSOR_ERR;
	}

}
