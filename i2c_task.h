/*
 * i2c_task.h
 *
 *  Created on: Jul 11, 2025
 *      Author: Volodymyr Marchuk
 */

#ifndef INC_I2C_TASK_H_
#define INC_I2C_TASK_H_

#include "i2c.h"
#include "RTOS_general.h"


#define SCAN_PERIOD		2000
#define TASK_PERIOD		10

#define SENSOR_OK 	0
#define SENSOR_BUSY 1
#define SENSOR_ERR 	2

typedef struct
{
	uint8_t address;
	uint8_t state;
	uint8_t nums;
	bool created;
	float* values;
	const char* format_str;
//	void (*Init_Callback) (void);
//	uint8_t (*Update_Callback) (void);
	char* name;
	char* description;

} Sensor_t;

typedef struct
{
	Sensor_t* (*Create_Sensor) (void);
	void (*Init_Callback) (Sensor_t*);
	uint8_t (*Update_Callback) (Sensor_t*);
	uint8_t (*Command_Callback) (Sensor_t*, char* name, char* args);
}Sensor_func_t;

typedef struct
{
	Sensor_func_t** func_list;
	Sensor_t** sensors;
	uint8_t channel;
	uint8_t num_sensors;
	bool sensors_active;
} Channel_t;


typedef enum
{
	HOLD = 0,
	WAIT,
	BUSY,
	SCAN,
}I2C_TASK_STATUS;

// Number of physical (multiplexed) I2C channels
#define I2C_CHANNELS		4

// Maximum length of a sensor command name and its arguments
#define SENSOR_CMD_NAME_LEN	16
#define SENSOR_CMD_ARGS_LEN	32

// Deferred sensor command, filled by the COM task and executed by the I2C task
typedef struct
{
	volatile bool ready;
	uint8_t channel;
	uint8_t address;
	char name[SENSOR_CMD_NAME_LEN];
	char args[SENSOR_CMD_ARGS_LEN];
} Sensor_Cmd_t;


void vTask_I2C(void* argument);

Sensor_t* Create_General_Sensor(uint8_t addr, const char* format_str, uint8_t nums, char* name);

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout);

void InitChannel(Channel_t* channel, uint8_t num_ch, Sensor_func_t** sensor_funcs);
void UpdateChannel(Channel_t* channel);
void ScanChannel(Channel_t* channel);

// Queue a sensor-directed command (called from the COM task).
// Returns false if a previous command has not been processed yet.
bool Sensor_Command_Enqueue(uint8_t channel, uint8_t address, const char* name, const char* args);
// Execute a pending sensor command (called from the I2C task, owns the bus/mux).
void Sensor_Command_Dispatch(void);

#endif /* INC_I2C_TASK_H_ */
