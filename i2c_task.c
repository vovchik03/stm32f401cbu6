/*
 * i2c_task.c
 *
 *  Created on: Jul 11, 2025
 *      Author: Volodymyr Marchuk
 */

#include "i2c_task.h"
#include "TCA9548.h"

extern Sensor_func_t abp2;

Sensor_func_t* sensor_funcs[] =
		{&abp2, NULL};

I2C_TASK_STATUS curr_status = WAIT;

i2c_mux_t I2C_MUX =
{
	.addr_offset = 0,
	.hi2c = &hi2c1,
	.rst_pin = MUX_Pin,
	.rst_port = MUX_GPIO_Port
};

Channel_t channels[5];

Sensor_Cmd_t sensor_cmd = { .ready = 0 };

void vTask_I2C(void* argument)
{
	HAL_I2C_Init(&hi2c1);

	uint16_t time = SCAN_PERIOD / TASK_PERIOD;

	for (uint8_t i = 0; i < 4; i++)
		InitChannel(&channels[i], i, &sensor_funcs);

	i2c_mux_reset(&I2C_MUX);

	i2c_mux_select(&I2C_MUX, 1);

	while(1)
	{

		Sensor_Command_Dispatch();

		switch(curr_status)
		{
		case HOLD:
			break;
		case WAIT:
			for (uint8_t i = 0; i < 4; i++)
			{
				UpdateChannel(&channels[i]);
			}

			break;
		case BUSY:
			break;
		case SCAN:
//			printf("Res: %d\r\n", res);
//			I2C_Scan_Bus(&hi2c1, sensors);

			for (uint8_t i = 0; i < 4; i++)
			{
				ScanChannel(&channels[i]);
			}
			I2C_ClearBusyFlagErratum(&hi2c1, 10);

			curr_status = WAIT;
			break;
		default:

			break;
		}

		if (time)
		{
			time--;
		}
		else
		{
			curr_status = SCAN;
			time = SCAN_PERIOD / TASK_PERIOD;
		}

		vTaskDelay(TASK_PERIOD);
	}
}


Sensor_t* Create_General_Sensor(uint8_t addr, const char* format_str, uint8_t nums, char* name)
{
	if(format_str == NULL)
		return NULL;

	Sensor_t* local_sensor = (Sensor_t*) calloc(1, sizeof(Sensor_t));

	if (local_sensor == NULL)
		return NULL;

	local_sensor->address = addr;
	local_sensor->created = 1;
	local_sensor->format_str = format_str;
	local_sensor->name = name;
	local_sensor->nums = nums;

	float* local_float = (float*) calloc(nums, sizeof(float));

	if (local_float == NULL)
	{
		free(local_sensor);
		return NULL;
	}

	local_sensor->values = local_float;

	return local_sensor;
}

bool Sensor_Command_Enqueue(uint8_t channel, uint8_t address, const char* name, const char* args)
{
	if ((name == NULL) || (args == NULL))
		return false;

	// Previous command still waiting to be processed by the I2C task
	if (sensor_cmd.ready)
		return false;

	sensor_cmd.channel = channel;
	sensor_cmd.address = address;

	strncpy(sensor_cmd.name, name, SENSOR_CMD_NAME_LEN - 1);
	sensor_cmd.name[SENSOR_CMD_NAME_LEN - 1] = 0;

	strncpy(sensor_cmd.args, args, SENSOR_CMD_ARGS_LEN - 1);
	sensor_cmd.args[SENSOR_CMD_ARGS_LEN - 1] = 0;

	sensor_cmd.ready = 1;	// set last, consumed by the I2C task
	return true;
}

void Sensor_Command_Dispatch(void)
{
	if (!sensor_cmd.ready)
		return;

	uint8_t ch = sensor_cmd.channel;

	if (ch >= I2C_CHANNELS)
	{
		printf("Wrong channel: %u\r\n", ch);
		sensor_cmd.ready = 0;
		return;
	}

	Channel_t* channel = &channels[ch];

	i2c_mux_select(&I2C_MUX, channel->channel);

	for (uint8_t i = 0; i < 5; i++)
		__ASM("nop");

	bool found = 0;
	for (uint8_t i = 0; i < channel->num_sensors; i++)
	{
		if (channel->sensors[i]->address == sensor_cmd.address)
		{
			found = 1;
			if (channel->func_list[i]->Command_Callback != NULL)
				channel->func_list[i]->Command_Callback(channel->sensors[i], sensor_cmd.name, sensor_cmd.args);
			else
				printf("Sensor 0x%02X: commands not supported\r\n", sensor_cmd.address);
			break;
		}
	}

	if (!found)
		printf("Sensor 0x%02X not found on Ch%u\r\n", sensor_cmd.address, ch);

	sensor_cmd.ready = 0;
}

void InitChannel(Channel_t* channel, uint8_t num_ch, Sensor_func_t** sensor_funcs)
{
	if(channel == NULL)
		return;

	if(sensor_funcs == NULL)
		return;

	channel->func_list = sensor_funcs;
	channel->channel = num_ch;
	channel->sensors_active = 0;

	uint8_t sensors_num = 0;
	for (; sensor_funcs[sensors_num] != NULL; sensors_num++);

	channel->num_sensors = sensors_num;

	Sensor_t** local_ptr = (Sensor_t**) calloc(sensors_num, sizeof(Sensor_t*));

	if (local_ptr == NULL)
		return;

	for(uint8_t i = 0; i < sensors_num; i++)
	{
		local_ptr[i] = sensor_funcs[i]->Create_Sensor();
	}

	channel->sensors = local_ptr;
}

void UpdateChannel(Channel_t* channel)
{
	if(channel == NULL)
		return;

	i2c_mux_select(&I2C_MUX, channel->channel);

	for(uint8_t i = 0; i < 5; i++)
		__ASM("nop");

	bool first_data = 0;
	for (uint8_t i = 0; i < channel->num_sensors; i++)
	{
		uint8_t res = 0;
		if (channel->sensors[i]->state == 1)
		{
			if (first_data)
				printf(",");
			else
			{
				first_data = 1;
				printf("Ch%d# ", channel->channel);
			}
			res = channel->func_list[i]->Update_Callback(channel->sensors[i]);
		}
			for (uint8_t i = 0; i < channel->num_sensors; i++)
			{
				uint8_t res = 0;
				if (channel->sensors[i]->state == 1)
				{
					if (first_data)
						printf(",");
					else
					{
						first_data = 1;
						printf("Ch%d# ", channel->channel);
					}
					res = channel->func_list[i]->Update_Callback(channel->sensors[i]);
				}
			}

			if (first_data)
				printf("\r\n");
	for(uint8_t i = 0; i < 5; i++)
		__ASM("nop");

	for (uint8_t i = 0; i < channel->num_sensors; i++)
	{
		if (HAL_I2C_IsDeviceReady(&hi2c1, (channel->sensors[i]->address << 1), 1, 1) == HAL_OK)
		{
			channel->sensors_active = 1;
			 if (channel->sensors[i]->state == 0)
			 {
				 channel->sensors[i]->state = 1;
				 if (channel->func_list[i]->Init_Callback != NULL )
					 channel->func_list[i]->Init_Callback(channel->sensors[i]);
			 }
		}
		else
		{
			channel->sensors[i]->state = 0;
		}
	}
	}
}

void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c, uint32_t timeout)
{
	HAL_I2C_DeInit(hi2c);
	__ASM("nop");
    // Call initialization function.
    HAL_I2C_Init(hi2c);
}
