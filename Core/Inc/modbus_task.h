/*
 * modbus_task.h
 *
 *  Created on: 2025年11月29日
 *      Author: 12543
 */

#ifndef INC_MODBUS_TASK_H_
#define INC_MODBUS_TASK_H_

void ModbusTCP_Task(void *argument);
void ModbusRTU1_Task(void *argument);
void ModbusRTU2_Task(void *argument);
void ModbusRTU1_MasterTask(void *argument);
void RS485_1_HelloTask(void *argument);
void RS485_2_HelloTask(void *argument);
void RS485_2_RxTask(void *argument);
void ModbusTCP_MasterTask(void *argument);

#endif /* INC_MODBUS_TASK_H_ */
