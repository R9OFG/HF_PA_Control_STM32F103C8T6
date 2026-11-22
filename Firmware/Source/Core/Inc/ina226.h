/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/*
 * ina226.h
 *
 * Author: r9ofg.ru
 */

#ifndef INC_INA226_H_
#define INC_INA226_H_

extern I2C_HandleTypeDef hi2c1;

#define R_SHUNT						0.001f

//адрес(7 bits) + bit R/W  1000000+0 | A1=GND A2=GND
#define INA226_I2C_ADDRESS_WRITE	0b10000000
#define INA226_I2C_ADDRESS_READ		0b10000001

#define I2C_DELAY                   10
#define INA_CONFIGURATION_REGISTER  0
#define INA_SHUNT_VOLTAGE_REGISTER  1
#define INA_BUS_VOLTAGE_REGISTER    2

#define INA_BUS_VOLTAGE_LSB         125 //LSB в uV *100 1.25mV
#define INA_SHUNT_VOLTAGE_LSB       25  //LSB в uV *10  2.5uV

#define INA_CONFIG_AVG_MASK         0x0E00
#define INA_CONFIG_BUS_TIME_MASK    0x01C0
#define INA_CONFIG_SHUNT_TIME_MASK  0x0038
#define INA_CONVERSION_READY_MASK   0x0080
#define INA_CONFIG_MODE_MASK        0x0007

#define NO_RESET_INA226 			0x4000
#define RESET_INA226    			0xC000

#define AVG_1    					0
#define AVG_4 	 					1
#define AVG_16 	 					2
#define AVG_64   					3
#define AVG_128  					4
#define AVG_256  					5
#define AVG_512  					6
#define AVG_1024 					7

#define T_Vbus_140us    			0
#define T_Vbus_204us    			1
#define T_Vbus_332us    			2
#define T_Vbus_588us    			3
#define T_Vbus_1_1ms    			4
#define T_Vbus_2_116ms  			5
#define T_Vbus_4_156ms  			6
#define T_Vbus_8_244ms  			7

#define T_Vshunt_140us    			0
#define T_Vshunt_204us    			1
#define T_Vshunt_332us    			2
#define T_Vshunt_588us    			3
#define T_Vshunt_1_1ms    			4
#define T_Vshunt_2_116ms  			5
#define T_Vshunt_4_156ms  			6
#define T_Vshunt_8_244ms  			7

#define MODE_POWER_DOWN  			   	0
#define MODE_SHUNT_VOLTAGE_TRIGGERED   	1
#define MODE_BUS_VOLTAGE_TRIGGERED     	2
#define MODE_SHUNT_BUS__TRIGGERED      	3
#define MODE_SHUNT_VOLTAGE_CONTINUOUS  	5
#define MODE_BUS_VOLTAGE_CONTINUOUS    	6
#define MODE_SHUNT_BUS_CONTINUOUS      	7

#define INA226_I2C_TIMEOUT				20	//таймаут обмена по шине I2C, мс
#define INA226_ERROR_LIMIT				20	//предел подряд идущих ошибок обмена

extern uint16_t ina226_error_cnt;			//счетчик ошибок обмена с INA226

void INA226_Init(uint16_t AVG, uint16_t Time_Vbus, uint16_t Time_Vshunt, uint16_t ModeOperation);
float INA226_Vbus(void);
float INA226_Current(void);

#endif /* INC_INA226_H_ */
