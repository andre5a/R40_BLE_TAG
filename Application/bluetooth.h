#ifndef BLUETOOTH_H__
#define BLUETOOTH_H__
#include <stdbool.h>
#include "nrf_soc.h"
#include "ble_controller_serv.h"
#include "ble_sensor_serv.h"

#include "ble_conn_params.h"
#include "state_machine.h"

//#include "safety_mode.h"
typedef enum
{
    TX_PLUS_4_dBm	=0,
    TX_0_dBm		=1,
    TX_MINUS_4_dBm	=2,
    TX_MINUS_8_dBm	=3,
    TX_MINUS_12_dBm	=4,
    TX_MINUS_16_dBm	=5,
    TX_MINUS_20_dBm	=6,
    TX_MINUS_30_dBm	=7,
 } tx_power_t;


 /**@brief Advertisement states. */
 typedef enum
 {
     BLE_NO_ADV,                                                                             /**< No advertising running. */
     BLE_FAST_ADV_WHITELIST,                                                                 /**< Advertising with whitelist. */
     BLE_FAST_ADV,                                                                           /**< Fast advertising running. */
     BLE_SLOW_ADV,                                                                           /**< Slow advertising running. */
     BLE_SLEEP,																					/**< Go to system-off. */
     BLE_BUTTON_ADV,
     BLE_SAFETY_MODE_ADV,
     BLE_CONNECTED_ADV,
     BLE_ACTIVE_SCAN_ADV,
     BLE_DIRECTED_ADV
     // BLE_ADC_ADV,
 } ble_advertising_mode_t;



#define DEFAULT_TX_POWER_TOUT					25000
#define SIGNAL_ALERT_BUTTON_ID            		0                                                 	/**< Button used for send or cancel High Alert to the peer. */
#define STOP_ALERTING_BUTTON_ID           		1                                                 	/**< Button used for clearing the Alert LED that may be blinking or turned ON because of alerts from the central. */
#define BOND_DELETE_ALL_BUTTON_ID         		1                                                 	/**< Button used for deleting all bonded centrals/services during startup. */

#ifdef _TEST_GATEWAY_
	#define APP_ADV_INTERVAL_SLOW             	MSEC_TO_UNITS((uint16_t) 4000, UNIT_0_625_MS) 		/**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
#else
		#define APP_ADV_INTERVAL_SLOW           MSEC_TO_UNITS((uint16_t) 2250, UNIT_0_625_MS) 		/**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
//#define APP_ADV_INTERVAL_SLOW             	MSEC_TO_UNITS((uint16_t) 1294, UNIT_0_625_MS) 		/**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
#endif
#define APP_SLOW_ADV_TIMEOUT_S         			0                                               /**< The duration of the slow advertising period (in seconds). */
#define APP_ADV_INTERVAL_FAST        			MSEC_TO_UNITS((uint16_t) 20, UNIT_0_625_MS)			/**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_FAST_ADV_TIMEOUT_S			       	5//2													/**< The duration of the fast advertising period (in seconds). */

//#define APP_ADV_INTERVAL_ADC             		APP_ADV_INTERVAL_SLOW 		/**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
//#define APP_ADC_ADV_TIMEOUT_S         		0                                               /**< The duration of the slow advertising period (in seconds). */


#define APP_ADV_INTERVAL_CONNECTED			MSEC_TO_UNITS((uint16_t) 3000, UNIT_0_625_MS)
#define APP_CONNECTED_ADV_TIMEOUT_S         	0                                               /**< The duration of the slow advertising period (in seconds). */

#define APP_ADV_INTERVAL_SAFETY_MODE  			MSEC_TO_UNITS((uint16_t) 20, UNIT_0_625_MS)			/**< Fast advertising interval (in units of 0.625 ms. This value corresponds to 25 ms.). */
#define APP_SAFETY_MODE_ADV_TIMEOUT_S    		6													/**< The duration of the fast advertising period (in seconds). */



#define APP_ADV_INTERVAL_BUTTON					MSEC_TO_UNITS((uint16_t) 26, UNIT_0_625_MS)
#define APP_BUTTON_ADV_TIME_S					3


#define APP_ADV_INTERVAL_ACTIVE_SCAN			MSEC_TO_UNITS((uint16_t) 500, UNIT_0_625_MS)
#define APP_ACTIVE_SCAN_ADV_DURATION_S			3


#define FIRST_CONN_PARAMS_UPDATE_DELAY  		APP_TIMER_TICKS(2500, APP_TIMER_PRESCALER)        /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   		APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)       /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    		1


 //Default connection parameters after negotiation

#define MIN_CONN_INTERVAL_DEFAULT_MS   			MSEC_TO_UNITS(500, UNIT_1_25_MS);//492//500//450//225//500// (alteracao 07-03-16) //ms /**< Minimum acceptable connection interval (0.5 seconds). iOS demands >= 20ms & <= MAX_CONN_INTERVAL - 20ms */
#define MAX_CONN_INTERVAL_DEFAULT_MS   			MSEC_TO_UNITS(1000, UNIT_1_25_MS);//595//512//595//650//325//595// (alteracao 07-03-16) //ms/**< Maximum acceptable connection interval (2 second). iOS demands <= 2s & <= CONN_SUP_TIMEOUT / 3*/


#define SLAVE_LATENCY_DEFAULT                   0//3//4//2//1  //(alteracao 12-09-16)                                         /**< Slave latency (in number of connection events that peripheral can get delayed) // By setting a non-zero slave latency, the Peripheral can choose to not answer when the Central asks for data up to the slave latency number of times.*/
#define CONN_SUP_TIMEOUT_DEFAULT               MSEC_TO_UNITS(4000, UNIT_10_MS)     //2segundos (alteracao 28-10-15)

#ifdef _TEST_GATEWAY_
#define TX_POWER_LEVEL_DEFAULT					(TX_MINUS_8_dBm)
#else
#define TX_POWER_LEVEL_DEFAULT					(TX_PLUS_4_dBm)
#endif

#define DEAD_BEEF                         		0xDEADBEEF                                        /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define DFU_REV_MAJOR                    0x00                                       /** DFU Major revision number to be exposed. */
#define DFU_REV_MINOR                    0x01                                       /** DFU Minor revision number to be exposed. */
#define DFU_REVISION                     ((DFU_REV_MAJOR << 8) | DFU_REV_MINOR)     /** DFU Revision number to be exposed. Combined of major and minor versions. */
#define APP_SERVICE_HANDLE_START         0x000C                                     /**< Handle of first application specific service when when service changed characteristic is present. */
#define BLE_HANDLE_MAX                   0xFFFF                                     /**< Max handle value in BLE. */

//STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */

#define MANUFACTURER_NAME           			"Altran"                      /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NUM                   			"029921900"                        /**< Model number. Will be passed to Device Information Service. */
#define FIRMWARE_REV							"2.4.0"
#define HARDWARE_REV							"2.0.0"


#define MAX_ACT_SCAN_ADD						5




#define DEV_NAME_SIZE							25



typedef struct
{

	//uint16_t 	max_conn_interval_default;
	uint16_t 	requested_max_conn_interval;
	//uint8_t 	requested_conn_params_change;
}CONN_PARAMS;


typedef struct
{
	bool en;
	bool subscribed;
	int8_t rssi;
}RSSI_PAR;

typedef struct
{
	bool en;
	bool subscribed;
	int16_t acel;
}ACEL_PAR;


typedef struct
{
	bool en;
	bool subscribed;
	int16_t gyro;
}GYRO_PAR;

typedef struct
{
	bool en;
	bool subscribed;
	uint16_t light;
}LIGHT_PAR;

typedef struct
{
	bool en;
	bool subscribed;
	uint16_t proximity;
}PROXIMITY_PAR;


typedef struct
{
	bool en;
	bool subscribed;
	int16_t gnd_station;
}GND_STATION_PAR;

typedef struct
{
	bool en;
	bool subscribed;
	uint32_t pressure;
}PRESSURE_PAR;

typedef struct
{
	bool en;
	bool subscribed;
	int8_t temp;
}TEMP_PAR;


typedef struct
{
	bool en;
	bool subscribed;
	uint16_t altitude;
}ALTITUDE_PAR;

typedef struct
{
	ble_advertising_mode_t 	next_mode;
	ble_advertising_mode_t 	current_mode;
	uint16_t	interval;
}ADV_PAR;

typedef struct
{
	uint8_t 	dev_name[DEV_NAME_SIZE+1];
	uint8_t 	mac_add[6];
	bool 		connected;
	ADV_PAR		adv;
	CONN_PARAMS conn_params;
	RSSI_PAR	rssi;
	ACEL_PAR	acel_x;
	ACEL_PAR	acel_y;
	ACEL_PAR	acel_z;
	GYRO_PAR	gyro_x;
	GYRO_PAR	gyro_y;
	GYRO_PAR	gyro_z;
	TEMP_PAR	temp;
	LIGHT_PAR	light;
	PROXIMITY_PAR proximity;
	GND_STATION_PAR gnd_station;
	PRESSURE_PAR pressure;
	ALTITUDE_PAR altitude;

}MY_BLE_DEV;








void process_rssi(int8_t rssi);


void advertising_start(bool erase_bonds);

void gap_params_init(void);
void advertising_init(uint8_t adv_flags);
void dis_init(void);

void services_init(void);
void conn_params_error_handler(uint32_t nrf_error);
void on_conn_params_evt(ble_conn_params_evt_t * p_evt);
void conn_params_init(void);
void gatt_init(void);



void on_ble_evt(ble_evt_t * p_ble_evt);
void on_sys_evt(uint32_t sys_evt);
void ble_evt_dispatch(ble_evt_t * p_ble_evt);
void sys_evt_dispatch(uint32_t sys_evt);
void ble_stack_init(int flag);

void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name);
void service_error_handler(uint32_t nrf_error);

void tx_power_set(tx_power_t tx_power_level);
void get_encrypted_name_for_authentication(void);
void gap_conn_params_and_tx_power_change(int16_t max_connection_interval, uint8_t tx_power_level);
void gap_conn_params_change(uint16_t max_connection_interval);
void on_controller_evt(ble_controller_t * p_beep, ble_controller_evt_t * p_evt);
void on_sensor_evt(ble_sensor_t * p_bas, ble_sensor_evt_t * p_evt);


void serv_controller_init(void);
void serv_sensor_init(void);
void advertising_set_default_params(void);
void bluetooth_init(void);
void bluetooth_read(MY_BLE_DEV *dev);
void bluetooth_write(MY_BLE_DEV dev);

int rssi_measure(int8_t rssi);

void rssi_process(int8_t rssi);

bool config_new_delta(int8_t delta);

void device_manager_init(bool erase_bonds);
void init_flash_test(void);


void flash_app_loop(void);


bool ble_act_scan_add_is_present(ble_gap_addr_t add_to_check);



#endif


