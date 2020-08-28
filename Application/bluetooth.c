#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include "mainfile.h"
#include "boards.h"
#include "ble_dfu.h"
//#include "dfu_app_handler.h"
#include "app_uart.h"
#include "ble_nus.h"
#include "ble_uart.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
//#include "nrf51_bitfields.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "timers.h"
#include "ble_controller_serv.h"
#include "ble_sensor_serv.h"
#include "ble_dis.h"
#include "sdk_config.h"
#include "ble_conn_params.h"
#include "ble_dfu.h"
#include "ble_advertising.h"
//#include "ble_radio_notification.h"
#include "app_util_platform.h"
#include "button.h"
//#include "nrf_adc.h"
#include "state_machine.h"
#include "bluetooth.h"
#include "nrf_delay.h"
#include "adc.h"
//#include "nrf_pwm.h"
#include "ble_bas.h"
#include "ble_advertising.h"
#include "nrf_pwr_mgmt.h"
#include "peer_manager_types.h"
#include "ble_db_discovery.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "fds.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "ble_tps.h"
#include "ble_ias.h"
#include "ble_lls.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "ble_ias_c.h"
#include "app_util.h"


#define SEC_PARAM_BOND                  1                                            /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                            /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                         /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                            /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                            /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                           /**< Maximum encryption key size. */

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific).*/


#define IS_SRVC_CHANGED_CHARACT_PRESENT  1                                          /**< Include or not the service_changed characteristic. if not enabled, the server's database cannot be changed for the lifetime of the device*/

STATIC_ASSERT(IS_SRVC_CHANGED_CHARACT_PRESENT);                                     /** When having DFU Service support in application the Service Changed Characteristic should always be present. */


 #define CENTRAL_LINK_COUNT              0                                 /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
 #define PERIPHERAL_LINK_COUNT           1                                 /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/


#define TIMER_RSSI_TIME_BASE	100									//ms

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */




#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.5 seconds).  */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)       /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)   

#define APP_ADV_INTERVAL                40  
#define APP_ADV_DURATION                18000      

static char buf_aux[100];
static MY_BLE_DEV 						my_ble;

//static dm_application_instance_t        m_app_handle;                                /**< Application identifier allocated by device manager */

//static bool                            	m_memory_access_in_progress = false;              /**< Flag to keep track of ongoing operations on persistent memory. */

//static int8_t raw_rssi=0;
//static ble_dfu_t                         m_dfus;                                    /**< Structure used to identify the DFU service. */

static ble_bas_t                         m_bas;                                     /**< Structure used to identify the battery service. */


//static ble_advertising_mode_t adv_pendent_mode=BLE_SLOW_ADV;

static ble_uuid_t m_adv_uuids[] =                       /**< Universally unique service identifiers. */
{
    {BLE_UUID_IMMEDIATE_ALERT_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_TX_POWER_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_LINK_LOSS_SERVICE, BLE_UUID_TYPE_BLE}
};

static ble_gap_addr_t addr_connected;



#ifdef _ADV_WHITE_LIST_
static dm_handle_t                      m_bonded_peer_handle;                       /**< Device reference handle to the current connected peer. */
#endif
static uint8_t                           m_direct_adv_cnt;                              /**< Counter of direct advertisements. */


NRF_BLE_GATT_DEF(m_gatt);                               /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising);                     /**< Advertising module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                 /**< Context for the Queued Write module.*/
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);               /**< DB discovery module instance. */




/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
void battery_perc_update(unsigned char val)
{
    uint32_t err_code;
/*    if(my_ble.connected)
    {
	sprintf(buf_aux,"BAT_VAL: %u",val);
	nus_printStr(buf_aux);
    }
    err_code = ble_bas_battery_level_update(&m_bas,val);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != BLE_ERROR_NO_TX_PACKETS) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
        )
    {
        APP_ERROR_HANDLER(err_code);
    }
*/
}

static void radio_notification_init(void)
{
    uint32_t err_code;

//    err_code = ble_radio_notification_init(APP_IRQ_PRIORITY_HIGHEST,
//                                           NRF_RADIO_NOTIFICATION_DISTANCE_4560US,
//                                           ble_flash_on_radio_active_evt);
//    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}



/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_DISCONNECTED:

            break;

        case BLE_GAP_EVT_CONNECTED:
        {
            // Assign connection handle to the Queued Write module.
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            // Discover peer's services.
            err_code = ble_db_discovery_start(&m_ble_db_discovery, p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
          //  NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
           // NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
         //   NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Beep events.
 *
 * @details This function will be called for all Immediate Alert events which are passed to the
 *          application.
 *
 * @param[in]   p_beep  Beep structure.
 * @param[in]   p_evt  Event received from the Beep service.
 */
void on_controller_evt(ble_controller_t * p_controller, ble_controller_evt_t * p_evt)
{
/*	uint32_t err_code;
	//uint8_t i=0;
	uint32_t gprepret_value;
	switch(p_evt->evt_type)
	{
		case PWM1_VAL:
			sprintf(buf_aux,"PWM1_VAL: %u",p_evt->pwm1_val);
			nus_printStr(buf_aux);
			if(p_evt->pwm1_val==0x9D)
			{

				//activate GPREGRET register for activating bootloader after system reset
				//err_code = sd_power_gpregret_clr(0xFF);
				//APP_ERROR_CHECK(err_code);
				err_code = sd_power_gpregret_set(0xB1);
				APP_ERROR_CHECK(err_code);
				//err_code = sd_power_gpregret_get(&gprepret_value); //Always 3
				//APP_ERROR_CHECK(err_code);
					//system reset
					sd_nvic_SystemReset();

				 // NRF_POWER->GPREGRET = 0xB1;

				   // NVIC_SystemReset();
			}
		    //timer_pwm_off_stop();
		    //timer_pwm_off_start(5000);
			break;
		case PWM2_VAL:
			sprintf(buf_aux,"PWM2_VAL: %u",p_evt->pwm2_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);
			break;
		case PWM3_VAL:

			sprintf(buf_aux,"PWM3_VAL: %u",p_evt->pwm3_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);

			break;
		case PWM4_VAL:
			sprintf(buf_aux,"PWM4_VAL: %u",p_evt->pwm4_val);
			nus_printStr(buf_aux);
		    timer_pwm_off_stop();
		    timer_pwm_off_start(5000);

			break;
		default:
			break;
	}
*/
}


/**@brief Function for handling the Battery Service events.
 *
 * @details This function will be called for all Battery Service events which are passed to the
 |          application.
 *
 * @param[in]   p_bas  Battery Service structure.
 * @param[in]   p_evt  Event received from the Battery Service.
 */

void on_sensor_evt(ble_sensor_t * p_extra, ble_sensor_evt_t *p_evt)
{
/*	//timer_fake_sensors_start(1000);


		switch (p_evt->evt_type)
		{
			case CHAR_ACEL_X_SUBSCRIBED:
				nus_printStr("ACEL_X_SUB");
				my_ble.acel_x.subscribed=true;
				break;
			case CHAR_ACEL_X_UNSUBSCRIBED:
				nus_printStr("ACEL_X_UNSUB");
				my_ble.acel_x.subscribed=false;

				break;
			case CHAR_ACEL_Y_SUBSCRIBED:
				nus_printStr("ACEL_Y_SUB");
				my_ble.acel_y.subscribed=true;

				break;
			case CHAR_ACEL_Y_UNSUBSCRIBED:
				nus_printStr("ACEL_Y_UNSUB");
				my_ble.acel_y.subscribed=false;

				break;
			case CHAR_ACEL_Z_SUBSCRIBED:
				my_ble.acel_z.subscribed=true;
				nus_printStr("ACEL_Z_SUB");

				break;
			case CHAR_ACEL_Z_UNSUBSCRIBED:
				my_ble.acel_z.subscribed=false;
				nus_printStr("ACEL_X_UNSUB");

				break;

			case CHAR_GYRO_X_SUBSCRIBED:
				my_ble.gyro_x.subscribed=true;
				nus_printStr("GYRO_X_SUB");

				break;
			case CHAR_GYRO_X_UNSUBSCRIBED:
				my_ble.gyro_x.subscribed=false;
				nus_printStr("GYRO_X_UNSUB");

				break;
			case CHAR_GYRO_Y_SUBSCRIBED:
				my_ble.gyro_y.subscribed=true;
				nus_printStr("GYRO_Y_SUB");

				break;
			case CHAR_GYRO_Y_UNSUBSCRIBED:
				my_ble.gyro_y.subscribed=false;
				nus_printStr("GYRO_Y_UNSUB");

				break;
			case CHAR_GYRO_Z_SUBSCRIBED:
				my_ble.gyro_z.subscribed=true;
				nus_printStr("GYRO_Z_SUB");

				break;
			case CHAR_GYRO_Z_UNSUBSCRIBED:
				my_ble.gyro_z.subscribed=false;
				nus_printStr("GYRO_Z_UNSUB");

				break;

			case CHAR_LIGHT_SUBSCRIBED:
				my_ble.light.subscribed=true;
			nus_printStr("LIGHT_SUB");

				break;
			case CHAR_LIGHT_UNSUBSCRIBED:
				my_ble.light.subscribed=false;
				nus_printStr("LIGHT_UNSUB");

				break;

			case CHAR_GND_STATION_SUBSCRIBED:
				nus_printStr("GND_STATION_SUB");
				my_ble.gnd_station.subscribed=true;
			break;
			case CHAR_GND_STATION_UNSUBSCRIBED:
				nus_printStr("GND_STATION_UNSUB");
				my_ble.gnd_station.subscribed=false;

				break;

			case CHAR_PRESSURE_SUBSCRIBED:
				nus_printStr("PRESSURE_SUB");
				my_ble.pressure.subscribed=true;
			break;
			case CHAR_PRESSURE_UNSUBSCRIBED:
				nus_printStr("PRESSURE_UNSUB");
				my_ble.pressure.subscribed=false;

				break;
			case CHAR_TEMP_SUBSCRIBED:
				nus_printStr("TEMP_SUB");
				my_ble.temp.subscribed=true;
			break;
			case CHAR_TEMP_UNSUBSCRIBED:
				nus_printStr("TEMP_UNSUB");
				my_ble.temp.subscribed=false;

				break;
			case CHAR_ALTITUDE_SUBSCRIBED:
				nus_printStr("ALTITUDE_SUB");
				my_ble.altitude.subscribed=true;
			break;
			case CHAR_ALTITUDE_UNSUBSCRIBED:
				nus_printStr("ALTITUDE_UNSUB");
				my_ble.altitude.subscribed=false;

				break;

			default:
				// No implementation needed.
			break;
		}

*/
}





void serv_controller_init(void)
{
	/*
	uint32_t       			err_code;
    ble_controller_init_t 	controller_init_obj;
    memset(&controller_init_obj, 0, sizeof(controller_init_obj));


    controller_init_obj.evt_handler 			= on_controller_evt;
    controller_init_obj.support_notification 	= true;
    controller_init_obj.p_report_ref       		= NULL;
    err_code = ble_controller_serv_init(&m_controller, &controller_init_obj);
    APP_ERROR_CHECK(err_code);

*/
}







void serv_sensor_init(void)
{
	/*
    uint32_t       err_code;
    ble_sensor_init_t sensor_init_obj;

    memset(&sensor_init_obj, 0, sizeof(sensor_init_obj));
    sensor_init_obj.evt_handler          = on_sensor_evt;

	sensor_init_obj.initial_acel_x_level=0;
	sensor_init_obj.initial_acel_y_level=0;
	sensor_init_obj.initial_acel_z_level=0;

	sensor_init_obj.initial_gyro_x_level=0;
	sensor_init_obj.initial_gyro_y_level=0;
	sensor_init_obj.initial_gyro_z_level=0;
	sensor_init_obj.initial_temp_level=0;
	sensor_init_obj.initial_light_level=0;


	sensor_init_obj.initial_ground_station_level=0;
	sensor_init_obj.initial_pressure_level=0;
	sensor_init_obj.initial_altitude_level=0;

	sensor_init_obj.initial_batt_level=0;

	sensor_init_obj.support_notification=true;
	sensor_init_obj.p_report_ref       	= NULL;




    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_x_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_x_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_y_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_y_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.acel_z_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.acel_z_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_x_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_x_char_attr_md.write_perm);


    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_y_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_y_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.gyro_z_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.gyro_z_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.temp_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.temp_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.light_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.light_char_attr_md.write_perm);



    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.ground_station_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.ground_station_char_attr_md.write_perm);

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.pressure_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.pressure_char_attr_md.write_perm);



    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sensor_init_obj.altitude_report_read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&sensor_init_obj.altitude_char_attr_md.write_perm);


    err_code = ble_sensor_serv_init(&m_sensor, &sensor_init_obj);
    APP_ERROR_CHECK(err_code);
*/
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
#define DEVICE_NAME                     "Nordic_Prox" 
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);


	sprintf((char *)my_ble.dev_name,"P%02x%02x",my_ble.mac_add[5],my_ble.mac_add[4]);
    err_code = sd_ble_gap_device_name_set(&sec_mode,(const uint8_t *)my_ble.dev_name,strlen((const char *)my_ble.dev_name));
    APP_ERROR_CHECK(err_code);
//////////////////////////////////////////////////////////////////////////////////////
    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_KEYRING);
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL_DEFAULT_MS;//MIN_CONN_INTERVAL_DEFAULT;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL_DEFAULT_MS;//MAX_CONN_INTERVAL_DEFAULT;
    gap_conn_params.slave_latency     = SLAVE_LATENCY_DEFAULT;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT_DEFAULT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
	//if(err_code == NRF_SUCCESS)
	//	ble_extra_max_conn_interval_default_set(&m_sensor, MAX_CONN_INTERVAL_DEFAULT_MS);
    tx_power_set(TX_POWER_LEVEL_DEFAULT);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}
/**@brief Function for initializing the Connection Parameters module.
 */
void conn_params_init(void)
{
    
uint32_t               err_code;
/*    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;

	cp_init.evt_handler                    = NULL;
	cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
*/
}




/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
	//dm_ble_evt_handler(p_ble_evt);
    ble_bas_on_ble_evt(&m_bas, p_ble_evt);
    ble_nus_on_ble_evt(&m_nus, p_ble_evt);

    ble_conn_params_on_ble_evt(p_ble_evt);

 //   ble_sensor_on_ble_evt(&m_sensor, p_ble_evt);
	ble_controller_on_ble_evt(&m_controller, p_ble_evt);

    /** @snippet [Propagating BLE Stack events to DFU Service] */
  //  ble_dfu_on_ble_evt(&m_dfus, p_ble_evt);
    /** @snippet [Propagating BLE Stack events to DFU Service] */


	on_ble_evt(p_ble_evt);
 //   ble_advertising_on_ble_evt(p_ble_evt);
}


/**@brief Function for dispatching a system event to interested modules.
 *
 * @details This function is called from the System event interrupt handler after a system
 *          event has been received.
 *
 * @param[in]   sys_evt   System stack event.
 */

void sys_evt_dispatch(uint32_t sys_evt)
{
	//pstorage_sys_event_handler(sys_evt); //Add this line
     ble_advertising_on_sys_evt(sys_evt);
	 on_sys_evt(sys_evt);

}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
void ble_stack_init(int flag)
{
	ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}



/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Service errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


void timer_poff_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);



}



void tx_power_set(tx_power_t tx_power_level)
{
	const int8_t 				TX_POWER_LIST[8] 			= {4, 0, -4, -8, -12, -16, -20, -30};
	int8_t tx_power;
	switch(tx_power_level)
	{
	case TX_PLUS_4_dBm:
		tx_power = TX_POWER_LIST[TX_PLUS_4_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: +4dBm\n");
		#endif
		break;
	case TX_0_dBm:
		tx_power = TX_POWER_LIST[TX_0_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: 0dBm\n");
		#endif
		break;
	case TX_MINUS_4_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_4_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -4dBm\n");
		#endif
		break;
	case  TX_MINUS_8_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_8_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -8dBm\n");
		#endif
		break;
	case TX_MINUS_12_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_12_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -12dBm\n");
		#endif
		break;
	case TX_MINUS_16_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_16_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -16dBm\n");
		#endif
		break;
	case TX_MINUS_20_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_20_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -20dBm\n");
		#endif
		break;
	case TX_MINUS_30_dBm:
		tx_power = TX_POWER_LIST[TX_MINUS_30_dBm];
		#ifdef _UART_ENABLE_
		printf((const char *)"TX PWR: -30dBm\n");
		#endif
		break;
	default:
		#ifdef _UART_ENABLE_
		printf((const char *)"ERROR: TX PWR fail\n");
		#endif
		tx_power = TX_POWER_LIST[TX_MINUS_30_dBm];
			break;
	}
/*	uint32_t err_code = sd_ble_gap_tx_power_set(tx_power);
	APP_ERROR_CHECK(err_code);
	if(err_code == NRF_SUCCESS)
	{
		//ble_extra_tx_power_level_set(&m_sensor, tx_power_level);
		//ble_extra_connection_mode_set(&m_sensor, (my_ble.conn_params.requested_max_conn_interval << 16 | tx_power_level << 0));
	}
*/


}




void gap_conn_params_and_tx_power_change(int16_t max_connection_interval, uint8_t tx_power_level)
{
	gap_conn_params_change(max_connection_interval);
	tx_power_set(tx_power_level);
}



/**@brief Function for the GAP Connection Parameters Change.
 *
 * @details This function is used to change the connection parameters
 *
 */
void gap_conn_params_change(uint16_t max_connection_interval)
{
	ble_gap_conn_params_t  updated_cnxn_param;
	uint32_t               err_code;
	
	updated_cnxn_param.min_conn_interval =  MSEC_TO_UNITS((uint16_t)(((uint16_t)max_connection_interval)-((uint16_t)25)), UNIT_1_25_MS);
	updated_cnxn_param.max_conn_interval =  MSEC_TO_UNITS((uint16_t) max_connection_interval, UNIT_1_25_MS);
	updated_cnxn_param.slave_latency     =  SLAVE_LATENCY_DEFAULT;
	updated_cnxn_param.conn_sup_timeout  =  CONN_SUP_TIMEOUT_DEFAULT;
		
	// Initialize and set-up connection parameter negotiation module
	// if this method is called upon connection it will immediately slow down
/*	err_code = ble_conn_params_change_conn_params(&updated_cnxn_param);
	if(err_code == NRF_SUCCESS)
	{
		// Procedure request succeeded. Connection parameters will be negotiated as requested.
		// BLE_CONN_PARAMS_EVT_SUCCEEDED will be notified if parameter negotiation is successful.
		//my_ble.conn_params.requested_conn_params_change = true;
		my_ble.conn_params.requested_max_conn_interval = max_connection_interval;
	}
	else
	{
			// Procedure request failed.
	}
*/
}

/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            //NRF_LOG_INFO("Fast advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_IDLE:
           // sleep_mode_enter();
            break; // BLE_ADV_EVT_IDLE

        default:
            break;
    }
}


/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 *
 * @param[in]  adv_flags  Indicates which type of advertisement to use, see @ref BLE_GAP_DISC_MODES.
 *
 */
void advertising_init(uint8_t adv_flags)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);

}





void dis_init(void)
{
	uint32_t       err_code;
/*	ble_dis_init_t   dis_init;
		
    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));
    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, MANUFACTURER_NAME);
    ble_srv_ascii_to_utf8(&dis_init.model_num_str,     MODEL_NUM);
    ble_srv_ascii_to_utf8(&dis_init.fw_rev_str,     	 FIRMWARE_REV);
    ble_srv_ascii_to_utf8(&dis_init.hw_rev_str,     	 HARDWARE_REV);	
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&dis_init.dis_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&dis_init.dis_attr_md.write_perm);
    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
*/
}
/**@brief Function for initializing the services that will be used by the application.
 */
void services_init(void)
{
	uint32_t       err_code;
	ble_bas_init_t bas_init;

	dis_init();

//	nus_services_init();
/*
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.cccd_write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_char_attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&bas_init.battery_level_char_attr_md.write_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&bas_init.battery_level_report_read_perm);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
*/

/*
    ble_dfu_init_t   dfus_init;

    // Initialize the Device Firmware Update Service.
    memset(&dfus_init, 0, sizeof(dfus_init));

    dfus_init.error_handler = NULL;
    dfus_init.evt_handler   = dfu_app_on_dfu_evt;
    dfus_init.revision      = DFU_REVISION;

    err_code = ble_dfu_init(&m_dfus, &dfus_init);
    APP_ERROR_CHECK(err_code);

    dfu_app_reset_prepare_set(reset_prepare);
    dfu_app_dm_appl_instance_set(m_app_handle);
*/

	serv_controller_init();
//	serv_sensor_init();

}

void bluetooth_init(void)
{
	my_ble.connected=false;
	ble_stack_init(1);	//Bluetooth Stack Init
	//radio_notification_init();
}







void bluetooth_read(MY_BLE_DEV *dev)
{
	memcpy(dev,&my_ble,sizeof(MY_BLE_DEV));
}



void on_sys_evt(uint32_t sys_evt)
{
    switch(sys_evt)
    {
 /*       case NRF_EVT_FLASH_OPERATION_SUCCESS:
        case NRF_EVT_FLASH_OPERATION_ERROR:
			#ifdef _DEVICE_MANAGER_EN_
        	if (m_memory_access_in_progress)
            {
				#ifdef _UART_ENABLE_
				printf("NRF_EVT_FLASH_OPERATION!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\r\n");
				#endif
                m_memory_access_in_progress = false;
                advertising_start(adv_pendent_mode);	//Recupera o advertise que não executou enquanto escrevia na flash
            }
			#endif
        	break;
*/
        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true){
       // delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCEEDED event.
    }
    else
    {
        uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);

        APP_ERROR_CHECK(err_code);
    }
}

void battery_adv_update(void)
{
	if(my_ble.connected==false)
	{
		advertising_start(BLE_NO_ADV);		//Advertise Stop
		advertising_start(BLE_SLOW_ADV);	//Advertise re-start in SLOW ADV
	}
}

void button_adv_set(void)
{
	if(my_ble.connected==false)
	{
		advertising_start(BLE_NO_ADV);
		advertising_start(BLE_BUTTON_ADV);
	}
}





/**@brief Function for handling the Device Manager events.
 *
 * @param[in] p_evt  Data associated to the device manager event.
 */
/*
static uint32_t device_manager_evt_handler(dm_handle_t const * p_handle,
                                           dm_event_t const  * p_event,
                                           ret_code_t        event_result)
{
    APP_ERROR_CHECK(event_result);

    switch (p_event->event_id)
    {
        case DM_EVT_DEVICE_CONTEXT_LOADED: // Fall through.
        case DM_EVT_SECURITY_SETUP_COMPLETE:
           // m_bonded_peer_handle = (*p_handle);
            break;
        case DM_EVT_LINK_SECURED:
        	app_context_load(p_handle);
        	break;
    }
    return NRF_SUCCESS;



}

*/
/**@brief Function for the Device Manager initialization.
 *
 * @param[in] erase_bonds  Indicates whether bonding information should be cleared from
 *                         persistent storage during initialization of the Device Manager.
 */
/*
void device_manager_init(bool erase_bonds)
{

    uint32_t               err_code;
    dm_init_param_t        init_param = {.clear_persistent_data = erase_bonds};
    dm_application_param_t register_param;


    // Initialize persistent storage module.
    err_code = pstorage_init();
    APP_ERROR_CHECK(err_code);

    err_code = dm_init(&init_param);
    APP_ERROR_CHECK(err_code);

    memset(&register_param.sec_param, 0, sizeof(ble_gap_sec_params_t));
    register_param.sec_param.bond         = SEC_PARAM_BOND;
    register_param.sec_param.mitm         = SEC_PARAM_MITM;
    register_param.sec_param.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    register_param.sec_param.oob          = SEC_PARAM_OOB;
    register_param.sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    register_param.sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
    register_param.evt_handler            = device_manager_evt_handler;
    register_param.service_type           = DM_PROTOCOL_CNTXT_GATT_SRVR_ID;

    err_code = dm_register(&m_app_handle, &register_param);
    APP_ERROR_CHECK(err_code);

}

*/

void timer_fake_sensors_handler(void * p_context)
{






}


void timer_pwm_off_handler(void * p_context)
{
/*
    nrf_pwm_set_value(0, 0);
	nrf_pwm_set_enabled(false);
    apply_pan73_workaround(PWM_TIMER, false);
    PWM_TIMER->TASKS_STOP = 1;
*/
}


void timer_bmp180_handler(void * p_context)
{



}




/**@brief Function for initializing the GATT module.
 */
void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}
