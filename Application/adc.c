#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "app_util_platform.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
//#include "softdevice_handler.h"
#include "boards.h"
#include "ble_sensor_serv.h"
#include "nrf_assert.h"

#include "timers.h"
#include "nrf_drv_saadc.h"
#include "bluetooth.h"
#include "ble_bas.h"
#include "adc.h"
#include "app_uart.h"
#include "ble_nus.h"
#include "ble_uart.h"


BLE_BAS_DEF(m_bas);                                     /**< Battery service instance. */
static nrf_saadc_value_t adc_buf[2];



static uint16_t time_counter=0;
static volatile unsigned int adc_wr_index,adc_rd_index,adc_counter;
// This flag is set on ADC buffer overflow
static volatile unsigned char adc_buffer_overflow=0;
static uint16_t     battery_mean[NR_BATTERY_READINGS_FOR_MEAN];                                                 
static volatile uint16_t    mean_batt;
static uint16_t fifo_buffer[ADC_BUF_SIZE];

static char buf_aux[100];

uint8_t     percentage_batt_lvl;
uint16_t    batt_lvl_in_milli_volts;

static uint8_t m_batt_state=0;

uint16_t	adc_read_fifo(void)
{
	uint16_t data;
	while (!adc_counter);
	data=fifo_buffer[adc_rd_index];
	if (++adc_rd_index >= ADC_BUF_SIZE) adc_rd_index=0;
	--adc_counter;
	return data;
}

void	adc_write_fifo(uint16_t val)
{
   	fifo_buffer[adc_wr_index]=val;
   	if (++adc_wr_index >= ADC_BUF_SIZE) adc_wr_index=0;
   	if (++adc_counter >= ADC_BUF_SIZE)
    {
      	adc_counter=0;
      	adc_buffer_overflow=1;
     };

}


uint16_t	adc_used_fifo(void)
{
	return adc_counter;
}

uint16_t battery_mean_calc(void)
{
	uint16_t mean=0;
	uint16_t i;
	uint8_t mean_idx=0;


	while (adc_used_fifo()>0)
	{
		battery_mean[mean_idx]=adc_read_fifo();
		mean_idx++;
	}

	if(mean_idx<NR_BATTERY_READINGS_FOR_MEAN)
	{
		for(i=0;i<mean_idx;i++)
			mean+=battery_mean[i];
		mean/=mean_idx;
	}
	else
	{
		for(i=0;i<NR_BATTERY_READINGS_FOR_MEAN;i++)
			mean+=battery_mean[i];
		mean/=NR_BATTERY_READINGS_FOR_MEAN;
	}

return mean;
	
}




uint8_t battery_level(void)
{
	uint8_t m_battery_level = 0x21; //corresponding to 3300mv

	m_battery_level = percentage_batt_lvl;//(uint8_t) (mean_batt / 100.0);
	return m_battery_level; 
}





/**@brief Function for handling the ADC driver eevent.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
/**@brief Function for handling the ADC interrupt.
 *
 * @details  This function will fetch the conversion result from the ADC, convert the value into
 *           percentage and send it to peer.
 */
void saadc_event_handler(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        nrf_saadc_value_t adc_result;
        uint16_t          batt_lvl_in_milli_volts;
        uint8_t           percentage_batt_lvl;
        uint32_t          err_code;

        adc_result = p_event->data.done.p_buffer[0];

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, 1);
        APP_ERROR_CHECK(err_code);

        batt_lvl_in_milli_volts = ADC_RESULT_IN_MILLI_VOLTS(adc_result) +
                                  DIODE_FWD_VOLT_DROP_MILLIVOLTS;
        percentage_batt_lvl = battery_level_in_percent(batt_lvl_in_milli_volts);

        err_code = ble_bas_battery_level_update(&m_bas, percentage_batt_lvl, BLE_CONN_HANDLE_ALL);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES) &&
            (err_code != NRF_ERROR_BUSY) &&
            (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
    }
}


/**@brief Function for configuring ADC to do battery level conversion.
 */

void adc_configure(void)
{

 ret_code_t err_code = nrf_drv_saadc_init(NULL, saadc_event_handler);
    APP_ERROR_CHECK(err_code);

    nrf_saadc_channel_config_t config =
        NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_VDD);
    err_code = nrf_drv_saadc_channel_init(0, &config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[0], 1);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(&adc_buf[1], 1);
    APP_ERROR_CHECK(err_code);

}

////////////////////////////////////////////////////////////////////////////////////////









/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *          This function will start the ADC.
 *
 * @param[in]   p_context   Pointer used for passing some arbitrary information (context) from the
 *                          app_start_timer() call to the timeout handler.
 */
void battery_base_time_handler(void * p_context)
{
  UNUSED_PARAMETER(p_context);
  //05-06-15
  //AQ: fun��o nova
/*
  //03-12-2015 Não usar Stop Timer conflito no Disconnect (O 2º Start timer quando o timer já está a correr é ignorado)
	switch(m_batt_state)
	{
		case 0:
		    nrf_drv_adc_sample();
			battery_timer_start(5000);	//5Sec
			time_counter++;
			if(time_counter>=NR_BATTERY_READINGS_FOR_MEAN)
			{
				time_counter=0;
				m_batt_state=1;
			}
			break;
		case 1:
			battery_timer_start(60000);	//1Min
			time_counter++;
			if(time_counter>=480)			//8Hr
			{
			    nrf_drv_adc_sample();
				time_counter=0;
			}
			m_batt_state=1;
			break;
		default:
			m_batt_state=0;
			battery_timer_start(500);	//Start Now (500ms)
			break;
	}

	*/
}

void battery_start_read(void)
{
	time_counter=0;
	battery_timer_start(500);	//Start Now (500ms)

}

