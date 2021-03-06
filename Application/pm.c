#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_soc.h"
#include "app_error.h"
#include "boards.h"
#include "uart.h"
#include "pm.h"

//static uint8_t m_dcdc=0;


/**@brief Function for the Power manager.
 */
void power_manage_loop(void)
{
	uint32_t err_code;
	/*
   	switch(m_dcdc)
	{
		case 0:
			err_code=sd_power_dcdc_mode_set(	NRF_POWER_DCDC_DISABLE	);					//Desactivar DC-DC
			APP_ERROR_CHECK(err_code);
			m_dcdc=1;
			#if (UART_EN==true)
			printf((const char *)"DCDC DISABLED\n");
			#endif
	//   	break;
		case 1:
			#if (DC_DC_EN==true)

			if(sound_led_is_busy()==false)
			{
				err_code=sd_power_dcdc_mode_set(	NRF_POWER_DCDC_ENABLE	);				//Activar DC-DC
				APP_ERROR_CHECK(err_code);
				m_dcdc=2;
				#if (UART_EN==true)
				printf((const char *)"DCDC ENABLED\n");
				#endif
			}
			#endif
			break;
		case 2:
				err_code=sd_power_dcdc_mode_set(	NRF_POWER_DCDC_DISABLE	);				//Desactivar DC-DC
				APP_ERROR_CHECK(err_code);
				m_dcdc=1;
				#if (UART_EN==true)
				printf((const char *)"DCDC DISABLED\n");
				#endif
			break;
		default:
			err_code=sd_power_dcdc_mode_set(	NRF_POWER_DCDC_DISABLE	);					//Desactivar DC-DC
			APP_ERROR_CHECK(err_code);
			m_dcdc=0;
			break;
	}
*/
    err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}

