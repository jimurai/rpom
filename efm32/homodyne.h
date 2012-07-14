/**
 * @file
 * @brief Homodyen Detection code
 *
 * @author James A. C. Patterson
 * @version 0.1
 * @date 01/07/2012
 *
 * @section LICENSE
 *
 * Free Beer License
 *
 */

#ifndef HOMODYNE_H
#define HOMODYNE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "lwdf.h"

typedef struct hd_local_oscillator {
	int8_t*  local_osc;
	uint8_t  phase_offset;
	uint8_t  phase_limit;
} HD_LO;

typedef struct homodyne_channel {
	uint8_t   led_phase;
	uint8_t   adc_phase;
  uint16_t   adc_hold;
  bool      adc_waiting;
  HD_LO     local_osc;
	LWDF_FILTER   filter;
} HOMODYNE_CHANNEL;


/* Set up the entire homodyne detection chain from a predefined configuration structure */
void hd_reset(HOMODYNE_CHANNEL* hd, LWDF_TYPE reg_init);
/* LED cycle is complete so get ready for the next one */
void hd_increment_phase(HOMODYNE_CHANNEL* hd);
/* ADC value is ready, but don't process it in the ISR */
void hd_adc_hold(HOMODYNE_CHANNEL* hd , int16_t adc_value);
/* Process the latest ADC value when outside of the ISR */
int32_t hd_adc_process(HOMODYNE_CHANNEL* hd);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif /* HOMODYNE_H */
