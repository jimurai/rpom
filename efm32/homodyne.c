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

#include "homodyne.h"

/* Set up the entire homodyne detection chain from a predefined configuration structure */
void hd_reset(HOMODYNE_CHANNEL* hd, LWDF_TYPE reg_init) {
	uint8_t i;
  hd->led_phase = hd->local_osc.phase_offset;
  hd->adc_phase = hd->local_osc.phase_offset;
  hd->adc_waiting =false;
  for (i=0;i<hd->filter.order;i++)
    hd->filter.registers[i] = reg_init;
}
/* LED cycle is complete so get ready for the next one */
void hd_increment_phase(HOMODYNE_CHANNEL* hd) {
  hd->adc_phase = hd->led_phase;
  if (++hd->led_phase==hd->local_osc.phase_limit) hd->led_phase=0;
}
/* ADC value is ready, but don't process it in the ISR */
void hd_adc_hold(HOMODYNE_CHANNEL* hd , int16_t adc_value) {
  // Catch over flow in an infinte loop
  // TODO: Should set a flag or send a message here
  if (hd->adc_waiting)
    while(1) {;}
  // Save the ADC value until the processor is ready to read it
  hd->adc_hold = adc_value;
  hd->adc_waiting = true;
}
/* Process the latest ADC value when outside of the ISR */
int32_t hd_adc_process(HOMODYNE_CHANNEL* hd) {
  /* Local variables */
  int16_t mix_output;
  /* Mix the input with the local oscillator */
  mix_output = hd->local_osc.local_osc[hd->adc_phase]*hd->adc_hold;
  /* Filter the signal */
  lwdf_write(&(hd->filter),mix_output);
  /* Signal that processing is finished */
  hd->adc_waiting = false;
  return (hd->filter.output[0] + hd->filter.output[1] + 1)>>1;
  
}
