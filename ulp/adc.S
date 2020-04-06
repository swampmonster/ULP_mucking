#include "soc/rtc_cntl_reg.h"
#include "soc/soc_ulp.h"

	/* ADC1 channel 0, The channel seems to be offset by 1 */
	.set adc_channel, 1

	/* Configure the number of ADC samples to average on each measurement.
	   For convenience, make it a power of 2. */
	.set adc_oversampling_factor_log, 4
	.set adc_oversampling_factor, (1 << adc_oversampling_factor_log)

	/* Define variables, which go into .bss section (zero-initialized data) */
	.bss

	/* Counter of measurements done */
	.global sample_counter
sample_counter:
	.long 0

	.global last_result
last_result:
	.long 0

	.global wait_processing
wait_processing:
	.long 0

	/* Code goes into .text section */
	.text
	.global entry
entry:
	/* do measurements using ADC */
	/* r0 will be used as accumulator */
	move r0, 0
	/* initialize the loop counter */
	stage_rst

waiting:
	move r3, wait_processing
	ld r0, r3, 0
	jumpr waiting, 1, ge

measure:
	/* increment sample counter */
	move r3, sample_counter
	ld r2, r3, 1
	add r2, r2, 1
	st r2, r3, 0

	/* measure and add value to accumulator */
	adc r1, 0, adc_channel
	add r0, r0, r1
	/* increment loop counter and check exit condition */
	stage_inc 1
	jumps measure, adc_oversampling_factor, lt

	/* divide accumulator by adc_oversampling_factor.
	   Since it is chosen as a power of two, use right shift */
	rsh r0, r0, adc_oversampling_factor_log
	/* averaged value is now in r0; store it into last_result */
	move r3, last_result
	st r0, r3, 0

	.global exit
exit:
	/* Wake up the SoC, end program */
	wake
	halt
