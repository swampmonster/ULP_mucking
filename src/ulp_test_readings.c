#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/rtc_cntl.h"
#include "driver/adc.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void IRAM_ATTR wk(void *arg) {
    SemaphoreHandle_t done = (SemaphoreHandle_t) arg;
    ulp_wait_processing = 1;
    xSemaphoreGiveFromISR(done, pdFALSE);
}

void app_main(void) {
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
            (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 20000);

    SemaphoreHandle_t ulp_isr_sem = xSemaphoreCreateBinary();
    assert(ulp_isr_sem);

    err = rtc_isr_register(&wk, (void*) ulp_isr_sem, RTC_CNTL_SAR_INT_ST_M);
    ESP_ERROR_CHECK(err);

    err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);

    ulp_sample_counter = 0;
    ulp_wait_processing = 0;

    while(1) {
        int result = xSemaphoreTake(ulp_isr_sem, portMAX_DELAY);
        if (result == pdPASS) {
            ulp_last_result &= UINT16_MAX;
            printf("Sample counter: %d\n", ulp_sample_counter & UINT16_MAX);
            printf("Last result: %d\n", ulp_last_result);
            xSemaphoreGive(ulp_isr_sem);
            ulp_sample_counter = 0;
            ulp_wait_processing = 0;
            //esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
            //ESP_ERROR_CHECK(err);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
