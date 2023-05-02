#include <stdio.h>
#include <esp_log.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "input_iot.h"
#include "freertos/timers.h"

input_callback_t input_callback = NULL;
timeoutButton_t timeoutButton_callback = NULL;

static TimerHandle_t xTimers;

static void IRAM_ATTR gpio_input_handler(void *arg) {
    int gpio_num = (uint32_t) arg;
    uint32_t rtc = xTaskGetTickCountFromISR();   //ms
    if(gpio_get_level(gpio_num) == 0)            //bat dau bam
    {
        xTimerStart(xTimers, 0);
    }                                           //tha tay ra
    else{   
        xTimerStop(xTimers, 0);
    }
}

static void vTimerCallback( TimerHandle_t xTimer )
 {
 uint32_t ID;

    /* Optionally do something if the pxTimer parameter is NULL. */
    configASSERT( xTimer );

    /* The number of times this timer has expired is saved as the
    timer's ID.  Obtain the count. */
    ID = ( uint32_t ) pvTimerGetTimerID( xTimer );
    if(ID == 0)
    {
        timeoutButton_callback(BUTTON0);
    }
 }

void input_io_create(gpio_num_t gpio_num, interrupt_type_edle_t type, uint8_t x)
{
    gpio_pad_select_gpio(gpio_num);
    gpio_set_direction(gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio_num, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(gpio_num, type);
    gpio_install_isr_service(x);
    gpio_isr_handler_add(gpio_num, gpio_input_handler, (void*) gpio_num);
    xTimers = xTimerCreate
                   ( /* Just a text name, not used by the RTOS
                     kernel. */
                     "TimerForTimeout",
                     /* The timer period in ticks, must be
                     greater than 0. */
                    500/portTICK_PERIOD_MS,
                     /* The timers will auto-reload themselves
                     when they expire. */
                     pdFAIL,
                     /* The ID is used to store a count of the
                     number of times the timer has expired, which
                     is initialised to 0. */
                     ( void * ) 0,
                     /* Each timer calls the same callback when
                     it expires. */
                     vTimerCallback
                   );
}

void input_io_get_level(gpio_num_t gpio_num)
{
    return gpio_get_level(gpio_num);
}

void input_set_timeout_callback(void * cb)
{
   timeoutButton_callback = cb;
}