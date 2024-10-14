/**
 * @file      portDW.c
 *
 * @brief     Platform-dependent functions for current application are collected here
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put copyright
 *            All rights reserved.
 *
 */

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "port.h"


/****************************************************************************//**
 * Sleep, usleep and bare sw_timer based on HAL tick
 */

/**
 * @brief Start a timer by saving the current timestamp in microseconds.
 * @param p_timestamp Pointer to store the current timestamp.
 */
void start_timer(volatile uint32_t *p_timestamp)
{
    *p_timestamp = esp_timer_get_time(); // Get time in microseconds
}

/**
 * @brief Check if a specified time interval has elapsed since the timestamp.
 * @param timestamp The starting timestamp in microseconds.
 * @param time The time interval to check against in microseconds.
 * @return true if the time interval has elapsed, false otherwise.
 */
bool check_timer(volatile uint32_t timestamp, uint32_t time)
{
    return (esp_timer_get_time() - timestamp) >= time;
}

/**
 * @brief Sleep for a specified number of milliseconds.
 * @param dwMs Number of milliseconds to sleep.
 */
void dwp_Sleep(uint32_t dwMs)
{
    // If using FreeRTOS, prefer vTaskDelay
    vTaskDelay(pdMS_TO_TICKS(dwMs));

    // If blocking delay is required without FreeRTOS:
    /*
    uint64_t dwStart;
    start_timer(&dwStart);
    uint64_t delay_us = (uint64_t)dwMs * 1000ULL; // Convert ms to us
    while (!check_timer(dwStart, delay_us))
    {
        // Optionally yield to other tasks or add a small delay
        taskYIELD();
    }
    */
}

/**
 * @brief Precise microsecond delay.
 * @param usec Number of microseconds to delay.
 */
void dwp_usleep(uint32_t usec)
{
    // Use the ROM function for microsecond delay
    ets_delay_us(usec);
}