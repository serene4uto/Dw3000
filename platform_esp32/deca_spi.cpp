/**
 * @file      deca_spi.c
 *
 * @brief     SPI functions to interface to DW3000 chip's from ESP32.
 *
 *
 * @author    Nguyen Ha Trung
 *
 * @attention //TODO: put attention here
 *
 */


#include "port.h"
#include "assert.h"

//-----------------------------------------------------------------------------
#include <SPI.h>

// Define the SPI handler structure
// static struct {
//     SPIClass *spi;                   // Pointer to SPI instance (e.g., &SPI)
//     SPISettings fastSettings;        // SPI settings for fast rate
//     SPISettings slowSettings;        // SPI settings for slow rate
//     uint8_t csPin;                   // Chip Select pin number
//     SemaphoreHandle_t spiMutex;      // Mutex for thread-safe access
// } SpiHandler;

//------------------------------------------------------------------------------
// DW chip description



//-----------------------------------------------------------------------------

/* @fn  set_dw_spi_slow_rate
 * @brief sets slow SPI clock speed for the DW chip
 *        left for compatibility.
 * */
void set_dw_spi_slow_rate(void)
{

}

/* @fn      set_dw_spi_fast_rate
 * @brief   sets High SPI clock speed for the DW chip
 * */
void set_dw_spi_fast_rate(void)
{

}



/****************************************************************************//**
 *
 *                              DWxxx SPI section
 *
 *******************************************************************************/
/*!
 * @fn  openspi()
 * @brief
 * Low level abstract function to open and initialise access to the SPI device.
 * returns 0 for success, or -1 for error
 */
int openspi(/*SPI_TypeDef* SPIx*/)
{
    return 0;
}


/*!
 * @fn  closespi()
 * @brief
 * Low level abstract function to close the the SPI device.
 * returns 0 for success, or -1 for error
 */
int closespi(void)
{
    return 0;
}