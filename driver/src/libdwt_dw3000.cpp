/**
 * @file libdwt_dw3000.cpp
 * 
 * @brief Implementation of the DW3000 driver for the Decawave DW3000 IC.
 * 
 * This file contains the implementation of the DW3000 driver for the Decawave DW3000 IC.
 * 
 * @attention //TODO: put attention here
 * 
 * @author Nguyen Ha Trung
 */


#include "deca_version.h"
#include "deca_interface.h"
#include "deca_device_api.h"

#include "dw3000_vals.h"
#include "dw3000_regs.h"

#define SET_SPI_HEADER_0(base_addr, sub_addr)\
    ((0x00 | 0x40) | ((base_addr) << 1) | (((sub_addr) & 0x40) >> 7)) // r bit + bit 1 + base address + 1 bit of sub address
#define SET_SPI_HEADER_1(sub_addr)\
    (((sub_addr) & 0x7F) << 2) // r bit + bit 1 + base address + 1 bit of sub address


// -------------------------------------------------------------------------------------------------------------------
struct dwt_probe_s *pdwt_probe;

// -------------------------------------------------------------------------------------------------------------------
// Structure to hold the device data

struct dwt_localdata_s
{
    uint32_t      partID ;            // IC Part ID - read during initialisation
    uint32_t      lotID ;             // IC Lot ID - read during initialisation
    uint8_t       bias_tune;          // bias tune code
    uint8_t       dgc_otp_set;        // Flag to check if DGC values are programmed in OTP
    uint8_t       vBatP;              // IC V bat read during production and stored in OTP (Vmeas @ 3V3)
    uint8_t       tempP;              // IC temp read during production and stored in OTP (Tmeas @ 23C)
    uint8_t       longFrames ;        // Flag in non-standard long frame mode
    uint8_t       otprev ;            // OTP revision number (read during initialisation)
    uint8_t       init_xtrim;         // initial XTAL trim value read from OTP (or defaulted to mid-range if OTP not programmed)
    uint8_t       dblbuffon;          // Double RX buffer mode and DB status flag
    uint16_t      sleep_mode;         // Used for automatic reloading of LDO tune and microcode at wake-up
    uint16_t       ststhreshold;       // Threshold for deciding if received STS is good or bad
    dwt_spi_crc_mode_e   spicrc;      // Use SPI CRC when this flag is true
    uint8_t       stsconfig;          // STS configuration mode
    uint8_t       cia_diagnostic;     // CIA dignostic logging level

    /* Callbacks */
    dwt_cb_data_t cbData;             // Callback data structure
    dwt_spierrcb_t cbSPIRDErr;        // Callback for SPI read error events
    dwt_cb_t    cbTxDone;             // Callback for TX confirmation event
    dwt_cb_t    cbRxOk;               // Callback for RX good frame event
    dwt_cb_t    cbRxTo;               // Callback for RX timeout events
    dwt_cb_t    cbRxErr;              // Callback for RX error events
    dwt_cb_t    cbSPIErr;             // Callback for SPI error events
    dwt_cb_t    cbSPIRdy;             // Callback for SPI ready events
};

typedef dwt_localdata_s dw_localdata_t;

dw_localdata_t dw_localdata[DWT_NUM_DW_DEV];

dw_localdata_t *p_dw_localdata;

// -------------------------------------------------------------------------------------------------------------------
// Function prototypes

static void dwt_readregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len) {
    uint8_t header_buf[2] = {
        SET_SPI_HEADER_0(base_addr, sub_addr),
        SET_SPI_HEADER_1(sub_addr)
    };

    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);

    spi_fct->readfromspi(2, header_buf, len, buffer);
}

static void dwt_writeregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len) {
    uint8_t header_buf[2] = {
        SET_SPI_HEADER_0(base_addr, sub_addr),
        SET_SPI_HEADER_1(sub_addr)
    };

    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);

    spi_fct->writetospi(2, header_buf, len, buffer);
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function selects the correct DecaDriver from the list
 *
 * input parameters
 * @param probe_interf pointer to a dwt_probe_s structure. See above description
 *
 * output parameters
 *
 * returns ret - DWT_ERROR  if no driver found or DWT_SUCCESS if driver is found.
 */
int dwt_probe(struct dwt_probe_s *probe_interf) {

    if (probe_interf == NULL) {
        return DWT_ERROR;
    }

    pdwt_probe = probe_interf;

    return DWT_SUCCESS;
}


/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function returns the version of the API
 *
 * input parameters
 *
 * output parameters
 *
 * returns version (DW3xxx_DRIVER_VERSION)
 */
int32_t dwt_apiversion(void) {

    return DRIVER_VERSION_HEX;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function will update dw pointer used by interrupt
 *
 * input parameters
 * @param new_dw - dw instatiated by MCPS layer
 *
 * return parameters
 * old_dw pointer. This pointer can be restored when deleting MCPS instance
 *
 */
struct dwchip_s* dwt_update_dw(struct dwchip_s* new_dw) {
    //TODO: implement this function
    return NULL;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function returns the driver version of the API
 *
 * input parameters
 *
 * output parameters
 *
 * returns version string
 */
char *dwt_version_string(void) {

    return (char*)DRIVER_VERSION_STR;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function sets the local data structure pointer to point to the element in the local array as given by the index.
 *
 * input parameters
 * @param index    - selects the array element to point to. Must be within the array bounds, i.e. < DWT_NUM_DW_DEV
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_setlocaldataptr(unsigned int index){
    
    if (index >= DWT_NUM_DW_DEV) {
        return DWT_ERROR;
    }

    p_dw_localdata = &dw_localdata[index];

    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read V measured @ 3.0 V value recorded in OTP address 0x8 (VBAT_ADDRESS)
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 8 bit V bat value as programmed in the factory
 */
uint8_t dwt_geticrefvolt(void) {
    return p_dw_localdata->vBatP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read T measured @ 22 C value recorded in OTP address 0x9 (VTEMP_ADDRESS)
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 8 bit V temp value as programmed in the factory
 */
uint8_t dwt_geticreftemp(void) {
    return p_dw_localdata->tempP;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read part ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit part ID value as programmed in the factory
 */
uint32_t dwt_getpartid(void) {
    return p_dw_localdata->partID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read lot ID of the device
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the 32 bit lot ID value as programmed in the factory
 */
uint32_t dwt_getlotid(void) {
    return p_dw_localdata->lotID;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read device type and revision information of the DW UWB chip
 *
 * input parameters
 *
 * output parameters
 *
 * returns the silicon DevID
 */
uint32_t dwt_readdevid(void) {

    uint8_t read_buf[4] = {0}; // Initialize the buffer to zero

    // Read the device ID from the DW3000_DEV_ID register
    dwt_readregfulladdr(DW3000_REG_0_ADDR, DW3000_REG_0_DEV_ID_OFFSET, read_buf, 4);

    // Return the device ID in little-endian format
    return *((uint32_t*)read_buf);  // This optimizes the conversion of 4 bytes into a 32-bit integer
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to return the read OTP revision
 *
 * NOTE: dwt_initialise() must be called prior to this function so that it can return a relevant value.
 *
 * input parameters
 *
 * output parameters
 *
 * returns the read OTP revision value
 */
uint8_t dwt_otprevision(void) {
    return p_dw_localdata->otprev;
}

/*! ------------------------------------------------------------------------------------------------------------------
  * @brief This function enables/disables the fine grain TX sequencing (enabled by default).
  *
  * input parameters
  * @param enable - 1 to enable fine grain TX sequencing, 0 to disable it.
  *
  * output parameters none
  *
  * no return value
  */
void dwt_setfinegraintxseq(bool enable) {

    // Select the appropriate value based on the 'enable' flag
    uint32_t fine_seq_value = enable ? PMSC_TXFINESEQ_ENABLE : PMSC_TXFINESEQ_DISABLE;

    // Convert the 32-bit value to a byte array (little-endian)
    uint8_t fine_seq_buf[4] = {
        (uint8_t)(fine_seq_value & 0xFF),
        (uint8_t)((fine_seq_value >> 8) & 0xFF),
        (uint8_t)((fine_seq_value >> 16) & 0xFF),
        (uint8_t)((fine_seq_value >> 24) & 0xFF)
    };

    // Obtain the SPI function pointer
    dwt_writeregfulladdr(DW3000_REG_17_ADDR, DW3000_REG_17_TXFSEQ_OFFSET, fine_seq_buf, 4);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to enable GPIO for external LNA or PA functionality - HW dependent, consult the DW3000 User Manual.
 *        This can also be used for debug as enabling TX and RX GPIOs is quite handy to monitor DW3000's activity.
 *
 * NOTE: Enabling PA functionality requires that fine grain TX sequencing is deactivated. This can be done using
 *       dwt_setfinegraintxseq().
 *
 * input parameters
 * @param lna_pa - bit field: bit 0 if set will enable LNA functionality,
 *                          : bit 1 if set will enable PA functionality,
 *                          : to disable LNA/PA set the bits to 0 (
 * output parameters
 *
 * no return value
 */
void dwt_setlnapamode(int lna_pa) {

    // read the current value
    uint32_t gpio_modes = 0;
    dwt_readregfulladdr(DW3000_REG_5_ADDR, DW3000_REG_5_GPIO_MODE_OFFSET, (uint8_t*)&gpio_modes, 4);

    // clear GPIO 0, 1, 4, 5, 6, configuration
    gpio_modes &= (~(DW3000_REG_5_GPIO_MODE_MSGP0_BIT_MASK | DW3000_REG_5_GPIO_MODE_MSGP1_BIT_MASK |\
                    DW3000_REG_5_GPIO_MODE_MSGP4_BIT_MASK | DW3000_REG_5_GPIO_MODE_MSGP5_BIT_MASK | DW3000_REG_5_GPIO_MODE_MSGP6_BIT_MASK));
    
    if (lna_pa & DWT_LNA_ENABLE)
    {
        gpio_modes |= GPIO_PIN6_EXTRX;   
    }
    if (lna_pa & DWT_PA_ENABLE)
    {
        gpio_modes |= (GPIO_PIN4_EXTDA | GPIO_PIN5_EXTTX);
    }
    if (lna_pa & DWT_TXRX_EN)
    {
        gpio_modes |= (GPIO_PIN0_EXTTXE | GPIO_PIN1_EXTRXE);
    }
    
    // write the new value
    dwt_writeregfulladdr(DW3000_REG_5_ADDR, DW3000_REG_5_GPIO_MODE_OFFSET, (uint8_t*)&gpio_modes, 4);
}













