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
#include "deca_device_api.h"


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

    return DRIVER_VERSION_STR;
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
    //TODO: implement this function
}










