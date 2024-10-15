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

#define SET_SPI_HEADER_0_R(base_addr, sub_addr) \
    static_cast<uint8_t>(((0x00 | 0x40) | ((base_addr) << 1) | (((sub_addr) & 0x40) >> 7))) // r bit + bit 1 + base address + 1 bit of sub address
#define SET_SPI_HEADER_0_W(base_addr, sub_addr) \
    static_cast<uint8_t>( 0x80 | 0x40 | (base_addr << 1) ) | ( ( sub_addr & 0x40 ) >> 7 ) // w bit + bit 1 + base address + 1 bit of sub address
#define SET_SPI_HEADER_1(sub_addr) \
    static_cast<uint8_t>((((sub_addr) & 0x7F) << 2)) // r bit + bit 1 + base address + 1 bit of sub address

// OTP addresses definitions
#define LDOTUNELO_ADDRESS (0x04)
#define LDOTUNEHI_ADDRESS (0x05)
#define PARTID_ADDRESS  (0x06)
#define LOTID_ADDRESS   (0x07)
#define VBAT_ADDRESS    (0x08)
#define VTEMP_ADDRESS   (0x09)
#define XTRIM_ADDRESS   (0x1E)
#define OTPREV_ADDRESS  (0x1F)
#define BIAS_TUNE_ADDRESS (0xA)
#define DGC_TUNE_ADDRESS (0x20)

// Defines for enable_clocks function
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_AUTO          (5)
//SYSCLK
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSCDIV4   (1)
#define FORCE_SYSCLK_FOSC       (3)
//RX and TX CLK
#define FORCE_CLK_PLL           (2)

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

dw_localdata_t *p_dw_localdata = &dw_localdata[0]; // Default to first instance

// -------------------------------------------------------------------------------------------------------------------
// Internal functions prototypes for controlling and configuring the device
//
static void dwt_readregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len);
static void dwt_writeregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len);
static void dwt_force_clocks(int clocks);

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function read register value from DW3000 with full address
 * 
 */
static void dwt_readregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len) {
    uint8_t header_buf[2] = {
        SET_SPI_HEADER_0_R(base_addr, sub_addr),
        SET_SPI_HEADER_1(sub_addr)
    };

    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);

    spi_fct->readfromspi(2, header_buf, len, buffer);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function write register value to DW3000 with full address
 * 
 */
static void dwt_writeregfulladdr(uint8_t base_addr, uint16_t sub_addr, uint8_t *buffer, uint16_t len) {
    uint8_t header_buf[2] = {
        SET_SPI_HEADER_0_W(base_addr, sub_addr),
        SET_SPI_HEADER_1(sub_addr)
    };

    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);

    spi_fct->writetospi(2, header_buf, len, buffer);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief function to enable/disable clocks to particular digital blocks/system
 *
 * input parameters
 * @param clocks - set of clocks to enable/disable
 *
 * output parameters none
 *
 * no return value
 */
static void dwt_force_clocks(int clocks) {
    uint16_t wregval = 0;
    if (clocks == FORCE_CLK_SYS_TX) {
        wregval = (uint16_t)(CLK_CTRL_TX_BUF_CLK_ON_BIT_MASK | CLK_CTRL_RX_BUF_CLK_ON_BIT_MASK)
    }

    if (clocks == FORCE_CLK_AUTO) {
        //Restore auto clock mode
        //we only need to restore the low 16 bits as they are the only ones to change as a result of FORCE_CLK_SYS_TX
        wregval = (uint16_t)DWT_AUTO_CLKS;
        dwt_writeregfulladdr(DW3000_REG_17_ADDR, DW3000_REG_17_CLK_CTRL_OFFSET, (uint8_t*)&wregval, 2);
    }
}


// -------------------------------------------------------------------------------------------------------------------
// API functions


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

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to configure GPIO function
 *
 *
 * input parameters
 * @param gpio_mask - the mask of the GPIOs to change the mode of. Typically built from dwt_gpio_mask_e values.
 * @param gpio_modes - the GPIO modes to set. Typically built from dwt_gpio_pin_e values.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setgpiomode(uint32_t gpio_mask, uint32_t gpio_modes) {
    //TODO: implement this function
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to configure the GPIOs as inputs or outputs, default is input == 1
 *
 * input parameters
 * @param in_out - if corresponding GPIO bit is set to 1 then it is input, otherwise it is output
 *               - GPIO 0 = bit 0, GPIO 1 = bit 1 etc...
 *
 * output parameters
 *
 * no return value
 */
void dwt_setgpiodir(uint16_t in_out){
    //TODO: implement this function
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to set output value on GPIOs that have been configured for output via dwt_setgpiodir() API
 *
 * input parameters
 * @param gpio - should be one or multiple of dwt_gpio_mask_e values
 * @param value - Logic value for GPIO or GPIOs if multiple set at same time.
 *
 * output parameters
 *
 * no return value
 */
void dwt_setgpiovalue(uint16_t gpio, int value){
    //TODO: implement this function
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the raw value of the GPIO pins.
 *        It is presumed that functions such as dwt_setgpiomode(), dwt_setgpiovalue() and dwt_setgpiodir() are called before this function.
 *
 * input parameters
 *
 * returns a uint16_t value that holds the value read on the GPIO pins.
 */
uint16_t dwt_readgpiovalue(void){
    //TODO: implement this function
    return 0;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function initialises the DW3xxx transceiver:
 * It performs the initially required device configurations and initializes
 * a static data belonging to the low-level driver.
 *
 * NOTES:
 * 1.this function needs to be run before dwt_configuresleep, also the SPI frequency has to be < 7MHz
 * 2.it also reads and applies LDO and BIAS tune and crystal trim values from OTP memory
 * 3.it is assumed this function is called after a reset or on power up of the DW3xxx transceiver
 *
 * input parameters
 * @param mode - mask which defines which OTP values to read.
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_initialise(int mode){

    uint32_t ldo_tune_lo;
    uint32_t ldo_tune_hi;
    uint32_t bias_tune;

    uint16_t otp_cfg;
    uint32_t otp_dgc_tune;
    uint32_t otp_part_id;
    uint32_t otp_lot_id;
    uint32_t otp_vbat;
    uint32_t otp_vtemp;
    uint32_t otp_prev;
    uint32_t otp_xtrim;

    // set SPI rate to < 7MHz
    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);
    spi_fct->setslowrate();

    // set local
    p_dw_localdata->dblbuffon = DBL_BUFF_OFF; // Double buffer mode off by default / clear the flag
    p_dw_localdata->sleep_mode = DWT_RUNSAR;  // Configure RUN_SAR on wake by default as it is needed when running PGF_CAL
    p_dw_localdata->spicrc = DWT_SPI_CRC_MODE_NO;
    p_dw_localdata->stsconfig = 0; //STS off
    p_dw_localdata->vBatP = 0;
    p_dw_localdata->tempP = 0;

    // Set Callbacks
    p_dw_localdata->cbTxDone = NULL;
    p_dw_localdata->cbRxOk = NULL;
    p_dw_localdata->cbRxTo = NULL;
    p_dw_localdata->cbRxErr = NULL;
    p_dw_localdata->cbSPIRdy = NULL;
    p_dw_localdata->cbSPIErr = NULL;

    // Read the device ID and check if it is the right one
    if (dwt_check_dev_id() == DWT_ERROR) {
        return DWT_ERROR;
    }

    //Read LDO_TUNE and BIAS_TUNE from OTP
    dwt_otpread(LDOTUNELO_ADDRESS, &ldo_tune_lo, 1);
    dwt_otpread(LDOTUNEHI_ADDRESS, &ldo_tune_hi, 1);
    dwt_otpread(BIAS_TUNE_ADDRESS, &bias_tune, 1);

    // Based on https://gist.github.com/egnor/455d510e11c22deafdec14b09da5bf54
    // The OTP memory contains calibration/tuning values which should be loaded into operating registers on startup (and possibly wake from sleep?).
    // There are "KICK" bits in the OTP_SF register to do this, but apparently they don't do a complete job.
    p_dw_localdata->bias_tune = (uint8_t)((bias_tune >> 16) & 0x1F);

    dwt_readregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_CFG_OFFSET, (uint8_t*)&otp_cfg, 2);
    // set LDO_KICK in OTP_CFG
    otp_cfg |= DW3000_REG_11_OTP_CFG_LDO_KICK_BIT_MASK; 
    // set BIAS_KICK (undocumented bit 8!) in OTP_CFG
    otp_cfg |= DW3000_REG_11_OTP_CFG_BIAS_KICK_BIT_MASK;

    if (ldo_tune_lo && ldo_tune_hi && p_dw_localdata->bias_tune) {
        uint8_t ldo_tune[8];
        for (int i = 0; i < 4; i++) {
            ldo_tune[i]     = (ldo_tune_lo  >> (i * 8)) & 0xFF;
            ldo_tune[i + 4] = (ldo_tune_hi  >> (i * 8)) & 0xFF;
        }
        // Write LDO_TUNE_LO and LDO_TUNE_HI to LDO_TUNE
        dwt_writeregfulladdr(DW3000_REG_7_ADDR, DW3000_REG_7_LDO_TUNE_OFFSET, ldo_tune, 8);

        // Update BIAS_CTRL with bias_tune
        bias_tune = (uint16_t)p_dw_localdata->bias_tune;
        dwt_writeregfulladdr(DW3000_REG_17_ADDR, DW3000_REG_17_BIAS_CTRL_OFFSET, (uint8_t *)&bias_tune, 2);
    }

    // Load DGC_CFG from OTP
    dwt_otpread(DGC_TUNE_ADDRESS, &otp_dgc_tune, 1);
    if (otp_dgc_tune == DWT_DGC_CFG0) {
        p_dw_localdata->dgc_otp_set = DWT_DGC_LOAD_FROM_OTP;
    }
    else
    {
        p_dw_localdata->dgc_otp_set = DWT_DGC_LOAD_FROM_SW;
    }


    // Load Part and Lot ID from OTP
    if(!(mode & DWT_READ_OTP_PID)) {
        dwt_otpread(PARTID_ADDRESS, &otp_part_id, 1);
        p_dw_localdata->partID = otp_part_id;
    }

    if(!(mode & DWT_READ_OTP_LID)) {
        dwt_otpread(LOTID_ADDRESS, &otp_lot_id, 1);
        p_dw_localdata->lotID = otp_lot_id;
    }

    // Load Vbat and Vtemp from OTP
    if(!(mode & DWT_READ_OTP_BAT)) {
        dwt_otpread(VBAT_ADDRESS, &otp_vbat, 1);
        p_dw_localdata->vBatP = (uint8_t)otp_vbat;
    }

    if(!(mode & DWT_READ_OTP_TMP)) {
        dwt_otpread(VTEMP_ADDRESS, &otp_vtemp, 1);
        p_dw_localdata->tempP = (uint8_t)otp_vtemp;
    }

    if(p_dw_localdata->tempP == 0) //if the reference temperature has not been programmed in OTP (early eng samples) set to default value
    {
        p_dw_localdata->tempP = 0x85 ; //@temp of 20 deg
    }

    if(p_dw_localdata->vBatP == 0) //if the reference voltage has not been programmed in OTP (early eng samples) set to default value
    {
        p_dw_localdata->vBatP = 0x74 ;  //@Vref of 3.0V
    }

    // Load OTP revision from OTP
    dwt_otpread(OTPREV_ADDRESS, &otp_prev, 1);
    p_dw_localdata->otprev = (uint8_t)otp_prev;

    // Load Xtrim from OTP
    dwt_otpread(XTRIM_ADDRESS, &otp_xtrim, 1);
    //copy bits 0-5 (6 bits) from OTP XTAL_Trim into XTAL
    p_dw_localdata->init_xtrim = (uint8_t)otp_xtrim & 0x7f;
    if(p_dw_localdata->init_xtrim == 0) //if the XTAL trim has not been programmed in OTP (early eng samples) set to default value
    {
        p_dw_localdata->init_xtrim = 0x2E ;  //default value
    }
    dwt_writeregfulladdr(DW3000_REG_9_ADDR, DW3000_REG_9_XTAL_XTAL_TRIM_BIT_OFFSET, (uint8_t*)&p_dw_localdata->init_xtrim, 1);

    //TODO: print the OTP values for debugging

    //TODO: check if it is necessary to set SPI rate to fast ?

    return DWT_SUCCESS;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This function can place DW3000 into IDLE/IDLE_PLL or IDLE_RC mode when it is not actively in TX or RX.
 *
 * input parameters
 * @param state - DWT_DW_IDLE (1) to put DW3000 into IDLE/IDLE_PLL state; DWT_DW_INIT (0) to put DW3000 into INIT_RC state;
 *                DWT_DE_IDLE_RC (2) to put DW3000 into IDLE_RC state.
 *
 * output parameters none
 *
 * no return value
 */
void dwt_setdwstate(int state) {

    if (state == DWT_DW_IDLE) {
        // Set the auto INIT2IDLE bit so that DW3000 enters IDLE mode before switching clocks to system_PLL
        //NOTE: PLL should be configured prior to this, and the device should be in IDLE_RC (if the PLL does not lock device will remain in IDLE_RC)

        //switch clock to auto - if coming here from INIT_RC the clock will be FOSC/4, need to switch to auto prior to setting auto INIT2IDLE bit


    }

    if (state == DWT_DW_INIT) {
    }

    if (state == DWT_DW_IDLE_RC) {

    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief This is used to read the OTP data from given address into provided array
 *
 * input parameters
 * @param address - this is the OTP address to read from
 * @param array - this is the pointer to the array into which to read the data
 * @param length - this is the number of 32 bit words to read (array needs to be at least this length)
 *
 * output parameters
 *
 * no return value
 */
void dwt_otpread(uint16_t address, uint32_t* array, uint8_t length) {
    uint16_t otp_cfg;

    // read current OTP configuration
    dwt_readregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_CFG_OFFSET, (uint8_t*)&otp_cfg, 2);

    for (int i = 0; i < length; i++) {

        // set OTP_MAN (bit 0) in OTP_CFG && clear OTP_READ / OTP_WRITE (bit 1 and 2) in OTP_CFG (to make sure)
        otp_cfg &= ~(DW3000_REG_11_OTP_CFG_OTP_READ_BIT_MASK | DW3000_REG_11_OTP_CFG_OTP_WRITE_BIT_MASK);
        otp_cfg |= DW3000_REG_11_OTP_CFG_OTP_MAN_BIT_MASK;
        dwt_writeregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_CFG_OFFSET, (uint8_t*)&otp_cfg, 2);

        // write the OTP register address (7 bits) to OTP_ADDR
        uint16_t otp_address = (address + i) & 0x7F; // 7 bits
        dwt_writeregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_ADDR_OFFSET, (uint8_t*)&otp_address, 2);

        // clear OTP_MAN (bit 0) and set OTP_READ (bit 1) in OTP_CFG
        otp_cfg &= ~DW3000_REG_11_OTP_CFG_OTP_MAN_BIT_MASK;
        otp_cfg |= DW3000_REG_11_OTP_CFG_OTP_READ_BIT_MASK;
        dwt_writeregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_CFG_OFFSET, (uint8_t*)&otp_cfg, 2);

        // read the data from OTP_RDATA
        dwt_readregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_RDATA_OFFSET, (uint8_t*)&array[i], 4);

        // clear OTP_READ (bit 1) in OTP_CFG
        otp_cfg &= ~DW3000_REG_11_OTP_CFG_OTP_READ_BIT_MASK;
        dwt_writeregfulladdr(DW3000_REG_11_ADDR, DW3000_REG_11_OTP_CFG_OFFSET, (uint8_t*)&otp_cfg, 2);

    }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief this reads the device ID and checks if it is the right one
 *
 * input parameters
 * None
 *
 * output parameters
 *
 * returns DWT_SUCCESS for success, or DWT_ERROR for error
 */
int dwt_check_dev_id(void) {

    uint32_t dev_id = dwt_readdevid();

    //TODO: print the device ID for debugging

    if (!(DWT_DW3000_DEV_ID == dev_id)) {
        return DWT_ERROR;
    }

    return DWT_SUCCESS;
}













