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

#include <assert.h>
#include <stdlib.h>
#include <stdint.h>

#include "deca_version.h"
#include "deca_interface.h"
#include "deca_device_api.h"

#include "deca_regs.h"
#include "deca_vals.h"

/* MACRO */
#define dwt_write32bitreg(addr,value)  dwt_write32bitoffsetreg(addr,0,value)
#define dwt_read32bitreg(addr)     dwt_read32bitoffsetreg(addr,0)
#define dwt_writefastCMD(cmd)     dwt_writetodevice(cmd,0,0,0)

#define dwt_or8bitoffsetreg(addr, offset, or_val) dwt_modify8bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and8bitoffsetreg(addr, offset, and_val) dwt_modify8bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or8bitoffsetreg(addr,offset, and_val, or_val) dwt_modify8bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_8bit_reg(addr,bit_num) dwt_modify8bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_8bit_reg(addr,bit_num) dwt_modify8bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

#define dwt_or16bitoffsetreg(addr, offset, or_val) dwt_modify16bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and16bitoffsetreg(addr, offset, and_val) dwt_modify16bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or16bitoffsetreg(addr,offset, and_val, or_val) dwt_modify16bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_16bit_reg(addr,bit_num) dwt_modify16bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_16bit_reg(addr,bit_num) dwt_modify16bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

#define dwt_or32bitoffsetreg(addr, offset, or_val) dwt_modify32bitoffsetreg(addr, offset, -1, or_val)
#define dwt_and32bitoffsetreg(addr, offset, and_val) dwt_modify32bitoffsetreg(addr, offset, and_val, 0)
#define dwt_and_or32bitoffsetreg(addr,offset, and_val, or_val) dwt_modify32bitoffsetreg(addr,offset,and_val,or_val)
#define dwt_set_bit_num_32bit_reg(addr,bit_num) dwt_modify32bitoffsetreg(addr,0,-1,DWT_BIT_MASK(bit_num))
#define dwt_clr_bit_num_32bit_reg(addr,bit_num) dwt_modify32bitoffsetreg(addr,0,~DWT_BIT_MASK(bit_num),0)

//DW-IC SPI CRC-8 polynomial
#define POLYNOMIAL  0x07    /* x^8 + x^2 + x^1 + x^0 */
#define TOPBIT      (1 << (8 - 1))

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

// dwt_readcarrierintegrator defines
#define B20_SIGN_EXTEND_TEST (0x00100000UL)
#define B20_SIGN_EXTEND_MASK (0xFFF00000UL)
#define DRX_CARRIER_INT_LEN  (3)

#define CIA_MANUALLOWERBOUND_TH_64  (0x10) //cia lower bound threshold values for 64 MHz PRF
#define STSQUAL_THRESH_64 (0.90f)
 //STS quality threshold values for 64 MHz PRF
 //when using 64 MHz PRF the stsCpQual should be > 90 % of STS length

// dwt_readclockoffset defines
#define B11_SIGN_EXTEND_TEST (0x1000UL)
#define B11_SIGN_EXTEND_MASK (0xE000UL)

// dwt_readpdoa defines
#define B12_SIGN_EXTEND_TEST (0x2000UL)
#define B12_SIGN_EXTEND_MASK (0xC000UL)

// -------------------------------------------------------------------------------------------------------------------
// Macros and Enumerations for SPI & CLock blocks
//
#define DW3000_SPI_FAC      (0<<6 | 1<<0)
#define DW3000_SPI_FARW     (0<<6 | 0<<0)
#define DW3000_SPI_EAMRW    (1<<6)

// Defines for enable_clocks function
#define FORCE_CLK_SYS_TX        (1)
#define FORCE_CLK_AUTO          (5)
//SYSCLK
#define FORCE_SYSCLK_PLL        (2)
#define FORCE_SYSCLK_FOSCDIV4   (1)
#define FORCE_SYSCLK_FOSC       (3)
//RX and TX CLK
#define FORCE_CLK_PLL           (2)

#define SEL_CHANNEL5            (5)
#define SEL_CHANNEL9            (9)

// -------------------------------------------------------------------------------------------------------------------
// Internal functions prototypes for controlling and configuring the device
//
static void dwt_force_clocks(int clocks);
static uint32_t _dwt_otpread(uint16_t address);                     // Read non-volatile memory
static void _dwt_otpprogword32(uint32_t data, uint16_t address);  // Program the non-volatile memory

static void dwt_xfer3000(const uint32_t regFileID, const uint16_t indx, const uint16_t length, uint8_t *buffer, const spi_modes_e mode);
static void dwt_writetodevice(const uint32_t regFileID, const uint16_t indx, const uint16_t length, const uint8_t *buffer);
static void dwt_readfromdevice(const uint32_t regFileID, const uint16_t indx, const uint16_t length, uint8_t *buffer);
static uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset);
static uint16_t dwt_read16bitoffsetreg(int regFileID,int regOffset);
static uint8_t dwt_read8bitoffsetreg(int regFileID, int regOffset);
static void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval);
static void dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16_t regval);
static void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8_t regval);
static void dwt_modify32bitoffsetreg(const int regFileID, const int regOffset, const uint32_t _and, const uint32_t _or);
static void dwt_modify16bitoffsetreg(const int regFileID, const int regOffset, const uint16_t _and, const uint16_t _or);
static void dwt_modify8bitoffsetreg(const int regFileID, const int regOffset, const uint8_t _and, const uint8_t _or);


// -------------------------------------------------------------------------------------------------------------------
// Data for DW3000 Decawave Transceiver control
//
// Structure to hold the device data
typedef struct
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
    int16_t       ststhreshold;       // Threshold for deciding if received STS is good or bad
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
} dwt_local_data_t ;

// -------------------------------------------------------------------------------------------------------------------
// Local variables
//
static dwt_local_data_t   DW3000local[DWT_NUM_DW_DEV] ; // Local device data, can be an array to support multiple DW3000 testing applications/platforms
static dwt_local_data_t *pdw3000local = &DW3000local[0];   // Local data structure pointer
static uint8_t crcTable[256];

// -------------------------------------------------------------------------------------------------------------------
struct dwt_probe_s *pdwt_probe;

// -------------------------------------------------------------------------------------------------------------------

/*! ------------------------------------------------------------------------------------------------------------------
* @brief  this function is used to read/write to the DW3000 device registers
*
* input parameters:
* @param recordNumber  - ID of register file or buffer being accessed
* @param index         - byte index into register file or buffer being accessed
* @param length        - number of bytes being written
* @param buffer        - pointer to buffer containing the 'length' bytes to be written
* @param rw            - DW3000_SPI_WR_BIT/DW3000_SPI_RD_BIT
*
* no return value
*/
static
void dwt_xfer3000
(
    const uint32_t    regFileID,  //0x0, 0x04-0x7F ; 0x10000, 0x10004, 0x10008-0x1007F; 0x20000 etc
    const uint16_t    indx,       //sub-index, calculated from regFileID 0..0x7F,
    const uint16_t    length,
    uint8_t           *buffer,
    const spi_modes_e mode
)
{
    uint8_t  header[2];           // Buffer to compose header in
    uint16_t cnt = 0;             // Counter for length of a header

    uint16_t reg_file     = 0x1F & ((regFileID + indx) >> 16);
    uint16_t reg_offset   = 0x7F &  (regFileID + indx);

    const struct dwt_spi_s *spi_fct = (const struct dwt_spi_s *)(pdwt_probe->spi);

    assert(reg_file     <= 0x1F);
    assert(reg_offset   <= 0x7F);
    assert(length       < 0x3100);
    assert(mode == DW3000_SPI_WR_BIT ||\
           mode == DW3000_SPI_RD_BIT ||\
           mode == DW3000_SPI_AND_OR_8 ||\
           mode == DW3000_SPI_AND_OR_16 ||\
           mode == DW3000_SPI_AND_OR_32);

    // Write message header selecting WRITE operation and addresses as appropriate
    uint16_t  addr;
    addr = (reg_file << 9) | (reg_offset << 2);

    header[0] = (uint8_t)((mode | addr) >> 8);//  & 0xFF; //bit7 + addr[4:0] + sub_addr[6:6]
    header[1] = (uint8_t)(addr | (mode & 0x03));// & 0xFF; //EAM: subaddr[5:0]+ R/W/AND_OR

    if (/*reg_offset == 0 && */length == 0)
    {   /* Fast Access Commands (FAC)
         * only write operation is possible for this mode
         * bit_7=one is W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=one: MODE of FastAccess
         */
        assert(mode == DW3000_SPI_WR_BIT);

        header[0] = (uint8_t)((DW3000_SPI_WR_BIT>>8) | (regFileID<<1) | DW3000_SPI_FAC);
        cnt = 1;
    }
    else if (reg_offset == 0 /*&& length > 0*/ && (mode == DW3000_SPI_WR_BIT || mode == DW3000_SPI_RD_BIT))
    {   /* Fast Access Commands with Read/Write support (FACRW)
         * bit_7 is R/W operation, bit_6=zero: FastAccess command, bit_[5..1] addr, bits_0=zero: MODE of FastAccess
         */
        header[0] |= DW3000_SPI_FARW;
        cnt = 1;
    }
    else
    {   /* Extended Address Mode with Read/Write support (EAMRW)
         * b[0] = bit_7 is R/W operation, bit_6 one = ExtendedAddressMode;
         * b[1] = addr<<2 | (mode&0x3)
         */
        header[0] |= DW3000_SPI_EAMRW;
        cnt = 2;
    }

    switch (mode)
    {
    case    DW3000_SPI_AND_OR_8:
    case    DW3000_SPI_AND_OR_16:
    case    DW3000_SPI_AND_OR_32:
    case    DW3000_SPI_WR_BIT:
    {
        uint8_t crc8 = 0;
        if (pdw3000local->spicrc != DWT_SPI_CRC_MODE_NO)
        {
            //generate 8 bit CRC
            crc8 = dwt_generatecrc8(header, cnt, 0);
            crc8 = dwt_generatecrc8(buffer, length, crc8);

            // Write it to the SPI
            spi_fct->writetospiwithcrc(cnt, header, length, buffer, crc8);
        }
        else
        {
            // Write it to the SPI
            spi_fct->writetospi(cnt, header, length, buffer);
        }
        break;
    }
    case DW3000_SPI_RD_BIT:
        {
            spi_fct->readfromspi(cnt, header, length, buffer);

            //check that the SPI read has correct CRC-8 byte
            //also don't do for SPICRC_CFG_ID register itself to prevent infinite recursion
            if ((pdw3000local->spicrc == DWT_SPI_CRC_MODE_WRRD) && (regFileID != SPICRC_CFG_ID))
            {
                uint8_t crc8, dwcrc8;
                //generate 8 bit CRC from the read data
                crc8 = dwt_generatecrc8(header, cnt, 0);
                crc8 = dwt_generatecrc8(buffer, length, crc8);

                //read the CRC that was generated in the DW3000 for the read transaction
                dwcrc8 = dwt_read8bitoffsetreg(SPICRC_CFG_ID, 0);

                //if the two CRC don't match report SPI read error
                //potential problem in callback if it will try to read/write SPI with CRC again.
                if (crc8 != dwcrc8)
                {
                    if (pdw3000local->cbSPIRDErr != NULL)
                        pdw3000local->cbSPIRDErr();
                }

            }
            break;
        }
    default:
        while(1);
        break;
    }

} // end dwt_xfer3000()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write to the DW3000 device registers
 *
 * input parameters:
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being written
 * @param buffer        - pointer to buffer containing the 'length' bytes to be written
 *
 * output parameters
 *
 * no return value
 */

static
void dwt_writetodevice
(
    uint32_t      regFileID,
    uint16_t      index,
    uint16_t      length,
    uint8_t       *buffer
)
{
    dwt_xfer3000(regFileID, index, length, buffer, DW3000_SPI_WR_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read from the DW3000 device registers
 *
 * @param recordNumber  - ID of register file or buffer being accessed
 * @param index         - byte index into register file or buffer being accessed
 * @param length        - number of bytes being read
 * @param buffer        - pointer to buffer in which to return the read data.
 *
 * output parameters
 *
 * no return value
 */
static
void dwt_readfromdevice
(
    uint32_t  regFileID,
    uint16_t  index,
    uint16_t  length,
    uint8_t   *buffer
)
{
    dwt_xfer3000(regFileID, index, length, buffer, DW3000_SPI_RD_BIT);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read 32-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 32 bit register value
 */
static
uint32_t dwt_read32bitoffsetreg(int regFileID, int regOffset)
{
    int     j ;
    uint32_t  regval = 0 ;
    uint8_t   buffer[4] ;

    dwt_readfromdevice(regFileID,regOffset,4,buffer); // Read 4 bytes (32-bits) register into buffer

    for (j = 3 ; j >= 0 ; j --)
    {
        regval = (regval << 8) + buffer[j] ;
    }

    return (regval);

} // end dwt_read32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read 16-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 16 bit register value
 */
static
uint16_t dwt_read16bitoffsetreg(int regFileID,int regOffset)
{
    uint16_t  regval = 0 ;
    uint8_t   buffer[2] ;

    dwt_readfromdevice(regFileID,regOffset,2,buffer); // Read 2 bytes (16-bits) register into buffer

    regval = ((uint16_t)buffer[1] << 8) + buffer[0] ;
    return regval ;

} // end dwt_read16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to read an 8-bit value from the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 *
 * output parameters
 *
 * returns 8-bit register value
 */
static
uint8_t dwt_read8bitoffsetreg(int regFileID, int regOffset)
{
    uint8_t regval;

    dwt_readfromdevice(regFileID, regOffset, 1, &regval);

    return regval ;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write 32-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
static
void dwt_write32bitoffsetreg(int regFileID, int regOffset, uint32_t regval)
{
    int     j ;
    uint8_t   buffer[4] ;

    for ( j = 0 ; j < 4 ; j++ )
    {
        buffer[j] = (uint8_t)regval;
        regval >>= 8 ;
    }

    dwt_writetodevice(regFileID,regOffset,4,buffer);
} // end dwt_write32bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write 16-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
static
void dwt_write16bitoffsetreg(int regFileID, int regOffset, uint16_t regval)
{
    uint8_t   buffer[2] ;

    buffer[0] = (uint8_t)regval;
    buffer[1] = regval >> 8 ;

    dwt_writetodevice(regFileID,regOffset,2,buffer);
} // end dwt_write16bitoffsetreg()

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to write an 8-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID - ID of register file or buffer being accessed
 * @param regOffset - the index into register file or buffer being accessed
 * @param regval    - the value to write
 *
 * output parameters
 *
 * no return value
 */
static
void dwt_write8bitoffsetreg(int regFileID, int regOffset, uint8_t regval)
{
    //uint8_t   buf[1];
    //buf[0] = regval;
    dwt_writetodevice(regFileID, regOffset, 1, &regval);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 32-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
static
void dwt_modify32bitoffsetreg(const int regFileID, const int regOffset, const uint32_t _and, const uint32_t _or)
{
    uint8_t buf[8];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)(_and>>16);// &0xFF;
    buf[3] = (uint8_t)(_and>>24);// &0xFF;
    buf[4] = (uint8_t)_or;//        &0xFF;
    buf[5] = (uint8_t)(_or>>8);//   &0xFF;
    buf[6] = (uint8_t)(_or>>16);//  &0xFF;
    buf[7] = (uint8_t)(_or>>24);//  &0xFF;
    dwt_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_32);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 16-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
static
void dwt_modify16bitoffsetreg(const int regFileID, const int regOffset, const uint16_t _and, const uint16_t _or)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)_and;//       &0xFF;
    buf[1] = (uint8_t)(_and>>8);//  &0xFF;
    buf[2] = (uint8_t)_or;//        &0xFF;
    buf[3] = (uint8_t)(_or>>8);//   &0xFF;
    dwt_xfer3000(regFileID, regOffset, sizeof(buf), buf, DW3000_SPI_AND_OR_16);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @brief  this function is used to modify a 8-bit value to the DW3000 device registers
 *
 * input parameters:
 * @param regFileID :   ID of register file or buffer being accessed
 * @param regOffset :   the index into register file or buffer being accessed
 * @param regval_and:   the value to AND to register
 * @param regval_or :   the value to OR to register
 * @output          :   no return value
 */
static
void dwt_modify8bitoffsetreg(const int regFileID, const int regOffset, const uint8_t _and, const uint8_t _or)
{
    uint8_t buf[2];
    buf[0] = _and;
    buf[1] = _or;
    dwt_xfer3000(regFileID, regOffset, sizeof(buf),buf, DW3000_SPI_AND_OR_8);
}


// -------------------------------------------------------------------------------------------------------------------



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
int32_t dwt_apiversion(void) 
{
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







