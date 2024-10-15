/*! ----------------------------------------------------------------------------
 * @file    dw3000_regs.h
 *
 * @brief   DW3000 register definitions
 * 
 * @attention //TODO: put attention here
 * 
 * @author  Nguyen Ha Trung
 *
 */

#ifndef __DW3000_REGS_H__
#define __DW3000_REGS_H__

// Macro to extract a specific field value from a register using its mask and offset
#define GET_FVAL(reg_val, mask, offset)       (((reg_val) & (mask)) >> (offset))
// Macro to set a specific field value in a register using its mask and offset
#define SET_FVAL(reg_val, mask, offset, value)  ((reg_val) = (((reg_val) & ~(mask)) | (((value) << (offset)) & (mask))))

/******************************************************************************
* @brief Bit definitions for register page 0x00 : GEN_CFG_AES LOW
**/
#define DW3000_REG_0_ADDR                                 0x00

/* Register: DEV_ID (0x00) */
#define DW3000_REG_0_DEV_ID_OFFSET                        0x00   
#define DW3000_REG_0_DEV_ID_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_0_DEV_ID_REV_BIT_OFFSET                (0U)
#define DW3000_REG_0_DEV_ID_REV_BIT_MASK                  (0xFU)
#define DW3000_REG_0_DEV_ID_VER_BIT_OFFSET                (4U)
#define DW3000_REG_0_DEV_ID_VER_BIT_MASK                  (0xF0U)
#define DW3000_REG_0_DEV_ID_MODEL_BIT_OFFSET              (8U)
#define DW3000_REG_0_DEV_ID_MODEL_BIT_MASK                (0xFF00U)
#define DW3000_REG_0_DEV_ID_RIDTAG_BIT_OFFSET             (16U)
#define DW3000_REG_0_DEV_ID_RIDTAG_BIT_MASK               (0xFFFF0000UL)

/* Register: EUI_64 (0x04) */
#define DW3000_REG_0_EUI_64_OFFSET                        0x04
#define DW3000_REG_0_EUI_64_BYTE_LEN                      (8U)
//TODO: add more info

/* Register: PANADR (0x0A) */
#define DW3000_REG_0_PANADR_OFFSET                        0x0C
#define DW3000_REG_0_PANADR_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_0_PANADR_SHORT_ADDR_BIT_OFFSET         (0U)
#define DW3000_REG_0_PANADR_SHORT_ADDR_BIT_MASK           (0xFFFFU)
#define DW3000_REG_0_PANADR_PAN_ID_BIT_OFFSET             (16U)
#define DW3000_REG_0_PANADR_PAN_ID_BIT_MASK               (0xFFFF0000UL)

/* Register: SYS_CFG (0x0E) */
#define DW3000_REG_0_SYS_CFG_OFFSET                       0x10
#define DW3000_REG_0_SYS_CFG_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_0_SYS_CFG_FFEN_BIT_OFFSET              (0U)
#define DW3000_REG_0_SYS_CFG_FFEN_BIT_MASK                (0x1U)
#define DW3000_REG_0_SYS_CFG_DIS_FCS_TX_BIT_OFFSET        (1U)
#define DW3000_REG_0_SYS_CFG_DIS_FCS_TX_BIT_MASK          (0x2U)
#define DW3000_REG_0_SYS_CFG_DIS_FCE_BIT_OFFSET           (2U)
#define DW3000_REG_0_SYS_CFG_DIS_FCE_BIT_MASK             (0x4U)
#define DW3000_REG_0_SYS_CFG_DIS_DRXB_BIT_OFFSET          (3U)
#define DW3000_REG_0_SYS_CFG_DIS_DRXB_BIT_MASK            (0x8U)
#define DW3000_REG_0_SYS_CFG_PHR_MODE_BIT_OFFSET          (4U)
#define DW3000_REG_0_SYS_CFG_PHR_MODE_BIT_MASK            (0x10U)
#define DW3000_REG_0_SYS_CFG_PHR_6M8_BIT_OFFSET           (5U)
#define DW3000_REG_0_SYS_CFG_PHR_6M8_BIT_MASK             (0x20U)
#define DW3000_REG_0_SYS_CFG_SPI_CRCEN_BIT_OFFSET         (6U)
#define DW3000_REG_0_SYS_CFG_SPI_CRCEN_BIT_MASK           (0x40U)
#define DW3000_REG_0_SYS_CFG_CIA_IPATOV_BIT_OFFSET        (7U)
#define DW3000_REG_0_SYS_CFG_CIA_IPATOV_BIT_MASK          (0x80U)
#define DW3000_REG_0_SYS_CFG_CIA_STS_BIT_OFFSET           (8U)
#define DW3000_REG_0_SYS_CFG_CIA_STS_BIT_MASK             (0x100U)
#define DW3000_REG_0_SYS_CFG_RXWTOE_BIT_OFFSET            (9U)
#define DW3000_REG_0_SYS_CFG_RXWTOE_BIT_MASK              (0x200U)
#define DW3000_REG_0_SYS_CFG_RXAUTR_BIT_OFFSET            (10U)
#define DW3000_REG_0_SYS_CFG_RXAUTR_BIT_MASK              (0x400U)
#define DW3000_REG_0_SYS_CFG_AUTO_ACK_BIT_OFFSET          (11U)
#define DW3000_REG_0_SYS_CFG_AUTO_ACK_BIT_MASK            (0x800U)
#define DW3000_REG_0_SYS_CFG_CP_SPC_BIT_OFFSET            (12U)
#define DW3000_REG_0_SYS_CFG_CP_SPC_BIT_MASK              (0x3000U)
#define DW3000_REG_0_SYS_CFG_CP_SDC_BIT_OFFSET            (15U)
#define DW3000_REG_0_SYS_CFG_CP_SDC_BIT_MASK              (0x8000U)
#define DW3000_REG_0_SYS_CFG_PDOA_MODE_BIT_OFFSET         (16U)
#define DW3000_REG_0_SYS_CFG_PDOA_MODE_BIT_MASK           (0x30000U)
#define DW3000_REG_0_SYS_CFG_FAST_AAT_BIT_OFFSET          (18U)
#define DW3000_REG_0_SYS_CFG_FAST_AAT_BIT_MASK            (0x40000U)

/* Register: FF_CFG (0x12) */
#define DW3000_REG_0_FF_CFG_OFFSET                         0x14
#define DW3000_REG_0_FF_CFG_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_0_FF_CFG_FFAB_BIT_OFFSET                (0U)
#define DW3000_REG_0_FF_CFG_FFAB_BIT_MASK                  (0x1U)
#define DW3000_REG_0_FF_CFG_FFAD_BIT_OFFSET                (1U)
#define DW3000_REG_0_FF_CFG_FFAD_BIT_MASK                  (0x2U)
#define DW3000_REG_0_FF_CFG_FFAA_BIT_OFFSET                (2U)
#define DW3000_REG_0_FF_CFG_FFAA_BIT_MASK                  (0x4U)
#define DW3000_REG_0_FF_CFG_FFAM_BIT_OFFSET                (3U)
#define DW3000_REG_0_FF_CFG_FFAM_BIT_MASK                  (0x8U)
#define DW3000_REG_0_FF_CFG_FFAR_BIT_OFFSET                (4U)
#define DW3000_REG_0_FF_CFG_FFAR_BIT_MASK                  (0x10U)
#define DW3000_REG_0_FF_CFG_FFAMULTI_BIT_OFFSET            (5U)
#define DW3000_REG_0_FF_CFG_FFAMULTI_BIT_MASK              (0x20U)
#define DW3000_REG_0_FF_CFG_FFAF_BIT_OFFSET                (6U)
#define DW3000_REG_0_FF_CFG_FFAF_BIT_MASK                  (0x40U)
#define DW3000_REG_0_FF_CFG_FFAE_BIT_OFFSET                (7U)
#define DW3000_REG_0_FF_CFG_FFAE_BIT_MASK                  (0x80U)
#define DW3000_REG_0_FF_CFG_FFBC_BIT_OFFSET                (8U)
#define DW3000_REG_0_FF_CFG_FFBC_BIT_MASK                  (0x100U)
#define DW3000_REG_0_FF_CFG_FFIB_BIT_OFFSET                (9U)
#define DW3000_REG_0_FF_CFG_FFIB_BIT_MASK                  (0x200U)
#define DW3000_REG_0_FF_CFG_LE0_PEND_BIT_OFFSET            (10U)
#define DW3000_REG_0_FF_CFG_LE0_PEND_BIT_MASK              (0x400U)
#define DW3000_REG_0_FF_CFG_LE1_PEND_BIT_OFFSET            (11U)
#define DW3000_REG_0_FF_CFG_LE1_PEND_BIT_MASK              (0x800U)
#define DW3000_REG_0_FF_CFG_LE2_PEND_BIT_OFFSET            (12U)
#define DW3000_REG_0_FF_CFG_LE2_PEND_BIT_MASK              (0x1000U)
#define DW3000_REG_0_FF_CFG_LE3_PEND_BIT_OFFSET            (13U)
#define DW3000_REG_0_FF_CFG_LE3_PEND_BIT_MASK              (0x2000U)
#define DW3000_REG_0_FF_CFG_SSADRAPE_BIT_OFFSET            (14U)
#define DW3000_REG_0_FF_CFG_SSADRAPE_BIT_MASK              (0x4000U)
#define DW3000_REG_0_FF_CFG_LSADRAPE_BIT_OFFSET            (15U)
#define DW3000_REG_0_FF_CFG_LSADRAPE_BIT_MASK              (0x8000U)

/* Register: SPI_RD_CRC (0x18) */
#define DW3000_REG_0_SPI_RD_CRC_OFFSET                     0x18
#define DW3000_REG_0_SPI_RD_CRC_BYTE_LEN                   (1U)

/* Register: SYS_TIME (0x1C) */
#define DW3000_REG_0_SYS_TIME_OFFSET                       0x1C
#define DW3000_REG_0_SYS_TIME_BYTE_LEN                     (4U)

/* Register: TX_FCTRL (0x24) */
#define DW3000_REG_0_TX_FCTRL_OFFSET                       0x24
#define DW3000_REG_0_TX_FCTRL_BYTE_LEN                     (6U)
#define DW3000_REG_0_TX_FCTRL_P0_BYTE_OFFSET               (0U)
#define DW3000_REG_0_TX_FCTRL_P1_BYTE_OFFSET               (4U)
/* Fields */
// P1
#define DW3000_REG_0_TX_FCTRL_P0_TXFLEN_BIT_OFFSET         (0U)
#define DW3000_REG_0_TX_FCTRL_P0_TXFLEN_BIT_MASK           (0x3FFU)     
#define DW3000_REG_0_TX_FCTRL_P0_TXBR_BIT_OFFSET           (10U)
#define DW3000_REG_0_TX_FCTRL_P0_TXBR_BIT_MASK             (0x400U)     
#define DW3000_REG_0_TX_FCTRL_P0_TR_BIT_OFFSET             (11U)
#define DW3000_REG_0_TX_FCTRL_P0_TR_BIT_MASK               (0x800U)     
#define DW3000_REG_0_TX_FCTRL_P0_TXPSR_BIT_OFFSET          (12U)
#define DW3000_REG_0_TX_FCTRL_P0_TXPSR_BIT_MASK            (0xF000U)    
#define DW3000_REG_0_TX_FCTRL_P0_TXB_OFFSET_BIT_OFFSET     (16U)
#define DW3000_REG_0_TX_FCTRL_P0_TXB_OFFSET_BIT_MASK       (0x3FF0000U) 
// P2
#define DW3000_REG_0_TX_FCTRL_P1_FINE_PLEN_BIT_OFFSET      (8U)
#define DW3000_REG_0_TX_FCTRL_P1_FINE_PLEN_BIT_MASK        (0xFF00U)   

/* Register: DX_TIME (0x2C) */
#define DW3000_REG_0_DX_TIME_OFFSET                        0x2C
#define DW3000_REG_0_DX_TIME_BYTE_LEN                      (4U)

/* Register: DREF_TIME (0x30) */
#define DW3000_REG_0_DREF_TIME_OFFSET                      0x30
#define DW3000_REG_0_DREF_TIME_BYTE_LEN                    (4U)

/* Register: RX_FWTO (0x34) */
#define DW3000_REG_0_RX_FWTO_OFFSET                        0x34
#define DW3000_REG_0_RX_FWTO_BYTE_LEN                      (3U)

/* Register: SYS_CTRL (0x38) */
#define DW3000_REG_0_SYS_CTRL_OFFSET                       0x38
#define DW3000_REG_0_SYS_CTRL_BYTE_LEN                     (1U)

/* Register: SYS_ENABLE (0x3C) */
#define DW3000_REG_0_SYS_ENABLE_OFFSET                     0x3C
#define DW3000_REG_0_SYS_ENABLE_BYTE_LEN                   (6U)
#define DW3000_REG_0_SYS_ENABLE_P0_BYTE_OFFSET             (0U)
#define DW3000_REG_0_SYS_ENABLE_P1_BYTE_OFFSET             (4U)
/* Fields */
// P0
#define DW3000_REG_0_SYS_ENABLE_P0_CPLOCK_EN_BIT_OFFSET    (1U)
#define DW3000_REG_0_SYS_ENABLE_P0_CPLOCK_EN_BIT_MASK      (0x2U)
#define DW3000_REG_0_SYS_ENABLE_P0_SPICRCE_EN_BIT_OFFSET   (2U)
#define DW3000_REG_0_SYS_ENABLE_P0_SPICRCE_EN_BIT_MASK     (0x4U)
#define DW3000_REG_0_SYS_ENABLE_P0_AAT_EN_BIT_OFFSET       (3U)
#define DW3000_REG_0_SYS_ENABLE_P0_AAT_EN_BIT_MASK         (0x8U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXFRB_EN_BIT_OFFSET     (4U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXFRB_EN_BIT_MASK       (0x10U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXPRS_EN_BIT_OFFSET     (5U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXPRS_EN_BIT_MASK       (0x20U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXPHS_EN_BIT_OFFSET     (6U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXPHS_EN_BIT_MASK       (0x40U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXFRS_EN_BIT_OFFSET     (7U)
#define DW3000_REG_0_SYS_ENABLE_P0_TXFRS_EN_BIT_MASK       (0x80U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPRD_EN_BIT_OFFSET     (8U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPRD_EN_BIT_MASK       (0x100U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXSFDD_EN_BIT_OFFSET    (9U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXSFDD_EN_BIT_MASK      (0x200U)
#define DW3000_REG_0_SYS_ENABLE_P0_CIADONE_EN_BIT_OFFSET   (10U)
#define DW3000_REG_0_SYS_ENABLE_P0_CIADONE_EN_BIT_MASK     (0x400U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPHD_EN_BIT_OFFSET     (11U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPHD_EN_BIT_MASK       (0x800U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPHE_EN_BIT_OFFSET     (12U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPHE_EN_BIT_MASK       (0x1000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFR_EN_BIT_OFFSET      (13U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFR_EN_BIT_MASK        (0x2000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFCG_EN_BIT_OFFSET     (14U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFCG_EN_BIT_MASK       (0x4000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFCE_EN_BIT_OFFSET     (15U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFCE_EN_BIT_MASK       (0x8000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXRFSL_EN_BIT_OFFSET    (16U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXRFSL_EN_BIT_MASK      (0x10000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFTO_EN_BIT_OFFSET     (17U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXFTO_EN_BIT_MASK       (0x20000U)
#define DW3000_REG_0_SYS_ENABLE_P0_CIAERR_EN_BIT_OFFSET    (18U)
#define DW3000_REG_0_SYS_ENABLE_P0_CIAERR_EN_BIT_MASK      (0x40000U)
#define DW3000_REG_0_SYS_ENABLE_P0_VWARN_EN_BIT_OFFSET     (19U)
#define DW3000_REG_0_SYS_ENABLE_P0_VWARN_EN_BIT_MASK       (0x80000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXOVRR_EN_BIT_OFFSET    (20U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXOVRR_EN_BIT_MASK      (0x100000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPTO_EN_BIT_OFFSET     (21U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXPTO_EN_BIT_MASK       (0x200000U)
#define DW3000_REG_0_SYS_ENABLE_P0_SPIRDY_EN_BIT_OFFSET    (23U)
#define DW3000_REG_0_SYS_ENABLE_P0_SPIRDY_EN_BIT_MASK      (0x800000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RCINIT_EN_BIT_OFFSET    (24U)
#define DW3000_REG_0_SYS_ENABLE_P0_RCINIT_EN_BIT_MASK      (0x1000000U)
#define DW3000_REG_0_SYS_ENABLE_P0_PLL_HILO_EN_BIT_OFFSET  (25U)
#define DW3000_REG_0_SYS_ENABLE_P0_PLL_HILO_EN_BIT_MASK    (0x2000000U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXSTO_EN_BIT_OFFSET     (26U)
#define DW3000_REG_0_SYS_ENABLE_P0_RXSTO_EN_BIT_MASK       (0x4000000U)
#define DW3000_REG_0_SYS_ENABLE_P0_HPDWARN_EN_BIT_OFFSET   (27U)
#define DW3000_REG_0_SYS_ENABLE_P0_HPDWARN_EN_BIT_MASK     (0x8000000U)
#define DW3000_REG_0_SYS_ENABLE_P0_CPERR_EN_BIT_OFFSET     (28U)
#define DW3000_REG_0_SYS_ENABLE_P0_CPERR_EN_BIT_MASK       (0x10000000U)
#define DW3000_REG_0_SYS_ENABLE_P0_ARFE_EN_BIT_OFFSET      (29U)
#define DW3000_REG_0_SYS_ENABLE_P0_ARFE_EN_BIT_MASK        (0x20000000U)
// P1
#define DW3000_REG_0_SYS_ENABLE_P1_RXPREJ_EN_BIT_OFFSET    (1U)
#define DW3000_REG_0_SYS_ENABLE_P1_RXPREJ_EN_BIT_MASK      (0x2U)
#define DW3000_REG_0_SYS_ENABLE_P1_VT_DET_EN_BIT_OFFSET    (4U)
#define DW3000_REG_0_SYS_ENABLE_P1_VT_DET_EN_BIT_MASK      (0x10U)
#define DW3000_REG_0_SYS_ENABLE_P1_GPIOIRQ_EN_BIT_OFFSET   (5U)
#define DW3000_REG_0_SYS_ENABLE_P1_GPIOIRQ_EN_BIT_MASK     (0x20U)
#define DW3000_REG_0_SYS_ENABLE_P1_AES_DONE_EN_BIT_OFFSET  (6U)
#define DW3000_REG_0_SYS_ENABLE_P1_AES_DONE_EN_BIT_MASK    (0x40U)
#define DW3000_REG_0_SYS_ENABLE_P1_AES_ERR_EN_BIT_OFFSET   (7U)
#define DW3000_REG_0_SYS_ENABLE_P1_AES_ERR_EN_BIT_MASK     (0x80U)
#define DW3000_REG_0_SYS_ENABLE_P1_CMD_ERR_EN_BIT_OFFSET   (8U)
#define DW3000_REG_0_SYS_ENABLE_P1_CMD_ERR_EN_BIT_MASK     (0x100U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPI_OVF_EN_BIT_OFFSET   (9U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPI_OVF_EN_BIT_MASK     (0x200U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPI_UNF_EN_BIT_OFFSET   (10U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPI_UNF_EN_BIT_MASK     (0x400U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPIERR_EN_BIT_OFFSET    (11U)
#define DW3000_REG_0_SYS_ENABLE_P1_SPIERR_EN_BIT_MASK      (0x800U)
#define DW3000_REG_0_SYS_ENABLE_P1_CCA_FAIL_EN_BIT_OFFSET  (12U)
#define DW3000_REG_0_SYS_ENABLE_P1_CCA_FAIL_EN_BIT_MASK    (0x1000U)

/* Register: SYS_STATUS (0x44) */
#define DW3000_REG_0_SYS_STATUS_OFFSET                     0x44
#define DW3000_REG_0_SYS_STATUS_BYTE_LEN                   (6U)
#define DW3000_REG_0_SYS_STATUS_P0_BYTE_OFFSET             (0U)
#define DW3000_REG_0_SYS_STATUS_P1_BYTE_OFFSET             (4U)
/* Fields */
// P0
#define DW3000_REG_0_SYS_STATUS_P0_IRQS_BIT_OFFSET         (0U)
#define DW3000_REG_0_SYS_STATUS_P0_IRQS_BIT_MASK           (0x1U)
#define DW3000_REG_0_SYS_STATUS_P0_CPLOCK_BIT_OFFSET       (1U)
#define DW3000_REG_0_SYS_STATUS_P0_CPLOCK_BIT_MASK         (0x2U)
#define DW3000_REG_0_SYS_STATUS_P0_SPICRCE_BIT_OFFSET      (2U)
#define DW3000_REG_0_SYS_STATUS_P0_SPICRCE_BIT_MASK        (0x4U)
#define DW3000_REG_0_SYS_STATUS_P0_AAT_BIT_OFFSET          (3U)
#define DW3000_REG_0_SYS_STATUS_P0_AAT_BIT_MASK            (0x8U)
#define DW3000_REG_0_SYS_STATUS_P0_TXFRB_BIT_OFFSET        (4U)
#define DW3000_REG_0_SYS_STATUS_P0_TXFRB_BIT_MASK          (0x10U)
#define DW3000_REG_0_SYS_STATUS_P0_TXPRS_BIT_OFFSET        (5U)
#define DW3000_REG_0_SYS_STATUS_P0_TXPRS_BIT_MASK          (0x20U)
#define DW3000_REG_0_SYS_STATUS_P0_TXPHS_BIT_OFFSET        (6U)
#define DW3000_REG_0_SYS_STATUS_P0_TXPHS_BIT_MASK          (0x40U)
#define DW3000_REG_0_SYS_STATUS_P0_TXFRS_BIT_OFFSET        (7U)
#define DW3000_REG_0_SYS_STATUS_P0_TXFRS_BIT_MASK          (0x80U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPRD_BIT_OFFSET        (8U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPRD_BIT_MASK          (0x100U)
#define DW3000_REG_0_SYS_STATUS_P0_RXSFDD_BIT_OFFSET       (9U)
#define DW3000_REG_0_SYS_STATUS_P0_RXSFDD_BIT_MASK         (0x200U)
#define DW3000_REG_0_SYS_STATUS_P0_CIADONE_BIT_OFFSET      (10U)
#define DW3000_REG_0_SYS_STATUS_P0_CIADONE_BIT_MASK        (0x400U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPHD_BIT_OFFSET        (11U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPHD_BIT_MASK          (0x800U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPHE_BIT_OFFSET        (12U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPHE_BIT_MASK          (0x1000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFR_BIT_OFFSET         (13U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFR_BIT_MASK           (0x2000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFCG_BIT_OFFSET        (14U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFCG_BIT_MASK          (0x4000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFCE_BIT_OFFSET        (15U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFCE_BIT_MASK          (0x8000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXRFSL_BIT_OFFSET       (16U)
#define DW3000_REG_0_SYS_STATUS_P0_RXRFSL_BIT_MASK         (0x10000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFTO_BIT_OFFSET        (17U)
#define DW3000_REG_0_SYS_STATUS_P0_RXFTO_BIT_MASK          (0x20000U)
#define DW3000_REG_0_SYS_STATUS_P0_CIAERR_BIT_OFFSET       (18U)
#define DW3000_REG_0_SYS_STATUS_P0_CIAERR_BIT_MASK         (0x40000U)
#define DW3000_REG_0_SYS_STATUS_P0_VWARN_BIT_OFFSET        (19U)
#define DW3000_REG_0_SYS_STATUS_P0_VWARN_BIT_MASK          (0x80000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXOVRR_BIT_OFFSET       (20U)
#define DW3000_REG_0_SYS_STATUS_P0_RXOVRR_BIT_MASK         (0x100000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPTO_BIT_OFFSET        (21U)
#define DW3000_REG_0_SYS_STATUS_P0_RXPTO_BIT_MASK          (0x200000U)
#define DW3000_REG_0_SYS_STATUS_P0_SPIRDY_BIT_OFFSET       (23U)
#define DW3000_REG_0_SYS_STATUS_P0_SPIRDY_BIT_MASK         (0x800000U)
#define DW3000_REG_0_SYS_STATUS_P0_RCINIT_BIT_OFFSET       (24U)
#define DW3000_REG_0_SYS_STATUS_P0_RCINIT_BIT_MASK         (0x1000000U)
#define DW3000_REG_0_SYS_STATUS_P0_PLLHILO_BIT_OFFSET      (25U)
#define DW3000_REG_0_SYS_STATUS_P0_PLLHILO_BIT_MASK        (0x2000000U)
#define DW3000_REG_0_SYS_STATUS_P0_RXSTO_BIT_OFFSET        (26U)
#define DW3000_REG_0_SYS_STATUS_P0_RXSTO_BIT_MASK          (0x4000000U)
#define DW3000_REG_0_SYS_STATUS_P0_HPDWARN_BIT_OFFSET      (27U)
#define DW3000_REG_0_SYS_STATUS_P0_HPDWARN_BIT_MASK        (0x8000000U)
#define DW3000_REG_0_SYS_STATUS_P0_CPERR_BIT_OFFSET        (28U)
#define DW3000_REG_0_SYS_STATUS_P0_CPERR_BIT_MASK          (0x10000000U)
#define DW3000_REG_0_SYS_STATUS_P0_ARFE_BIT_OFFSET         (29U)
#define DW3000_REG_0_SYS_STATUS_P0_ARFE_BIT_MASK           (0x20000000U)
// P1
#define DW3000_REG_0_SYS_STATUS_P1_RXPREJ_BIT_OFFSET       (1U)
#define DW3000_REG_0_SYS_STATUS_P1_RXPREJ_BIT_MASK         (0x2U)
#define DW3000_REG_0_SYS_STATUS_P1_VT_DET_BIT_OFFSET       (4U)
#define DW3000_REG_0_SYS_STATUS_P1_VT_DET_BIT_MASK         (0x10U)
#define DW3000_REG_0_SYS_STATUS_P1_GPIOIRQ_BIT_OFFSET      (5U)
#define DW3000_REG_0_SYS_STATUS_P1_GPIOIRQ_BIT_MASK        (0x20U)
#define DW3000_REG_0_SYS_STATUS_P1_AES_DONE_BIT_OFFSET     (6U)
#define DW3000_REG_0_SYS_STATUS_P1_AES_DONE_BIT_MASK       (0x40U)
#define DW3000_REG_0_SYS_STATUS_P1_AES_ERR_BIT_OFFSET      (7U)
#define DW3000_REG_0_SYS_STATUS_P1_AES_ERR_BIT_MASK        (0x80U)
#define DW3000_REG_0_SYS_STATUS_P1_CMD_ERR_BIT_OFFSET      (8U)
#define DW3000_REG_0_SYS_STATUS_P1_CMD_ERR_BIT_MASK        (0x100U)
#define DW3000_REG_0_SYS_STATUS_P1_SPI_OVR_BIT_OFFSET      (9U)
#define DW3000_REG_0_SYS_STATUS_P1_SPI_OVR_BIT_MASK        (0x200U)
#define DW3000_REG_0_SYS_STATUS_P1_SPI_UNF_BIT_OFFSET      (10U)
#define DW3000_REG_0_SYS_STATUS_P1_SPI_UNF_BIT_MASK        (0x400U)
#define DW3000_REG_0_SYS_STATUS_P1_SPIERR_BIT_OFFSET       (11U)
#define DW3000_REG_0_SYS_STATUS_P1_SPIERR_BIT_MASK         (0x800U)
#define DW3000_REG_0_SYS_STATUS_P1_CCA_FAIL_BIT_OFFSET     (12U)
#define DW3000_REG_0_SYS_STATUS_P1_CCA_FAIL_BIT_MASK       (0x1000U)

/* Register: RX_FINFO (0x4C) */
#define DW3000_REG_0_RX_FINFO_OFFSET                        0x4C
#define DW3000_REG_0_RX_FINFO_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_0_RX_FINFO_RXFLEN_BIT_OFFSET             (0U)
#define DW3000_REG_0_RX_FINFO_RXFLEN_BIT_MASK               (0x3FFU)
#define DW3000_REG_0_RX_FINFO_RXNSPL_BIT_OFFSET             (11U)
#define DW3000_REG_0_RX_FINFO_RXNSPL_BIT_MASK               (0x1800U)
#define DW3000_REG_0_RX_FINFO_RXBR_BIT_OFFSET               (13U)
#define DW3000_REG_0_RX_FINFO_RXBR_BIT_MASK                 (0x2000U)
#define DW3000_REG_0_RX_FINFO_RNG_BIT_OFFSET                (15U)
#define DW3000_REG_0_RX_FINFO_RNG_BIT_MASK                  (0x8000U)
#define DW3000_REG_0_RX_FINFO_RXPRF_BIT_OFFSET              (16U)
#define DW3000_REG_0_RX_FINFO_RXPRF_BIT_MASK                (0x30000U)
#define DW3000_REG_0_RX_FINFO_RXPSR_BIT_OFFSET              (18U)
#define DW3000_REG_0_RX_FINFO_RXPSR_BIT_MASK                (0xC0000U)
#define DW3000_REG_0_RX_FINFO_RXPACC_BIT_OFFSET             (20U)
#define DW3000_REG_0_RX_FINFO_RXPACC_BIT_MASK               (0xFFF00000U)

/* Register: RX_TIME (0x64) */
#define DW3000_REG_0_RX_TIME_OFFSET                         0x64
#define DW3000_REG_0_RX_TIME_BYTE_LEN                       (16U)
#define DW3000_REG_0_RX_TIME_P0_BYTE_OFFSET                 (0U)
#define DW3000_REG_0_RX_TIME_P1_BYTE_OFFSET                 (4U)
#define DW3000_REG_0_RX_TIME_P2_BYTE_OFFSET                 (8U)
/* Fields */
// P0
#define DW3000_REG_0_RX_TIME_P0_RX_STAMP_BIT_OFFSET         (0U)
#define DW3000_REG_0_RX_TIME_P0_RX_STAMP_BIT_MASK           (0xFFFFFFFFUL)
// P1
#define DW3000_REG_0_RX_TIME_P1_RX_STAMP_BIT_OFFSET         (0U)
#define DW3000_REG_0_RX_TIME_P1_RX_STAMP_BIT_MASK           (0xFFU)
// P2
#define DW3000_REG_0_RX_TIME_P2_RX_RAWST_BIT_OFFSET         (0U)
#define DW3000_REG_0_RX_TIME_P2_RX_RAWST_BIT_MASK           (0xFFFFFFFFUL)

/* Register: TX_TIME (0x74) */
#define DW3000_REG_0_TX_TIME_OFFSET                         0x74
#define DW3000_REG_0_TX_TIME_BYTE_LEN                       (5U)
#define DW3000_REG_0_TX_TIME_P0_BYTE_OFFSET                 (0U)
#define DW3000_REG_0_TX_TIME_P1_BYTE_OFFSET                 (4U)
/* Fields */
// P0
#define DW3000_REG_0_TX_TIME_P0_TX_STAMP_BIT_OFFSET         (0U)
#define DW3000_REG_0_TX_TIME_P0_TX_STAMP_BIT_MASK           (0xFFFFFFFFUL)
// P1
#define DW3000_REG_0_TX_TIME_P1_TX_STAMP_BIT_OFFSET         (0U)
#define DW3000_REG_0_TX_TIME_P1_TX_STAMP_BIT_MASK           (0xFFU)

/******************************************************************************
* @brief Bit definitions for register page 0x01 : GEN_CFG_AES HIGH
**/
#define DW3000_REG_1_ADDR                                 0x01

/* Register: TX_RAWST (0x00) */
#define DW3000_REG_1_TX_RAWST_OFFSET                        0x00
#define DW3000_REG_1_TX_RAWST_BYTE_LEN                      (4U)

/* Register: TX_ANTD (0x04) */
#define DW3000_REG_1_TX_ANTD_OFFSET                         0x04
#define DW3000_REG_1_TX_ANTD_BYTE_LEN                       (2U)

/* Register: ACK_RESP_T (0x08) */
#define DW3000_REG_1_ACK_RESP_T_OFFSET                      0x08
#define DW3000_REG_1_ACK_RESP_T_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_1_ACK_RESP_T_W4R_TIM_BIT_OFFSET          (0U)
#define DW3000_REG_1_ACK_RESP_T_W4R_TIM_BIT_MASK            (0xFFFFFU)
#define DW3000_REG_1_ACK_RESP_T_ACK_TIM_BIT_OFFSET          (24U)
#define DW3000_REG_1_ACK_RESP_T_ACK_TIM_BIT_MASK            (0xFF000000U)

/* Register: TX_POWER (0x0C) */
#define DW3000_REG_1_TX_POWER_OFFSET                        0x0C
#define DW3000_REG_1_TX_POWER_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_1_TX_POWER_DATA_PWR_BIT_OFFSET           (0U)
#define DW3000_REG_1_TX_POWER_DATA_PWR_BIT_MASK             (0xFFU)
#define DW3000_REG_1_TX_POWER_PHR_PWR_BIT_OFFSET            (8U)
#define DW3000_REG_1_TX_POWER_PHR_PWR_BIT_MASK              (0xFF00U)
#define DW3000_REG_1_TX_POWER_SHR_PWR_BIT_OFFSET            (16U)
#define DW3000_REG_1_TX_POWER_SHR_PWR_BIT_MASK              (0xFF0000U)
#define DW3000_REG_1_TX_POWER_STS_PWR_BIT_OFFSET            (24U)
#define DW3000_REG_1_TX_POWER_STS_PWR_BIT_MASK              (0xFF000000U)

/* Register: CHAN_CTRL (0x14) */
#define DW3000_REG_1_CHAN_CTRL_OFFSET                       0x14
#define DW3000_REG_1_CHAN_CTRL_BYTE_LEN                     (2U)
/* Fields */
#define DW3000_REG_1_CHAN_CTRL_RF_CHAN_BIT_OFFSET           (0U)
#define DW3000_REG_1_CHAN_CTRL_RF_CHAN_BIT_MASK             (0x1U)
#define DW3000_REG_1_CHAN_CTRL_SFD_TYPE_BIT_OFFSET          (1U)
#define DW3000_REG_1_CHAN_CTRL_SFD_TYPE_BIT_MASK            (0x6U)
#define DW3000_REG_1_CHAN_CTRL_TX_PCODE_OFFSET              (3U)
#define DW3000_REG_1_CHAN_CTRL_TX_PCODE_BIT_MASK            (0xF8U)
#define DW3000_REG_1_CHAN_CTRL_RX_PCODE_OFFSET              (8U)
#define DW3000_REG_1_CHAN_CTRL_RX_PCODE_BIT_MASK            (0x1F00U)

/* Register: LE_PEND_01 (0x18) */
#define DW3000_REG_1_LE_PEND_01_OFFSET                      0x18
#define DW3000_REG_1_LE_PEND_01_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_1_LE_PEND_01_LE_ADDR0_BIT_OFFSET         (0U)
#define DW3000_REG_1_LE_PEND_01_LE_ADDR0_BIT_MASK           (0xFFFFU)
#define DW3000_REG_1_LE_PEND_01_LE_ADDR1_BIT_OFFSET         (16U)
#define DW3000_REG_1_LE_PEND_01_LE_ADDR1_BIT_MASK           (0xFFFF0000UL)

/* Register: LE_PEND_23 (0x1C) */
#define DW3000_REG_1_LE_PEND_23_OFFSET                      0x1C
#define DW3000_REG_1_LE_PEND_23_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_1_LE_PEND_23_LE_ADDR2_BIT_OFFSET         (0U)
#define DW3000_REG_1_LE_PEND_23_LE_ADDR2_BIT_MASK           (0xFFFFU)
#define DW3000_REG_1_LE_PEND_23_LE_ADDR3_BIT_OFFSET         (16U)
#define DW3000_REG_1_LE_PEND_23_LE_ADDR3_BIT_MASK           (0xFFFF0000UL)

/* Register: SPI_COLLISION (0x20) */
#define DW3000_REG_1_SPI_COLLISION_OFFSET                   0x20
#define DW3000_REG_1_SPI_COLLISION_BYTE_LEN                 (1U)
/* Fields */
#define DW3000_REG_1_SPI_COLLISION_SPI_COLLISION_BIT_OFFSET (0U)
#define DW3000_REG_1_SPI_COLLISION_SPI_COLLISION_BIT_MASK   (0x1FU)

/* Register: RDB_STATUS (0x24) */
#define DW3000_REG_1_RDB_STATUS_OFFSET                      0x24
#define DW3000_REG_1_RDB_STATUS_BYTE_LEN                    (1U)
/* Fields */
#define DW3000_REG_1_RDB_STATUS_RXFCG0_BIT_OFFSET           (0U)
#define DW3000_REG_1_RDB_STATUS_RXFCG0_BIT_MASK             (0x1U)
#define DW3000_REG_1_RDB_STATUS_RXFR0_BIT_OFFSET            (1U)
#define DW3000_REG_1_RDB_STATUS_RXFR0_BIT_MASK              (0x2U)
#define DW3000_REG_1_RDB_STATUS_CIADONE0_BIT_OFFSET         (2U)
#define DW3000_REG_1_RDB_STATUS_CIADONE0_BIT_MASK           (0x4U)
#define DW3000_REG_1_RDB_STATUS_CP_ERR0_BIT_OFFSET          (3U)
#define DW3000_REG_1_RDB_STATUS_CP_ERR0_BIT_MASK            (0x8U)
#define DW3000_REG_1_RDB_STATUS_RXFCG1_BIT_OFFSET           (4U)
#define DW3000_REG_1_RDB_STATUS_RXFCG1_BIT_MASK             (0x10U)
#define DW3000_REG_1_RDB_STATUS_RXFR1_BIT_OFFSET            (5U)
#define DW3000_REG_1_RDB_STATUS_RXFR1_BIT_MASK              (0x20U)
#define DW3000_REG_1_RDB_STATUS_CIADONE1_BIT_OFFSET         (6U)
#define DW3000_REG_1_RDB_STATUS_CIADONE1_BIT_MASK           (0x40U)
#define DW3000_REG_1_RDB_STATUS_CP_ERR1_BIT_OFFSET          (7U)
#define DW3000_REG_1_RDB_STATUS_CP_ERR1_BIT_MASK            (0x80U)

/* Register: RDB_DIAG (0x28) */
#define DW3000_REG_1_RDB_DIAG_OFFSET                        0x28
#define DW3000_REG_1_RDB_DIAG_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_1_RDB_DIAG_RDB_DMODE_BIT_OFFSET          (0U)
#define DW3000_REG_1_RDB_DIAG_RDB_DMODE_BIT_MASK            (0x3U)

/* Register: AES_CFG (0x30) */
#define DW3000_REG_1_AES_CFG_OFFSET                         0x30
#define DW3000_REG_1_AES_CFG_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_1_AES_CFG_MODE_BIT_OFFSET                (0U)
#define DW3000_REG_1_AES_CFG_MODE_BIT_MASK                  (0x1U)
#define DW3000_REG_1_AES_CFG_KEY_SIZE_BIT_OFFSET            (1U)
#define DW3000_REG_1_AES_CFG_KEY_SIZE_BIT_MASK              (0x6U)
#define DW3000_REG_1_AES_CFG_KEY_ADDR_BIT_OFFSET            (3U)
#define DW3000_REG_1_AES_CFG_KEY_ADDR_BIT_MASK              (0x38U)
#define DW3000_REG_1_AES_CFG_KEY_LOAD_BIT_OFFSET            (6U)
#define DW3000_REG_1_AES_CFG_KEY_LOAD_BIT_MASK              (0x40U)
#define DW3000_REG_1_AES_CFG_KEY_SRC_BIT_OFFSET             (7U)
#define DW3000_REG_1_AES_CFG_KEY_SRC_BIT_MASK               (0x80U)
#define DW3000_REG_1_AES_CFG_TAG_SIZE_BIT_OFFSET            (8U)
#define DW3000_REG_1_AES_CFG_TAG_SIZE_BIT_MASK              (0x700U)
#define DW3000_REG_1_AES_CFG_CORE_SEL_BIT_OFFSET            (11U)
#define DW3000_REG_1_AES_CFG_CORE_SEL_BIT_MASK              (0x800U)
#define DW3000_REG_1_AES_CFG_KEY_OTP_BIT_OFFSET             (12U)
#define DW3000_REG_1_AES_CFG_KEY_OTP_BIT_MASK               (0x1000U)

/* Register: AES_IV0 (0x34) */
#define DW3000_REG_1_AES_IV0_OFFSET                         0x34
#define DW3000_REG_1_AES_IV0_BYTE_LEN                       (4U)

/* Register: AES_IV1 (0x38) */
#define DW3000_REG_1_AES_IV1_OFFSET                         0x38
#define DW3000_REG_1_AES_IV1_BYTE_LEN                       (4U)

/* Register: AES_IV2 (0x3C) */
#define DW3000_REG_1_AES_IV2_OFFSET                         0x3C
#define DW3000_REG_1_AES_IV2_BYTE_LEN                       (4U)

/* Register: AES_IV3 (0x40) */
#define DW3000_REG_1_AES_IV3_OFFSET                         0x40
#define DW3000_REG_1_AES_IV3_BYTE_LEN                       (2U)

/* Register: AES_IV4 (0x42) */
#define DW3000_REG_1_AES_IV4_OFFSET                         0x42
#define DW3000_REG_1_AES_IV4_BYTE_LEN                       (2U)

/* Register: DMA_CFG (0x44) */
#define DW3000_REG_1_DMA_CFG_OFFSET                         0x44
#define DW3000_REG_1_DMA_CFG_BYTE_LEN                       (8U)
#define DW3000_REG_1_DMA_CFG_P0_BYTE_OFFSET                 (0U)
#define DW3000_REG_1_DMA_CFG_P1_BYTE_OFFSET                 (4U)
/* Fields */
// P0
#define DW3000_REG_1_P0_DMA_CFG_SRC_PORT_BIT_OFFSET         (0U)
#define DW3000_REG_1_P0_DMA_CFG_SRC_PORT_BIT_MASK           (0x7U)          
#define DW3000_REG_1_P0_DMA_CFG_SRC_ADDR_BIT_OFFSET         (3U)
#define DW3000_REG_1_P0_DMA_CFG_SRC_ADDR_BIT_MASK           (0x1FF8U)       
#define DW3000_REG_1_P0_DMA_CFG_DST_PORT_BIT_OFFSET         (13U)
#define DW3000_REG_1_P0_DMA_CFG_DST_PORT_BIT_MASK           (0xE000U)       
#define DW3000_REG_1_P0_DMA_CFG_DST_ADDR_BIT_OFFSET         (16U)
#define DW3000_REG_1_P0_DMA_CFG_DST_ADDR_BIT_MASK           (0x3FF0000U)    
#define DW3000_REG_1_P0_DMA_CFG_CP_END_SEL_BIT_OFFSET       (26U)
#define DW3000_REG_1_P0_DMA_CFG_CP_END_SEL_BIT_MASK         (0x4000000U)    
// P1
#define DW3000_REG_1_P1_DMA_CFG_HDR_SIZE_BIT_OFFSET         (0U)
#define DW3000_REG_1_P1_DMA_CFG_HDR_SIZE_BIT_MASK           (0x7FU)
#define DW3000_REG_1_P1_DMA_CFG_PYLD_SIZE_BIT_OFFSET        (7U)
#define DW3000_REG_1_P1_DMA_CFG_PYLD_SIZE_BIT_MASK          (0x1FF80U)

/* Register: AES_START (0x4C) */
#define DW3000_REG_1_AES_START_OFFSET                       0x4C
#define DW3000_REG_1_AES_START_BYTE_LEN                     (1U)
/* Fields */
#define DW3000_REG_1_AES_START_AES_START_BIT_OFFSET         (0U)
#define DW3000_REG_1_AES_START_AES_START_BIT_MASK           (0x1U)

/* Register: AES_STS (0x50) */
#define DW3000_REG_1_AES_STS_OFFSET                         0x50
#define DW3000_REG_1_AES_STS_BYTE_LEN                       (4U)
/* Fields */
#define DW3000_REG_1_AES_STS_AES_DONE_BIT_OFFSET            (0U)
#define DW3000_REG_1_AES_STS_AES_DONE_BIT_MASK              (0x1U)
#define DW3000_REG_1_AES_STS_AUTH_ERR_BIT_OFFSET            (1U)
#define DW3000_REG_1_AES_STS_AUTH_ERR_BIT_MASK              (0x2U)
#define DW3000_REG_1_AES_STS_TRANS_ERR_BIT_OFFSET           (2U)
#define DW3000_REG_1_AES_STS_TRANS_ERR_BIT_MASK             (0x4U)
#define DW3000_REG_1_AES_STS_MEM_CONF_BIT_OFFSET            (3U)
#define DW3000_REG_1_AES_STS_MEM_CONF_BIT_MASK              (0x8U)
#define DW3000_REG_1_AES_STS_RAM_EMPTY_BIT_OFFSET           (4U)
#define DW3000_REG_1_AES_STS_RAM_EMPTY_BIT_MASK             (0x10U)
#define DW3000_REG_1_AES_STS_RAM_FULL_BIT_OFFSET            (5U)
#define DW3000_REG_1_AES_STS_RAM_FULL_BIT_MASK              (0x20U)

/* Register: AES_KEY (0x54) */
#define DW3000_REG_1_AES_KEY_OFFSET                         0x54
#define DW3000_REG_1_AES_KEY_BYTE_LEN                       (16U)

/******************************************************************************
* @brief Bit definitions for register page 0x02 : STS_CONFIG
* @note  Scrambled Timestamp Sequence configuration and status registers
**/
#define DW3000_REG_2_ADDR                                 0x02

/* Register: STS_CFG (0x00) */
#define DW3000_REG_2_STS_CFG_OFFSET                         0x00
#define DW3000_REG_2_STS_CFG_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_2_STS_CFG_STS_CPS_LEN_BIT_OFFSET         (0U)
#define DW3000_REG_2_STS_CFG_STS_CPS_LEN_BIT_MASK           (0xFFU)

/* Register: STS_CTRL (0x04) */
#define DW3000_REG_2_STS_CTRL_OFFSET                        0x04
#define DW3000_REG_2_STS_CTRL_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_2_STS_CTRL_STS_LOAD_IV_BIT_OFFSET        (0U)
#define DW3000_REG_2_STS_CTRL_STS_LOAD_IV_BIT_MASK          (0x1U)
#define DW3000_REG_2_STS_CTRL_STS_RST_LAST_BIT_OFFSET       (1U)
#define DW3000_REG_2_STS_CTRL_STS_RST_LAST_BIT_MASK         (0x2U)

/* Register: STS_STS (0x08) */
#define DW3000_REG_2_STS_STS_OFFSET                         0x08
#define DW3000_REG_2_STS_STS_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_2_STS_STS_ACC_QUAL_BIT_OFFSET            (0U)
#define DW3000_REG_2_STS_STS_ACC_QUAL_BIT_MASK              (0xFFFU)

/* Register: STS_KEY (0x0C) */
#define DW3000_REG_2_STS_KEY_OFFSET                         0x0C
#define DW3000_REG_2_STS_KEY_BYTE_LEN                       (16U)

/* Register: STS_IV (0x1C) */
#define DW3000_REG_2_STS_IV_OFFSET                          0x1C
#define DW3000_REG_2_STS_IV_BYTE_LEN                        (16U)

/******************************************************************************
* @brief Bit definitions for register page 0x03 : RX_TUNE
* @note  Receiver tuning parameters
**/
#define DW3000_REG_3_ADDR                                   0x03

/* Register: DGC_CFG (0x00) */
#define DW3000_REG_3_DGC_CFG_OFFSET                         0x00
#define DW3000_REG_3_DGC_CFG_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_3_DGC_CFG_RX_TUNE_EN_BIT_OFFSET          (0U)
#define DW3000_REG_3_DGC_CFG_RX_TUNE_EN_BIT_MASK            (0x1U)
#define DW3000_REG_3_DGC_CFG_THR_64_BIT_OFFSET              (9U)
#define DW3000_REG_3_DGC_CFG_THR_64_BIT_MASK                (0x7E00U)

/* Register: DGC_CFG0 (0x1C) */
#define DW3000_REG_3_DGC_CFG0_OFFSET                        0x1C
#define DW3000_REG_3_DGC_CFG0_BYTE_LEN                      (4U)

/* Register: DGC_CFG1 (0x20) */
#define DW3000_REG_3_DGC_CFG1_OFFSET                        0x20
#define DW3000_REG_3_DGC_CFG1_BYTE_LEN                      (4U)

/* Register: DGC_LUT_0 (0x38) */
#define DW3000_REG_3_DGC_LUT_0_OFFSET                       0x38
#define DW3000_REG_3_DGC_LUT_0_BYTE_LEN                     (4U)

/* Register: DGC_LUT_1 (0x3C) */
#define DW3000_REG_3_DGC_LUT_1_OFFSET                       0x3C
#define DW3000_REG_3_DGC_LUT_1_BYTE_LEN                     (4U)

/* Register: DGC_LUT_2 (0x40) */
#define DW3000_REG_3_DGC_LUT_2_OFFSET                       0x40
#define DW3000_REG_3_DGC_LUT_2_BYTE_LEN                     (4U)

/* Register: DGC_LUT_3 (0x44) */
#define DW3000_REG_3_DGC_LUT_3_OFFSET                       0x44
#define DW3000_REG_3_DGC_LUT_3_BYTE_LEN                     (4U)

/* Register: DGC_LUT_4 (0x48) */
#define DW3000_REG_3_DGC_LUT_4_OFFSET                       0x48
#define DW3000_REG_3_DGC_LUT_4_BYTE_LEN                     (4U)

/* Register: DGC_LUT_5 (0x4C) */
#define DW3000_REG_3_DGC_LUT_5_OFFSET                       0x4C
#define DW3000_REG_3_DGC_LUT_5_BYTE_LEN                     (4U)

/* Register: DGC_LUT_6 (0x50) */
#define DW3000_REG_3_DGC_LUT_6_OFFSET                       0x50
#define DW3000_REG_3_DGC_LUT_6_BYTE_LEN                     (4U)

/* Register: DGC_DBG (0x60) */
#define DW3000_REG_3_DGC_DBG_OFFSET                         0x60
#define DW3000_REG_3_DGC_DBG_BYTE_LEN                       (4U)
/* Fields */
#define DW3000_REG_3_DGC_DBG_DGC_DECISION_BIT_OFFSET        (28U)
#define DW3000_REG_3_DGC_DBG_DGC_DECISION_BIT_MASK          (0x70000000U)

/******************************************************************************
* @brief Bit definitions for register page 0x04 : EXT_SYNC
* @note  External synchronisation control and RX calibration
**/
#define DW3000_REG_4_ADDR                                   0x04

/* Register: EC_CTRL (0x00) */
#define DW3000_REG_4_EC_CTRL_OFFSET                         0x00
#define DW3000_REG_4_EC_CTRL_BYTE_LEN                       (4U)
/* Fields */
#define DW3000_REG_4_EC_CTRL_OSTS_WAIT_BIT_OFFSET           (3U)
#define DW3000_REG_4_EC_CTRL_OSTS_WAIT_BIT_MASK             (0x7F8U)
#define DW3000_REG_4_EC_CTRL_OSTR_MODE_BIT_OFFSET           (11U)
#define DW3000_REG_4_EC_CTRL_OSTR_MODE_BIT_MASK             (0x800U)

/* Register: RX_CAL (0x0C) */
#define DW3000_REG_4_RX_CAL_OFFSET                          0x0C
#define DW3000_REG_4_RX_CAL_BYTE_LEN                        (4U)
/* Fields */
#define DW3000_REG_4_RX_CAL_CAL_MODE_BIT_OFFSET             (0U)
#define DW3000_REG_4_RX_CAL_CAL_MODE_BIT_MASK               (0x3U)
#define DW3000_REG_4_RX_CAL_CAL_EN_BIT_OFFSET               (4U)
#define DW3000_REG_4_RX_CAL_CAL_EN_BIT_MASK                 (0x10U)
#define DW3000_REG_4_RX_CAL_COMP_DLY_BIT_OFFSET             (16U)
#define DW3000_REG_4_RX_CAL_COMP_DLY_BIT_MASK               (0xF0000U)

/* Register: RX_CAL_RESI (0x14) */
#define DW3000_REG_4_RX_CAL_RESI_OFFSET                     0x14
#define DW3000_REG_4_RX_CAL_RESI_BYTE_LEN                   (4U)

/* Register: RX_CAL_RESQ (0x1C) */
#define DW3000_REG_4_RX_CAL_RESQ_OFFSET                     0x1C
#define DW3000_REG_4_RX_CAL_RESQ_BYTE_LEN                   (4U)

/* Register: RX_CAL_STS (0x20) */
#define DW3000_REG_4_RX_CAL_STS_OFFSET                      0x20
#define DW3000_REG_4_RX_CAL_STS_BYTE_LEN                    (1U)

/******************************************************************************
* @brief Bit definitions for register page 0x05 : GPIO_CTRL
* @note  General Purpose Input-Output control registers
**/
#define DW3000_REG_5_ADDR                                   0x05

/* Register: GPIO_MODE (0x00) - GPIO mode control register */
#define DW3000_REG_5_GPIO_MODE_OFFSET                       0x00
#define DW3000_REG_5_GPIO_MODE_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_5_GPIO_MODE_MSGP0_BIT_OFFSET             (0U)
#define DW3000_REG_5_GPIO_MODE_MSGP0_BIT_MASK               (0x7U)
#define DW3000_REG_5_GPIO_MODE_MSGP1_BIT_OFFSET             (3U)
#define DW3000_REG_5_GPIO_MODE_MSGP1_BIT_MASK               (0x38U)
#define DW3000_REG_5_GPIO_MODE_MSGP2_BIT_OFFSET             (6U)
#define DW3000_REG_5_GPIO_MODE_MSGP2_BIT_MASK               (0x1C0U)
#define DW3000_REG_5_GPIO_MODE_MSGP3_BIT_OFFSET             (9U)
#define DW3000_REG_5_GPIO_MODE_MSGP3_BIT_MASK               (0xE00U)
#define DW3000_REG_5_GPIO_MODE_MSGP4_BIT_OFFSET             (12U)
#define DW3000_REG_5_GPIO_MODE_MSGP4_BIT_MASK               (0x7000U)
#define DW3000_REG_5_GPIO_MODE_MSGP5_BIT_OFFSET             (15U)
#define DW3000_REG_5_GPIO_MODE_MSGP5_BIT_MASK               (0x38000U)
#define DW3000_REG_5_GPIO_MODE_MSGP6_BIT_OFFSET             (18U)
#define DW3000_REG_5_GPIO_MODE_MSGP6_BIT_MASK               (0x1C0000U)
#define DW3000_REG_5_GPIO_MODE_MSGP7_BIT_OFFSET             (21U)
#define DW3000_REG_5_GPIO_MODE_MSGP7_BIT_MASK               (0xE00000U)
#define DW3000_REG_5_GPIO_MODE_MSGP8_BIT_OFFSET             (24U)
#define DW3000_REG_5_GPIO_MODE_MSGP8_BIT_MASK               (0x7000000U)

//TODO: Add the rest of the registers


/******************************************************************************
* @brief Bit definitions for register page 0x06 : DRX_CONF
* @note  Digital receiver configuration
**/
#define DW3000_REG_6_ADDR                                   0x06

/* Register: DTUNE (0x00) */
#define DW3000_REG_6_DTUNE_OFFSET                           0x00
#define DW3000_REG_6_DTUNE_BYTE_LEN                         (2U)
/* Fields */
#define DW3000_REG_6_DTUNE_PAC_BIT_OFFSET                   (0U)
#define DW3000_REG_6_DTUNE_PAC_BIT_MASK                     (0x3U)
#define DW3000_REG_6_DTUNE_DT0B4_BIT_OFFSET                 (4U)
#define DW3000_REG_6_DTUNE_DT0B4_BIT_MASK                   (0x10U)

/* Register: RX_SFD_TOC (0x02) */
#define DW3000_REG_6_RX_SFD_TOC_OFFSET                      0x02
#define DW3000_REG_6_RX_SFD_TOC_BYTE_LEN                    (2U)

/* Register: PRE_TOC (0x04) */
#define DW3000_REG_6_PRE_TOC_OFFSET                         0x04
#define DW3000_REG_6_PRE_TOC_BYTE_LEN                       (2U)

/* Register: DTUNE3 (0x0C) */
#define DW3000_REG_6_DTUNE3_OFFSET                          0x0C
#define DW3000_REG_6_DTUNE3_BYTE_LEN                        (4U)

/* Register: DTUNE_5 (0x14) */
#define DW3000_REG_6_DTUNE_5_OFFSET                         0x14
#define DW3000_REG_6_DTUNE_5_BYTE_LEN                       (4U)

/* Register: DRX_CAR_INT (0x29) */
#define DW3000_REG_6_DRX_CAR_INT_OFFSET                     0x29
#define DW3000_REG_6_DRX_CAR_INT_BYTE_LEN                   (3U)

/******************************************************************************
* @brief Bit definitions for register page 0x07 : RF_CONF
* @note  Analog RF configuration
**/
#define DW3000_REG_7_ADDR                                   0x07

/* Register: RF_ENABLE (0x00) */
#define DW3000_REG_7_RF_ENABLE_OFFSET                       0x00
#define DW3000_REG_7_RF_ENABLE_BYTE_LEN                     (4U)

/* Register: RF_CTRL_MASK (0x04) */
#define DW3000_REG_7_RF_CTRL_MASK_OFFSET                    0x04
#define DW3000_REG_7_RF_CTRL_MASK_BYTE_LEN                  (4U)

/* Register: RF_SWITCH (0x14) */
#define DW3000_REG_7_RF_SWITCH_OFFSET                       0x14
#define DW3000_REG_7_RF_SWITCH_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_7_RF_SWITCH_ANTSWNOTOGGLE_BIT_OFFSET     (0U)
#define DW3000_REG_7_RF_SWITCH_ANTSWNOTOGGLE_BIT_MASK       (0x1U)
#define DW3000_REG_7_RF_SWITCH_ANTSWPDOAPORT_BIT_OFFSET     (1U)
#define DW3000_REG_7_RF_SWITCH_ANTSWPDOAPORT_BIT_MASK       (0x2U)
#define DW3000_REG_7_RF_SWITCH_ANTSWEN_BIT_OFFSET           (8U)
#define DW3000_REG_7_RF_SWITCH_ANTSWEN_BIT_MASK             (0x100U)
#define DW3000_REG_7_RF_SWITCH_ANTSWCTRL_BIT_OFFSET         (12U)
#define DW3000_REG_7_RF_SWITCH_ANTSWCTRL_BIT_MASK           (0x7000U)
#define DW3000_REG_7_RF_SWITCH_TRXSWEN_BIT_OFFSET           (16U)
#define DW3000_REG_7_RF_SWITCH_TRXSWEN_BIT_MASK             (0x10000U)
#define DW3000_REG_7_RF_SWITCH_TRXSWCTRL_BIT_OFFSET         (24U)
#define DW3000_REG_7_RF_SWITCH_TRXSWCTRL_BIT_MASK           (0x3F000000U)

/* Register: RF_TX_CTRL_1 (0x1A) */
#define DW3000_REG_7_RF_TX_CTRL_1_OFFSET                    0x1A
#define DW3000_REG_7_RF_TX_CTRL_1_BYTE_LEN                  (1U)

/* Register: RF_TX_CTRL_2 (0x1C) */
#define DW3000_REG_7_RF_TX_CTRL_2_OFFSET                    0x1C
#define DW3000_REG_7_RF_TX_CTRL_2_BYTE_LEN                  (4U)
/* Fields */
#define DW3000_REG_7_RF_TX_CTRL_2_PG_DELAY_BIT_OFFSET       (0U)
#define DW3000_REG_7_RF_TX_CTRL_2_PG_DELAY_BIT_MASK         (0x3FU)

/* Register: TX_TEST (0x28) */
#define DW3000_REG_7_TX_TEST_OFFSET                         0x28
#define DW3000_REG_7_TX_TEST_BYTE_LEN                       (1U)
/* Fields */
#define DW3000_REG_7_TX_TEST_TX_ENTEST_BIT_OFFSET           (0U)
#define DW3000_REG_7_TX_TEST_TX_ENTEST_BIT_MASK             (0xFU)

/* Register: SAR_TEST (0x34) */
#define DW3000_REG_7_SAR_TEST_OFFSET                        0x34
#define DW3000_REG_7_SAR_TEST_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_7_SAR_TEST_SAR_RDEN_BIT_OFFSET           (2U)
#define DW3000_REG_7_SAR_TEST_SAR_RDEN_BIT_MASK             (0x4U)

/* Register: LDO_TUNE (0x40) */
#define DW3000_REG_7_LDO_TUNE_OFFSET                        0x40
#define DW3000_REG_7_LDO_TUNE_BYTE_LEN                      (8U)
#define DW3000_REG_7_LDO_TUNE_P0_BYTE_OFFSET                (0U)
#define DW3000_REG_7_LDO_TUNE_P1_BYTE_OFFSET                (4U)
/* Fields */
// P0
#define DW3000_REG_7_LDO_TUNE_P0_LDO_TUNE_BIT_OFFSET        (0U)
#define DW3000_REG_7_LDO_TUNE_P0_LDO_TUNE_BIT_MASK          (0xFFFFFFFFU)
// P1
#define DW3000_REG_7_LDO_TUNE_P1_LDO_TUNE_BIT_OFFSET        (0U)
#define DW3000_REG_7_LDO_TUNE_P1_LDO_TUNE_BIT_MASK          (0xFFFFFFFU)

/* Register: LDO_CTRL (0x48) */
#define DW3000_REG_7_LDO_CTRL_OFFSET                        0x48
#define DW3000_REG_7_LDO_CTRL_BYTE_LEN                      (4U)

/* Register: LDO_RLOAD (0x51) */
#define DW3000_REG_7_LDO_RLOAD_OFFSET                       0x51
#define DW3000_REG_7_LDO_RLOAD_BYTE_LEN                     (1U)

/******************************************************************************
* @brief Bit definitions for register page 0x08 : TX_CAL
* @note  Transmitter calibration block
**/
#define DW3000_REG_8_ADDR                                   0x08

/* Register: SAR_CTRL (0x00) */
#define DW3000_REG_8_SAR_CTRL_OFFSET                        0x00
#define DW3000_REG_8_SAR_CTRL_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_8_SAR_CTRL_SAR_START_BIT_OFFSET          (0U)
#define DW3000_REG_8_SAR_CTRL_SAR_START_BIT_MASK            (0x1U)

/* Register: SAR_STATUS (0x04) */
#define DW3000_REG_8_SAR_STATUS_OFFSET                      0x04
#define DW3000_REG_8_SAR_STATUS_BYTE_LEN                    (1U)
/* Fields */
#define DW3000_REG_8_SAR_STATUS_SAR_DONE_BIT_OFFSET         (0U)
#define DW3000_REG_8_SAR_STATUS_SAR_DONE_BIT_MASK           (0x1U)

/* Register: SAR_READING (0x08) */
#define DW3000_REG_8_SAR_READING_OFFSET                     0x08
#define DW3000_REG_8_SAR_READING_BYTE_LEN                   (3U)
/* Fields */
#define DW3000_REG_8_SAR_READING_SAR_LVBAT_BIT_OFFSET       (0U)
#define DW3000_REG_8_SAR_READING_SAR_LVBAT_BIT_MASK         (0xFFU)
#define DW3000_REG_8_SAR_READING_SAR_LTEMP_BIT_OFFSET       (8U)
#define DW3000_REG_8_SAR_READING_SAR_LTEMP_BIT_MASK         (0xFF00U)

/* Register: SAR_WAKE_RD (0x0C) */
#define DW3000_REG_8_SAR_WAKE_RD_OFFSET                     0x0C
#define DW3000_REG_8_SAR_WAKE_RD_BYTE_LEN                   (2U)
/* Fields */
#define DW3000_REG_8_SAR_WAKE_RD_SAR_WVBAT_BIT_OFFSET       (0U)
#define DW3000_REG_8_SAR_WAKE_RD_SAR_WVBAT_BIT_MASK         (0xFFU)
#define DW3000_REG_8_SAR_WAKE_RD_SAR_WTEMP_BIT_OFFSET       (8U)
#define DW3000_REG_8_SAR_WAKE_RD_SAR_WTEMP_BIT_MASK         (0xFF00U)

/* Register: PGC_CTRL (0x10) */
#define DW3000_REG_8_PGC_CTRL_OFFSET                        0x10
#define DW3000_REG_8_PGC_CTRL_BYTE_LEN                      (2U)
/* Fields */
#define DW3000_REG_8_PGC_CTRL_PGC_START_BIT_OFFSET          (0U)
#define DW3000_REG_8_PGC_CTRL_PGC_START_BIT_MASK            (0x1U)
#define DW3000_REG_8_PGC_CTRL_PGC_AUTO_CAL_BIT_OFFSET       (1U)
#define DW3000_REG_8_PGC_CTRL_PGC_AUTO_CAL_BIT_MASK         (0x2U)
#define DW3000_REG_8_PGC_CTRL_PGC_TMEAS_BIT_OFFSET          (2U)
#define DW3000_REG_8_PGC_CTRL_PGC_TMEAS_BIT_MASK            (0x3CU)

/* Register: PGC_STATUS (0x14) */
#define DW3000_REG_8_PGC_STATUS_OFFSET                      0x14
#define DW3000_REG_8_PGC_STATUS_BYTE_LEN                    (2U)
/* Fields */
#define DW3000_REG_8_PGC_STATUS_PG_DELAY_CNT_BIT_OFFSET     (0U)
#define DW3000_REG_8_PGC_STATUS_PG_DELAY_CNT_BIT_MASK       (0xFFFU)
#define DW3000_REG_8_PGC_STATUS_PAUTOCAL_DONE_BIT_OFFSET    (12U)
#define DW3000_REG_8_PGC_STATUS_PAUTOCAL_DONE_BIT_MASK      (0x1000U)

/* Register: PG_TEST (0x18) */
#define DW3000_REG_8_PG_TEST_OFFSET                         0x18
#define DW3000_REG_8_PG_TEST_BYTE_LEN                       (2U)

/* Register: PG_CAL_TARGET (0x1C) */
#define DW3000_REG_8_PG_CAL_TARGET_OFFSET                   0x1C
#define DW3000_REG_8_PG_CAL_TARGET_BYTE_LEN                 (2U)
/* Fields */
#define DW3000_REG_8_PG_CAL_TARGET_PG_TARGET_BIT_OFFSET     (0U)
#define DW3000_REG_8_PG_CAL_TARGET_PG_TARGET_BIT_MASK       (0xFFFU)

/******************************************************************************
* @brief Bit definitions for register page 0x09 : FS_CTRL
* @note  Frequency synthesiser control block
**/
#define DW3000_REG_9_ADDR                                   0x09

/* Register: PLL_CFG (0x00) */
#define DW3000_REG_9_PLL_CFG_OFFSET                         0x00
#define DW3000_REG_9_PLL_CFG_BYTE_LEN                       (2U)

/* Register: PLL_CC (0x04) */
#define DW3000_REG_9_PLL_CC_OFFSET                          0x04
#define DW3000_REG_9_PLL_CC_BYTE_LEN                        (1U)
/* Fields */
#define DW3000_REG_9_PLL_CC_CH9_CODE_BIT_OFFSET             (0U)
#define DW3000_REG_9_PLL_CC_CH9_CODE_BIT_MASK               (0xFFU)
#define DW3000_REG_9_PLL_CC_CH5_CODE_BIT_OFFSET             (8U)
#define DW3000_REG_9_PLL_CC_CH5_CODE_BIT_MASK               (0x3FFF00U)

/* Register: PLL_CAL (0x08) */
#define DW3000_REG_9_PLL_CAL_OFFSET                         0x08
#define DW3000_REG_9_PLL_CAL_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_9_PLL_CAL_USE_OLD_BIT_OFFSET             (1U)
#define DW3000_REG_9_PLL_CAL_USE_OLD_BIT_MASK               (0x2U)
#define DW3000_REG_9_PLL_CAL_PLL_CFG_LD_BIT_OFFSET          (4U)
#define DW3000_REG_9_PLL_CAL_PLL_CFG_LD_BIT_MASK            (0xF0U)
#define DW3000_REG_9_PLL_CAL_CAL_EN_BIT_OFFSET              (8U)
#define DW3000_REG_9_PLL_CAL_CAL_EN_BIT_MASK                (0x100U)

/* Register: XTAL (0x14) */
#define DW3000_REG_9_XTAL_OFFSET                            0x14
#define DW3000_REG_9_XTAL_BYTE_LEN                          (1U)
/* Fields */
#define DW3000_REG_9_XTAL_XTAL_TRIM_BIT_OFFSET              (0U)
#define DW3000_REG_9_XTAL_XTAL_TRIM_BIT_MASK                (0x3FU)

/******************************************************************************
* @brief Bit definitions for register page 0x0A : AON
* @note  Always on system control interface block
**/
#define DW3000_REG_10_ADDR                                   0x0A

/* Register: AON_DIG_CFG (0x00) */
#define DW3000_REG_10_AON_DIG_CFG_OFFSET                     0x00
#define DW3000_REG_10_AON_DIG_CFG_BYTE_LEN                   (3U)
/* Fields */
#define DW3000_REG_10_AON_DIG_CFG_ONW_AON_DLD_BIT_OFFSET     (0U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_AON_DLD_BIT_MASK       (0x1U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_RUN_SAR_BIT_OFFSET     (1U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_RUN_SAR_BIT_MASK       (0x2U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_GO2IDLE_BIT_OFFSET     (8U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_GO2IDLE_BIT_MASK       (0x100U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_GO2RX_BIT_OFFSET       (9U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_GO2RX_BIT_MASK         (0x200U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_PGFCAL_BIT_OFFSET      (11U)
#define DW3000_REG_10_AON_DIG_CFG_ONW_PGFCAL_BIT_MASK        (0x800U)

/* Register: AON_CTRL (0x04) */
#define DW3000_REG_10_AON_CTRL_OFFSET                        0x04
#define DW3000_REG_10_AON_CTRL_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_10_AON_CTRL_RESTORE_BIT_OFFSET            (0U)
#define DW3000_REG_10_AON_CTRL_RESTORE_BIT_MASK              (0x1U)
#define DW3000_REG_10_AON_CTRL_SAVE_BIT_OFFSET               (1U)
#define DW3000_REG_10_AON_CTRL_SAVE_BIT_MASK                 (0x2U)
#define DW3000_REG_10_AON_CTRL_CFG_UPLOAD_BIT_OFFSET         (2U)
#define DW3000_REG_10_AON_CTRL_CFG_UPLOAD_BIT_MASK           (0x4U)
#define DW3000_REG_10_AON_CTRL_DCA_READ_BIT_OFFSET           (3U)
#define DW3000_REG_10_AON_CTRL_DCA_READ_BIT_MASK             (0x8U)
#define DW3000_REG_10_AON_CTRL_DCA_WRITE_BIT_OFFSET          (4U)
#define DW3000_REG_10_AON_CTRL_DCA_WRITE_BIT_MASK            (0x10U)
#define DW3000_REG_10_AON_CTRL_DCA_WRITE_HI_BIT_OFFSET       (5U)
#define DW3000_REG_10_AON_CTRL_DCA_WRITE_HI_BIT_MASK         (0x20U)
#define DW3000_REG_10_AON_CTRL_DCA_ENAB_BIT_OFFSET           (7U)
#define DW3000_REG_10_AON_CTRL_DCA_ENAB_BIT_MASK             (0x80U)

/* Register: AON_RDATA (0x08) */
#define DW3000_REG_10_AON_RDATA_OFFSET                       0x08
#define DW3000_REG_10_AON_RDATA_BYTE_LEN                     (1U)

/* Register: AON_ADDR (0x0C) */
#define DW3000_REG_10_AON_ADDR_OFFSET                        0x0C
#define DW3000_REG_10_AON_ADDR_BYTE_LEN                      (2U)

/* Register: AON_WDATA (0x10) */
#define DW3000_REG_10_AON_WDATA_OFFSET                       0x10
#define DW3000_REG_10_AON_WDATA_BYTE_LEN                     (1U)

/* Register: AON_CFG (0x14) */
#define DW3000_REG_10_AON_CFG_OFFSET                         0x14
#define DW3000_REG_10_AON_CFG_BYTE_LEN                       (1U)
/* Fields */
#define DW3000_REG_10_AON_CFG_SLEEP_EN_BIT_OFFSET            (0U)
#define DW3000_REG_10_AON_CFG_SLEEP_EN_BIT_MASK              (0x1U)
#define DW3000_REG_10_AON_CFG_WAKE_CNT_BIT_OFFSET            (1U)
#define DW3000_REG_10_AON_CFG_WAKE_CNT_BIT_MASK              (0x2U)
#define DW3000_REG_10_AON_CFG_BROUT_EN_BIT_OFFSET            (2U)
#define DW3000_REG_10_AON_CFG_BROUT_EN_BIT_MASK              (0x4U)
#define DW3000_REG_10_AON_CFG_WAKE_CSN_BIT_OFFSET            (3U)
#define DW3000_REG_10_AON_CFG_WAKE_CSN_BIT_MASK              (0x8U)
#define DW3000_REG_10_AON_CFG_WAKE_WUP_BIT_OFFSET            (4U)
#define DW3000_REG_10_AON_CFG_WAKE_WUP_BIT_MASK              (0x10U)
#define DW3000_REG_10_AON_CFG_PRES_SLEEP_BIT_OFFSET          (5U)
#define DW3000_REG_10_AON_CFG_PRES_SLEEP_BIT_MASK            (0x20U)

/******************************************************************************
* @brief Bit definitions for register page 0x0B : OTP_IF
* @note  One Time Programmable memory interface
**/
#define DW3000_REG_11_ADDR                                   0x0B

/* Register:  OTP_WDATA (0x00) */
#define DW3000_REG_11_OTP_WDATA_OFFSET                       0x00
#define DW3000_REG_11_OTP_WDATA_BYTE_LEN                     (4U)

/* Register:  OTP_ADDR (0x04) */
#define DW3000_REG_11_OTP_ADDR_OFFSET                        0x04
#define DW3000_REG_11_OTP_ADDR_BYTE_LEN                      (2U)
/* Fields */
#define DW3000_REG_11_OTP_ADDR_OTP_ADDR_BIT_OFFSET           (0U)
#define DW3000_REG_11_OTP_ADDR_OTP_ADDR_BIT_MASK             (0x3FFU)

/* Register:  OTP_CFG (0x08) */
#define DW3000_REG_11_OTP_CFG_OFFSET                         0x08
#define DW3000_REG_11_OTP_CFG_BYTE_LEN                       (2U)
/* Fields */
#define DW3000_REG_11_OTP_CFG_OTP_MAN_BIT_OFFSET             (0U)
#define DW3000_REG_11_OTP_CFG_OTP_MAN_BIT_MASK               (0x1U)
#define DW3000_REG_11_OTP_CFG_OTP_READ_BIT_OFFSET            (1U)
#define DW3000_REG_11_OTP_CFG_OTP_READ_BIT_MASK              (0x2U)
#define DW3000_REG_11_OTP_CFG_OTP_WRITE_BIT_OFFSET           (2U)
#define DW3000_REG_11_OTP_CFG_OTP_WRITE_BIT_MASK             (0x4U)
#define DW3000_REG_11_OTP_CFG_OTP_WRITE_MR_BIT_OFFSET        (3U)
#define DW3000_REG_11_OTP_CFG_OTP_WRITE_MR_BIT_MASK          (0x8U)
#define DW3000_REG_11_OTP_CFG_DGC_KICK_BIT_OFFSET            (6U)
#define DW3000_REG_11_OTP_CFG_DGC_KICK_BIT_MASK              (0x40U)
#define DW3000_REG_11_OTP_CFG_LDO_KICK_BIT_OFFSET            (7U)
#define DW3000_REG_11_OTP_CFG_LDO_KICK_BIT_MASK              (0x80U)
#define DW3000_REG_11_OTP_CFG_BIAS_KICK_BIT_OFFSET           (8U)       // missing in the manual
#define DW3000_REG_11_OTP_CFG_BIAS_KICK_BIT_MASK             (0x100U)
#define DW3000_REG_11_OTP_CFG_OPS_KICK_BIT_OFFSET            (10U)
#define DW3000_REG_11_OTP_CFG_OPS_KICK_BIT_MASK              (0x400U)
#define DW3000_REG_11_OTP_CFG_OPS_SEL_BIT_OFFSET             (11U)
#define DW3000_REG_11_OTP_CFG_OPS_SEL_BIT_MASK               (0x1800U)
#define DW3000_REG_11_OTP_CFG_DGC_SEL_BIT_OFFSET             (13U)
#define DW3000_REG_11_OTP_CFG_DGC_SEL_BIT_MASK               (0x2000U)

/* Register:  OTP_STAT (0x0C) */
#define DW3000_REG_11_OTP_STAT_OFFSET                        0x0C
#define DW3000_REG_11_OTP_STAT_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_11_OTP_STAT_OTP_PROG_DONE_BIT_OFFSET      (0U)
#define DW3000_REG_11_OTP_STAT_OTP_PROG_DONE_BIT_MASK        (0x1U)
#define DW3000_REG_11_OTP_STAT_OTP_VPP_OK_BIT_OFFSET         (1U)
#define DW3000_REG_11_OTP_STAT_OTP_VPP_OK_BIT_MASK           (0x2U)

/* Register:  OTP_RDATA (0x10) */
#define DW3000_REG_11_OTP_RDATA_OFFSET                       0x10
#define DW3000_REG_11_OTP_RDATA_BYTE_LEN                     (4U)

/* Register:  OTP_SRDATA (0x14) */
#define DW3000_REG_11_OTP_SRDATA_OFFSET                      0x14
#define DW3000_REG_11_OTP_SRDATA_BYTE_LEN                    (4U)

/******************************************************************************
* @brief Bit definitions for register page 0x0C : CIA_IF p1
* @note  Channel Impulse Response Analyser (CIA) Interface part 1
**/
#define DW3000_REG_12_ADDR                                   0x0C

/* Register:  IP_TS (0x00) */
#define DW3000_REG_12_IP_TS_OFFSET                           0x00
#define DW3000_REG_12_IP_TS_BYTE_LEN                         (8U)
#define DW3000_REG_12_IP_TS_P0_BYTE_OFFSET                   (0U)
#define DW3000_REG_12_IP_TS_P1_BYTE_OFFSET                   (4U)
/* Fields */
// P0
#define DW3000_REG_12_IP_TS_P0_IP_TOA_BIT_OFFSET             (0U)
#define DW3000_REG_12_IP_TS_P0_IP_TOA_BIT_MASK               (0xFFFFFFFFU)
// P1
#define DW3000_REG_12_IP_TS_P1_IP_TOA_BIT_OFFSET             (0U)
#define DW3000_REG_12_IP_TS_P1_IP_TOA_BIT_MASK               (0xFFU)
#define DW3000_REG_12_IP_TS_P1_IP_POA_BIT_OFFSET             (8U)
#define DW3000_REG_12_IP_TS_P1_IP_POA_BIT_MASK               (0x3FFF00U)
#define DW3000_REG_12_IP_TS_P1_IP_TOAST_BIT_OFFSET           (24U)
#define DW3000_REG_12_IP_TS_P1_IP_TOAST_BIT_MASK             (0xFF000000U)

/* Register:  STS_TS (0x08) */
#define DW3000_REG_12_STS_TS_OFFSET                          0x08
#define DW3000_REG_12_STS_TS_BYTE_LEN                        (8U)
#define DW3000_REG_12_STS_TS_P0_BYTE_OFFSET                  (0U)
#define DW3000_REG_12_STS_TS_P1_BYTE_OFFSET                  (4U)
/* Fields */
// P0
#define DW3000_REG_12_STS_TS_P0_STS_TOA_BIT_OFFSET           (0U)
#define DW3000_REG_12_STS_TS_P0_STS_TOA_BIT_MASK             (0xFFFFFFFFU)
// P1
#define DW3000_REG_12_STS_TS_P1_STS_TOA_BIT_OFFSET           (0U)
#define DW3000_REG_12_STS_TS_P1_STS_TOA_BIT_MASK             (0xFFU)
#define DW3000_REG_12_STS_TS_P1_STS_POA_BIT_OFFSET           (8U)
#define DW3000_REG_12_STS_TS_P1_STS_POA_BIT_MASK             (0x3FFF00U)
#define DW3000_REG_12_STS_TS_P1_STS_TOAST_BIT_OFFSET         (23U)
#define DW3000_REG_12_STS_TS_P1_STS_TOAST_BIT_MASK           (0xFF800000U)

/* Register:  STS1_TS (0x10) */
#define DW3000_REG_12_STS1_TS_OFFSET                         0x10
#define DW3000_REG_12_STS1_TS_BYTE_LEN                       (8U)
#define DW3000_REG_12_STS1_TS_P0_BYTE_OFFSET                 (0U)
#define DW3000_REG_12_STS1_TS_P1_BYTE_OFFSET                 (4U)
/* Fields */
// P0
#define DW3000_REG_12_STS1_TS_P0_STS1_TOA_BIT_OFFSET         (0U)
#define DW3000_REG_12_STS1_TS_P0_STS1_TOA_BIT_MASK           (0xFFFFFFFFU)
// P1
#define DW3000_REG_12_STS1_TS_P1_STS1_TOA_BIT_OFFSET         (0U)
#define DW3000_REG_12_STS1_TS_P1_STS1_TOA_BIT_MASK           (0xFFU)
#define DW3000_REG_12_STS1_TS_P1_STS1_POA_BIT_OFFSET         (8U)
#define DW3000_REG_12_STS1_TS_P1_STS1_POA_BIT_MASK           (0x3FFF00U)
#define DW3000_REG_12_STS1_TS_P1_STS1_TOAST_BIT_OFFSET       (23U)
#define DW3000_REG_12_STS1_TS_P1_STS1_TOAST_BIT_MASK         (0xFF800000U)

/* Register:  TDOA (0x18) */
#define DW3000_REG_12_TDOA_OFFSET                            0x18
#define DW3000_REG_12_TDOA_BYTE_LEN                          (6U)

/* Register:  PDOA (0x1E) */
#define DW3000_REG_12_PDOA_OFFSET                            0x1E
#define DW3000_REG_12_PDOA_BYTE_LEN                          (2U)
/* Fields */
#define DW3000_REG_12_PDOA_PDOA_BIT_OFFSET                   (0U)
#define DW3000_REG_12_PDOA_PDOA_BIT_MASK                     (0x3FFFU)
#define DW3000_REG_12_PDOA_FP_TH_MD_BIT_OFFSET               (14U)
#define DW3000_REG_12_PDOA_FP_TH_MD_BIT_MASK                 (0x4000U)

/* Register:  CIA_DIAG_0 (0x20) */
#define DW3000_REG_12_CIA_DIAG_0_OFFSET                      0x20
#define DW3000_REG_12_CIA_DIAG_0_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_CIA_DIAG_0_COE_PPM_BIT_OFFSET          (0U)
#define DW3000_REG_12_CIA_DIAG_0_COE_PPM_BIT_MASK            (0x1FFU)

/* Register:  CIA_DIAG_1 (0x24) */
#define DW3000_REG_12_CIA_DIAG_1_OFFSET                      0x24
#define DW3000_REG_12_CIA_DIAG_1_BYTE_LEN                    (4U)

/* Register:  IP_DIAG_0 (0x28) */
#define DW3000_REG_12_IP_DIAG_0_OFFSET                       0x28
#define DW3000_REG_12_IP_DIAG_0_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_0_IP_PEAKA_BIT_OFFSET          (0U)
#define DW3000_REG_12_IP_DIAG_0_IP_PEAKA_BIT_MASK            (0x1FFFFFU)
#define DW3000_REG_12_IP_DIAG_0_IP_PEAKI_BIT_OFFSET          (24U)
#define DW3000_REG_12_IP_DIAG_0_IP_PEAKI_BIT_MASK            (0x7F000000U)

/* Register:  IP_DIAG_1 (0x2C) */
#define DW3000_REG_12_IP_DIAG_1_OFFSET                       0x2C
#define DW3000_REG_12_IP_DIAG_1_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_1_IP_CAREA_BIT_OFFSET          (0U)
#define DW3000_REG_12_IP_DIAG_1_IP_CAREA_BIT_MASK            (0x1FFFFU)

/* Register:  IP_DIAG_2 (0x30) */
#define DW3000_REG_12_IP_DIAG_2_OFFSET                       0x30
#define DW3000_REG_12_IP_DIAG_2_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_2_IP_FP1M_BIT_OFFSET           (0U)
#define DW3000_REG_12_IP_DIAG_2_IP_FP1M_BIT_MASK             (0x3FFFFFU)

/* Register:  IP_DIAG_3 (0x34) */
#define DW3000_REG_12_IP_DIAG_3_OFFSET                       0x34
#define DW3000_REG_12_IP_DIAG_3_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_3_IP_FP2M_BIT_OFFSET           (0U)
#define DW3000_REG_12_IP_DIAG_3_IP_FP2M_BIT_MASK             (0x3FFFFFU)

/* Register:  IP_DIAG_4 (0x38) */
#define DW3000_REG_12_IP_DIAG_4_OFFSET                       0x38
#define DW3000_REG_12_IP_DIAG_4_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_4_IP_FP3M_BIT_OFFSET           (0U)
#define DW3000_REG_12_IP_DIAG_4_IP_FP3M_BIT_MASK             (0x3FFFFFU)

/* Register:  IP_DIAG_RES1 (0x3C) */
#define DW3000_REG_12_IP_DIAG_RES1_OFFSET                    0x3C
#define DW3000_REG_12_IP_DIAG_RES1_BYTE_LEN                  (12U)

/* Register:  IP_DIAG_8 (0x48) */
#define DW3000_REG_12_IP_DIAG_8_OFFSET                       0x48
#define DW3000_REG_12_IP_DIAG_8_BYTE_LEN                     (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_8_IP_FP_BIT_OFFSET             (0U)
#define DW3000_REG_12_IP_DIAG_8_IP_FP_BIT_MASK               (0xFFFFU)

/* Register:  IP_DIAG_RES2 (0x4C) */
#define DW3000_REG_12_IP_DIAG_RES2_OFFSET                    0x4C
#define DW3000_REG_12_IP_DIAG_RES2_BYTE_LEN                  (12U)

/* Register:  IP_DIAG_12 (0x58) */
#define DW3000_REG_12_IP_DIAG_12_OFFSET                      0x58
#define DW3000_REG_12_IP_DIAG_12_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_IP_DIAG_12_IP_NACC_BIT_OFFSET          (0U)
#define DW3000_REG_12_IP_DIAG_12_IP_NACC_BIT_MASK            (0xFFU)

/* Register:  STS_DIAG_0 (0x5C) */
#define DW3000_REG_12_STS_DIAG_0_OFFSET                      0x5C
#define DW3000_REG_12_STS_DIAG_0_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_STS_DIAG_0_CP_PEAKA_BIT_OFFSET         (0U)
#define DW3000_REG_12_STS_DIAG_0_CP_PEAKA_BIT_MASK           (0x1FFFFFU)
#define DW3000_REG_12_STS_DIAG_0_CP_PEAKI_BIT_OFFSET         (24U)
#define DW3000_REG_12_STS_DIAG_0_CP_PEAKI_BIT_MASK           (0x3F000000U)

/* Register:  STS_DIAG_1 (0x60) */
#define DW3000_REG_12_STS_DIAG_1_OFFSET                      0x60
#define DW3000_REG_12_STS_DIAG_1_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_STS_DIAG_1_CP_CAREA_BIT_OFFSET         (0U)
#define DW3000_REG_12_STS_DIAG_1_CP_CAREA_BIT_MASK           (0xFFFFU)

/* Register:  STS_DIAG_2 (0x64) */
#define DW3000_REG_12_STS_DIAG_2_OFFSET                      0x64
#define DW3000_REG_12_STS_DIAG_2_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_STS_DIAG_2_CP_FP1M_BIT_OFFSET          (0U)
#define DW3000_REG_12_STS_DIAG_2_CP_FP1M_BIT_MASK            (0x3FFFFFU)

/* Register:  STS_DIAG_3 (0x68) */
#define DW3000_REG_12_STS_DIAG_3_OFFSET                      0x68
#define DW3000_REG_12_STS_DIAG_3_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_12_STS_DIAG_3_CP_FP2M_BIT_OFFSET          (0U)
#define DW3000_REG_12_STS_DIAG_3_CP_FP2M_BIT_MASK            (0x3FFFFFU)

/******************************************************************************
* @brief Bit definitions for register page 0x0D : CIA_IF p2
* @note  Channel Impulse Response Analyser (CIA) Interface part 2
**/
#define DW3000_REG_13_ADDR                                   0x0D

/* Register:  STS_DIAG_4 (0x00) */
#define DW3000_REG_13_STS_DIAG_4_OFFSET                      0x00
#define DW3000_REG_13_STS_DIAG_4_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_13_STS_DIAG_4_CP_FP3M_BIT_OFFSET          (0U)
#define DW3000_REG_13_STS_DIAG_4_CP_FP3M_BIT_MASK            (0x3FFFFFU)

/* Register:  STS_DIAG_RES1 (0x04) */
#define DW3000_REG_13_STS_DIAG_RES1_OFFSET                   0x04
#define DW3000_REG_13_STS_DIAG_RES1_BYTE_LEN                 (12U)

/* Register:  STS_DIAG_8 (0x10) */
#define DW3000_REG_13_STS_DIAG_8_OFFSET                      0x10
#define DW3000_REG_13_STS_DIAG_8_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_13_STS_DIAG_8_CP_FP_BIT_OFFSET            (0U)
#define DW3000_REG_13_STS_DIAG_8_CP_FP_BIT_MASK              (0x7FFFU)

/* Register:  STS_DIAG_RES2 (0x14) */
#define DW3000_REG_13_STS_DIAG_RES2_OFFSET                   0x14
#define DW3000_REG_13_STS_DIAG_RES2_BYTE_LEN                 (12U)

/* Register:  STS_DIAG_12 (0x20) */
#define DW3000_REG_13_STS_DIAG_12_OFFSET                     0x20
#define DW3000_REG_13_STS_DIAG_12_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS_DIAG_12_CP_NACC_BIT_OFFSET         (0U)
#define DW3000_REG_13_STS_DIAG_12_CP_NACC_BIT_MASK           (0x7FU)

/* Register:  STS_DIAG_RES3 (0x24) */
#define DW3000_REG_13_STS_DIAG_RES3_OFFSET                   0x24
#define DW3000_REG_13_STS_DIAG_RES3_BYTE_LEN                 (20U)

/* Register:  STS1_DIAG_0 (0x38) */
#define DW3000_REG_13_STS1_DIAG_0_OFFSET                     0x38
#define DW3000_REG_13_STS1_DIAG_0_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_0_CP_PEAKA_BIT_OFFSET        (0U)
#define DW3000_REG_13_STS1_DIAG_0_CP_PEAKA_BIT_MASK          (0x1FFFFFU)
#define DW3000_REG_13_STS1_DIAG_0_CP_PEAKI_BIT_OFFSET        (21U)
#define DW3000_REG_13_STS1_DIAG_0_CP_PEAKI_BIT_MASK          (0x3FE00000U)

/* Register:  STS1_DIAG_1 (0x3C) */
#define DW3000_REG_13_STS1_DIAG_1_OFFSET                     0x3C
#define DW3000_REG_13_STS1_DIAG_1_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_1_CP1_CAREA_BIT_OFFSET       (0U)
#define DW3000_REG_13_STS1_DIAG_1_CP1_CAREA_BIT_MASK         (0xFFFFU)

/* Register:  STS1_DIAG_2 (0x40) */
#define DW3000_REG_13_STS1_DIAG_2_OFFSET                     0x40
#define DW3000_REG_13_STS1_DIAG_2_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_2_CP_FP1M_BIT_OFFSET         (0U)
#define DW3000_REG_13_STS1_DIAG_2_CP_FP1M_BIT_MASK           (0x3FFFFFU)

/* Register:  STS1_DIAG_3 (0x44) */
#define DW3000_REG_13_STS1_DIAG_3_OFFSET                     0x44
#define DW3000_REG_13_STS1_DIAG_3_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_3_CP_FP2M_BIT_OFFSET         (0U)
#define DW3000_REG_13_STS1_DIAG_3_CP_FP2M_BIT_MASK           (0x3FFFFFU)

/* Register:  STS1_DIAG_4 (0x48) */
#define DW3000_REG_13_STS1_DIAG_4_OFFSET                     0x48
#define DW3000_REG_13_STS1_DIAG_4_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_4_CP_FP3M_BIT_OFFSET         (0U)
#define DW3000_REG_13_STS1_DIAG_4_CP_FP3M_BIT_MASK           (0x3FFFFFU)

/* Register:  STS1_DIAG_RES1 (0x4C) */
#define DW3000_REG_13_STS1_DIAG_RES1_OFFSET                  0x4C
#define DW3000_REG_13_STS1_DIAG_RES1_BYTE_LEN                (12U)

/* Register:  STS1_DIAG_8 (0x58) */
#define DW3000_REG_13_STS1_DIAG_8_OFFSET                     0x58
#define DW3000_REG_13_STS1_DIAG_8_BYTE_LEN                   (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_8_CP_FP_BIT_OFFSET           (0U)
#define DW3000_REG_13_STS1_DIAG_8_CP_FP_BIT_MASK             (0x7FFFU)

/* Register:  STS1_DIAG_RES2 (0x5C) */
#define DW3000_REG_13_STS1_DIAG_RES2_OFFSET                  0x5C
#define DW3000_REG_13_STS1_DIAG_RES2_BYTE_LEN                (12U)

/* Register:  STS1_DIAG_12 (0x68) */
#define DW3000_REG_13_STS1_DIAG_12_OFFSET                    0x68
#define DW3000_REG_13_STS1_DIAG_12_BYTE_LEN                  (4U)
/* Fields */
#define DW3000_REG_13_STS1_DIAG_12_CP_NACC_BIT_OFFSET        (0U)
#define DW3000_REG_13_STS1_DIAG_12_CP_NACC_BIT_MASK          (0x7FU)

/******************************************************************************
* @brief Bit definitions for register page 0x0E : CIA_IF p3
* @note  Channel Impulse Response Analyser (CIA) Interface part 3
**/
#define DW3000_REG_14_ADDR                                   0x0E

/* Register:  CIA_CONF (0x00) */
#define DW3000_REG_14_CIA_CONF_OFFSET                        0x00
#define DW3000_REG_14_CIA_CONF_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_14_CIA_CONF_RXANTD_BIT_OFFSET             (0U)
#define DW3000_REG_14_CIA_CONF_RXANTD_BIT_MASK               (0xFFFFU)
#define DW3000_REG_14_CIA_CONF_MINDIAG_BIT_OFFSET            (20U)
#define DW3000_REG_14_CIA_CONF_MINDIAG_BIT_MASK              (0x100000U)

/* Register:  FP_CONF (0x04) */
#define DW3000_REG_14_FP_CONF_OFFSET                         0x04
#define DW3000_REG_14_FP_CONF_BYTE_LEN                       (4U)
/* Fields */
#define DW3000_REG_14_FP_CONF_FP_AGREED_TH_BIT_OFFSET        (0U)
#define DW3000_REG_14_FP_CONF_FP_AGREED_TH_BIT_MASK          (0x7FFU)
#define DW3000_REG_14_FP_CONF_CAL_TEMP_BIT_OFFSET            (11U)
#define DW3000_REG_14_FP_CONF_CAL_TEMP_BIT_MASK              (0x7F800U)
#define DW3000_REG_14_FP_CONF_TC_RXDLY_EN_BIT_OFFSET         (20U)
#define DW3000_REG_14_FP_CONF_TC_RXDLY_EN_BIT_MASK           (0x100000U)

/* Register:  IP_CONF (0x0C) */
#define DW3000_REG_14_IP_CONF_OFFSET                         0x0C
#define DW3000_REG_14_IP_CONF_BYTE_LEN                       (4U)
/* Fields */
#define DW3000_REG_14_IP_CONF_IP_NTM_BIT_OFFSET              (0U)
#define DW3000_REG_14_IP_CONF_IP_NTM_BIT_MASK                (0x1FU)
#define DW3000_REG_14_IP_CONF_IP_PMULT_BIT_OFFSET            (5U)
#define DW3000_REG_14_IP_CONF_IP_PMULT_BIT_MASK              (0x60U)
#define DW3000_REG_14_IP_CONF_IP_RTM_BIT_OFFSET              (16U)
#define DW3000_REG_14_IP_CONF_IP_RTM_BIT_MASK                (0x1F0000U)

/* Register:  STS_CONF_0 (0x12) */
#define DW3000_REG_14_STS_CONF_0_OFFSET                      0x12
#define DW3000_REG_14_STS_CONF_0_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_14_STS_CONF_0_STS_NTM_BIT_OFFSET          (0U)
#define DW3000_REG_14_STS_CONF_0_STS_NTM_BIT_MASK            (0x1FU)
#define DW3000_REG_14_STS_CONF_0_STS_PMULT_BIT_OFFSET        (5U)
#define DW3000_REG_14_STS_CONF_0_STS_PMULT_BIT_MASK          (0x60U)
#define DW3000_REG_14_STS_CONF_0_STS_MNTH_BIT_OFFSET         (16U)
#define DW3000_REG_14_STS_CONF_0_STS_MNTH_BIT_MASK           (0x7F0000U)

/* Register:  STS_CONF_1 (0x16) */
#define DW3000_REG_14_STS_CONF_1_OFFSET                      0x16
#define DW3000_REG_14_STS_CONF_1_BYTE_LEN                    (4U)
/* Fields */
#define DW3000_REG_14_STS_CONF_1_RES_B0_BIT_OFFSET           (0U)
#define DW3000_REG_14_STS_CONF_1_RES_B0_BIT_MASK             (0xFFU)
#define DW3000_REG_14_STS_CONF_1_FP_AGREED_EN_BIT_OFFSET     (28U)
#define DW3000_REG_14_STS_CONF_1_FP_AGREED_EN_BIT_MASK       (0x10000000U)
#define DW3000_REG_14_STS_CONF_1_STS_CQ_EN_BIT_OFFSET        (29U)
#define DW3000_REG_14_STS_CONF_1_STS_CQ_EN_BIT_MASK          (0x20000000U)
#define DW3000_REG_14_STS_CONF_1_STS_SS_EN_BIT_OFFSET        (30U)
#define DW3000_REG_14_STS_CONF_1_STS_SS_EN_BIT_MASK          (0x40000000U)
#define DW3000_REG_14_STS_CONF_1_STS_PGR_EN_BIT_OFFSET       (31U)
#define DW3000_REG_14_STS_CONF_1_STS_PGR_EN_BIT_MASK         (0x80000000U)

/* Register:  CIA_ADJUST (0x1A) */
#define DW3000_REG_14_CIA_ADJUST_OFFSET                      0x1A
#define DW3000_REG_14_CIA_ADJUST_BYTE_LEN                    (2U)
/* Fields */
#define DW3000_REG_14_CIA_ADJUST_PDOA_ADJ_BIT_OFFSET         (0U)
#define DW3000_REG_14_CIA_ADJUST_PDOA_ADJ_BIT_MASK           (0x3FFFU)

/******************************************************************************
* @brief Bit definitions for register page 0x0F : DIG_DIAG
* @note  Digital diagnostics interface
**/
#define DW3000_REG_15_ADDR                                   0x0F
//TODO: Add register definitions

/******************************************************************************
* @brief Bit definitions for register page 0x11 : PMSC_CTRL
* @note  Power management, timing and seq control
**/
#define DW3000_REG_17_ADDR                                   0x11

/* Register:  SOFT_RST (0x00) */
#define DW3000_REG_17_SOFT_RST_OFFSET                        0x00
#define DW3000_REG_17_SOFT_RST_BYTE_LEN                      (1U)
/* Fields */
#define DW3000_REG_17_SOFT_RST_ARM_RST_BIT_OFFSET           (0U)
#define DW3000_REG_17_SOFT_RST_ARM_RST_BIT_MASK             (0x1U)
#define DW3000_REG_17_SOFT_RST_PRGN_RST_BIT_OFFSET          (1U)
#define DW3000_REG_17_SOFT_RST_PRGN_RST_BIT_MASK            (0x2U)
#define DW3000_REG_17_SOFT_RST_CIA_RST_BIT_OFFSET           (2U)
#define DW3000_REG_17_SOFT_RST_CIA_RST_BIT_MASK             (0x4U)
#define DW3000_REG_17_SOFT_RST_BIST_RST_BIT_OFFSET          (3U)
#define DW3000_REG_17_SOFT_RST_BIST_RST_BIT_MASK            (0x8U)
#define DW3000_REG_17_SOFT_RST_RX_RST_BIT_OFFSET            (4U)
#define DW3000_REG_17_SOFT_RST_RX_RST_BIT_MASK              (0x10U)
#define DW3000_REG_17_SOFT_RST_TX_RST_BIT_OFFSET            (5U)
#define DW3000_REG_17_SOFT_RST_TX_RST_BIT_MASK              (0x20U)
#define DW3000_REG_17_SOFT_RST_HIF_RST_BIT_OFFSET           (6U)
#define DW3000_REG_17_SOFT_RST_HIF_RST_BIT_MASK             (0x40U)
#define DW3000_REG_17_SOFT_RST_PMSC_RST_BIT_OFFSET          (7U)
#define DW3000_REG_17_SOFT_RST_PMSC_RST_BIT_MASK            (0x80U)
#define DW3000_REG_17_SOFT_RST_GPIO_RST_BIT_OFFSET          (8U)
#define DW3000_REG_17_SOFT_RST_GPIO_RST_BIT_MASK            (0x100U)

/* Register:  CLK_CTRL (0x04) - PMSC clock control register*/
#define DW3000_REG_17_CLK_CTRL_OFFSET                        0x04
#define DW3000_REG_17_CLK_CTRL_BYTE_LEN                      (4U)
/* Fields */
#define DW3000_REG_17_CLK_CTRL_SYS_CLK_BIT_OFFSET            (0U)
#define DW3000_REG_17_CLK_CTRL_SYS_CLK_BIT_MASK              (0x3U)
#define DW3000_REG_17_CLK_CTRL_RX_CLK_BIT_OFFSET             (2U)
#define DW3000_REG_17_CLK_CTRL_RX_CLK_BIT_MASK               (0xCU)
#define DW3000_REG_17_CLK_CTRL_TX_CLK_BIT_OFFSET             (4U)
#define DW3000_REG_17_CLK_CTRL_TX_CLK_BIT_MASK               (0x30U)
#define DW3000_REG_17_CLK_CTRL_ACC_CLK_EN_BIT_OFFSET         (6U)
#define DW3000_REG_17_CLK_CTRL_ACC_CLK_EN_BIT_MASK           (0x40U)
#define DW3000_REG_17_CLK_CTRL_CIA_CLK_EN_BIT_OFFSET         (8U)
#define DW3000_REG_17_CLK_CTRL_CIA_CLK_EN_BIT_MASK           (0x100U)
#define DW3000_REG_17_CLK_CTRL_SAR_CLK_EN_BIT_OFFSET         (10U)
#define DW3000_REG_17_CLK_CTRL_SAR_CLK_EN_BIT_MASK           (0x400U)
#define DW3000_REG_17_CLK_CTRL_ACC_MCLK_EN_BIT_OFFSET        (15U)
#define DW3000_REG_17_CLK_CTRL_ACC_MCLK_EN_BIT_MASK          (0x8000U)
#define DW3000_REG_17_CLK_CTRL_GPIO_CLK_EN_BIT_OFFSET        (16U)
#define DW3000_REG_17_CLK_CTRL_GPIO_CLK_EN_BIT_MASK          (0x10000U)
#define DW3000_REG_17_CLK_CTRL_GPIO_DCLK_EN_BIT_OFFSET       (18U)
#define DW3000_REG_17_CLK_CTRL_GPIO_DCLK_EN_BIT_MASK         (0x40000U)
#define DW3000_REG_17_CLK_CTRL_GPIO_DRST_N_BIT_OFFSET        (19U)
#define DW3000_REG_17_CLK_CTRL_GPIO_DRST_N_BIT_MASK          (0x80000U)
#define DW3000_REG_17_CLK_CTRL_LP_CLK_EN_BIT_OFFSET          (23U)
#define DW3000_REG_17_CLK_CTRL_LP_CLK_EN_BIT_MASK            (0x800000U)

/* Register:  SEQ_CTRL (0x08) */
//TODO: Add register definitions

/* Register:  TXFSEQ (0x012) - PMSC fine grain TX sequencing control */
#define DW3000_REG_17_TXFSEQ_OFFSET                          0x12
#define DW3000_REG_17_TXFSEQ_BYTE_LEN                        (4U)
/* Fields */
#define DW3000_REG_17_TXFSEQ_TXFINESEQ_BIT_OFFSET            (0U)
#define DW3000_REG_17_TXFSEQ_TXFINESEQ_BIT_MASK              (0xFFFFU)

/* Register:  LED_CTRL (0x16) - LED control register */
//TODO: Add register definitions

/* Register:  RX_SNIFF (0x1A) - Receiver SNIFF mode configuration */
//TODO: Add register definitions

/* Register:  BIAS_CTRL (0x1F) - Analog blocks’ calibration values */
#define DW3000_REG_17_BIAS_CTRL_OFFSET                       0x1F
#define DW3000_REG_17_BIAS_CTRL_BYTE_LEN                     (2U)
/* Fields */
#define DW3000_REG_17_BIAS_CTRL_BIAS_CTRL_BIT_OFFSET         (0U)
#define DW3000_REG_17_BIAS_CTRL_BIAS_CTRL_BIT_MASK           (0x1FU)


/******************************************************************************
* @brief Bit definitions for register page 0x12 : RX_BUFFER_0
* @note  RX frame data buffer 0
**/
#define DW3000_REG_18_RX_BUFFER_0_ADDR                       0x12
#define DW3000_REG_18_RX_BUFFER_0_BYTE_LEN                   (1024U)

/******************************************************************************
* @brief Bit definitions for register page 0x13 : RX_BUFFER_1
* @note  RX frame data buffer 1
**/
#define DW3000_REG_19_RX_BUFFER_1_ADDR                       0x13
#define DW3000_REG_19_RX_BUFFER_1_BYTE_LEN                   (1024U)

/******************************************************************************
* @brief Bit definitions for register page 0x14 : TX_BUFFER
* @note  Transmit data buffer
**/
#define DW3000_REG_20_TX_BUFFER_ADDR                         0x14
#define DW3000_REG_20_TX_BUFFER_BYTE_LEN                     (1024U)






#endif // __DW3000_REGS_H__