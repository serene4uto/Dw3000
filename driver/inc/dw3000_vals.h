/*! ----------------------------------------------------------------------------
 * @file    dw3000_vals.h
 *
 * @brief   DW3000 register values
 * 
 * @attention //TODO: put attention here
 * 
 * @author  Nguyen Ha Trung
 *
 */

#ifndef __DW3000_VALS_H__
#define __DW3000_VALS_H__

#define DWT_DGC_CFG             0x32
#define DWT_DGC_CFG0            0x10000240
#define DWT_DGC_CFG1            0x1b6da489

#define DWT_AUTO_CLKS          (0x200 | 0x200000 | 0x100000)  //this is the default value of CLK_CTRL register

#define PMSC_TXFINESEQ_ENABLE   0x4d28874
#define PMSC_TXFINESEQ_DISABLE  0x0d20874


#endif /* __DW3000_VALS_H__ */