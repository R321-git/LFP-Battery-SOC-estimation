//###########################################################################
//
// FILE:    hw_memcfg.h
//
// TITLE:   Definitions for the MEMCFG registers.
//
//###########################################################################
// 
// C2000Ware v5.05.00.00
//
// Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

#ifndef HW_MEMCFG_H
#define HW_MEMCFG_H

//*************************************************************************************************
//
// The following are defines for the MEMCFG register offsets
//
//*************************************************************************************************
#define MEMCFG_O_DXLOCK         0x0U    // Dedicated RAM Config Lock Register
#define MEMCFG_O_DXCOMMIT       0x2U    // Dedicated RAM Config Lock Commit Register
#define MEMCFG_O_DXACCPROT0     0x8U    // Dedicated RAM Config Register
#define MEMCFG_O_DXTEST         0x10U   // Dedicated RAM TEST Register
#define MEMCFG_O_DXINIT         0x12U   // Dedicated RAM Init Register
#define MEMCFG_O_DXINITDONE     0x14U   // Dedicated RAM InitDone Status Register
#define MEMCFG_O_LSXLOCK        0x20U   // Local Shared RAM Config Lock Register
#define MEMCFG_O_LSXCOMMIT      0x22U   // Local Shared RAM Config Lock Commit Register
#define MEMCFG_O_LSXMSEL        0x24U   // Local Shared RAM Master Sel Register
#define MEMCFG_O_LSXCLAPGM      0x26U   // Local Shared RAM Prog/Exe control Register
#define MEMCFG_O_LSXACCPROT0    0x28U   // Local Shared RAM Config Register 0
#define MEMCFG_O_LSXACCPROT1    0x2AU   // Local Shared RAM Config Register 1
#define MEMCFG_O_LSXTEST        0x30U   // Local Shared RAM TEST Register
#define MEMCFG_O_LSXINIT        0x32U   // Local Shared RAM Init Register
#define MEMCFG_O_LSXINITDONE    0x34U   // Local Shared RAM InitDone Status Register
#define MEMCFG_O_GSXLOCK        0x40U   // Global Shared RAM Config Lock Register
#define MEMCFG_O_GSXCOMMIT      0x42U   // Global Shared RAM Config Lock Commit Register
#define MEMCFG_O_GSXMSEL        0x44U   // Global Shared RAM Master Sel Register
#define MEMCFG_O_GSXACCPROT0    0x48U   // Global Shared RAM Config Register 0
#define MEMCFG_O_GSXACCPROT1    0x4AU   // Global Shared RAM Config Register 1
#define MEMCFG_O_GSXACCPROT2    0x4CU   // Global Shared RAM Config Register 2
#define MEMCFG_O_GSXACCPROT3    0x4EU   // Global Shared RAM Config Register 3
#define MEMCFG_O_GSXTEST        0x50U   // Global Shared RAM TEST Register
#define MEMCFG_O_GSXINIT        0x52U   // Global Shared RAM Init Register
#define MEMCFG_O_GSXINITDONE    0x54U   // Global Shared RAM InitDone Status Register
#define MEMCFG_O_MSGXTEST       0x70U   // Message RAM TEST Register
#define MEMCFG_O_MSGXINIT       0x72U   // Message RAM Init Register
#define MEMCFG_O_MSGXINITDONE   0x74U   // Message RAM InitDone Status Register

#define MEMCFG_O_EMIF1LOCK       0x0U   // EMIF1 Config Lock Register
#define MEMCFG_O_EMIF1COMMIT     0x2U   // EMIF1 Config Lock Commit Register
#define MEMCFG_O_EMIF1MSEL       0x4U   // EMIF1 Master Sel Register
#define MEMCFG_O_EMIF1ACCPROT0   0x8U   // EMIF1 Config Register 0

#define MEMCFG_O_EMIF2LOCK       0x0U   // EMIF2 Config Lock Register
#define MEMCFG_O_EMIF2COMMIT     0x2U   // EMIF2 Config Lock Commit Register
#define MEMCFG_O_EMIF2ACCPROT0   0x8U   // EMIF2 Config Register 0

#define MEMCFG_O_NMAVFLG          0x0U    // Non-Master Access Violation Flag Register
#define MEMCFG_O_NMAVSET          0x2U    // Non-Master Access Violation Flag Set Register
#define MEMCFG_O_NMAVCLR          0x4U    // Non-Master Access Violation Flag Clear Register
#define MEMCFG_O_NMAVINTEN        0x6U    // Non-Master Access Violation Interrupt Enable Register
#define MEMCFG_O_NMCPURDAVADDR    0x8U    // Non-Master CPU Read Access Violation Address
#define MEMCFG_O_NMCPUWRAVADDR    0xAU    // Non-Master CPU Write Access Violation Address
#define MEMCFG_O_NMCPUFAVADDR     0xCU    // Non-Master CPU Fetch Access Violation Address
#define MEMCFG_O_NMDMAWRAVADDR    0xEU    // Non-Master DMA Write Access Violation Address
#define MEMCFG_O_NMCLA1RDAVADDR   0x10U   // Non-Master CLA1 Read Access Violation Address
#define MEMCFG_O_NMCLA1WRAVADDR   0x12U   // Non-Master CLA1 Write Access Violation Address
#define MEMCFG_O_NMCLA1FAVADDR    0x14U   // Non-Master CLA1 Fetch Access Violation Address
#define MEMCFG_O_MAVFLG           0x20U   // Master Access Violation Flag Register
#define MEMCFG_O_MAVSET           0x22U   // Master Access Violation Flag Set Register
#define MEMCFG_O_MAVCLR           0x24U   // Master Access Violation Flag Clear Register
#define MEMCFG_O_MAVINTEN         0x26U   // Master Access Violation Interrupt Enable Register
#define MEMCFG_O_MCPUFAVADDR      0x28U   // Master CPU Fetch Access Violation Address
#define MEMCFG_O_MCPUWRAVADDR     0x2AU   // Master CPU Write Access Violation Address
#define MEMCFG_O_MDMAWRAVADDR     0x2CU   // Master  DMA Write Access Violation Address

#define MEMCFG_O_UCERRFLG       0x0U    // Uncorrectable Error Flag Register
#define MEMCFG_O_UCERRSET       0x2U    // Uncorrectable Error Flag Set Register
#define MEMCFG_O_UCERRCLR       0x4U    // Uncorrectable Error Flag Clear Register
#define MEMCFG_O_UCCPUREADDR    0x6U    // Uncorrectable CPU Read Error Address
#define MEMCFG_O_UCDMAREADDR    0x8U    // Uncorrectable DMA Read Error Address
#define MEMCFG_O_UCCLA1READDR   0xAU    // Uncorrectable CLA1 Read Error Address
#define MEMCFG_O_CERRFLG        0x20U   // Correctable Error Flag Register
#define MEMCFG_O_CERRSET        0x22U   // Correctable Error Flag Set Register
#define MEMCFG_O_CERRCLR        0x24U   // Correctable Error Flag Clear Register
#define MEMCFG_O_CCPUREADDR     0x26U   // Correctable CPU Read Error Address
#define MEMCFG_O_CERRCNT        0x2EU   // Correctable Error Count Register
#define MEMCFG_O_CERRTHRES      0x30U   // Correctable Error Threshold Value Register
#define MEMCFG_O_CEINTFLG       0x32U   // Correctable Error Interrupt Flag Status Register
#define MEMCFG_O_CEINTCLR       0x34U   // Correctable Error Interrupt Flag Clear Register
#define MEMCFG_O_CEINTSET       0x36U   // Correctable Error Interrupt Flag Set Register
#define MEMCFG_O_CEINTEN        0x38U   // Correctable Error Interrupt Enable Register

#define MEMCFG_O_ROMWAITSTATE   0x0U   // ROM Wait State Configuration Register

#define MEMCFG_O_ROMPREFETCH   0x0U   // ROM Prefetch Configuration Register


//*************************************************************************************************
//
// The following are defines for the bit fields in the DxLOCK register
//
//*************************************************************************************************
#define MEMCFG_DXLOCK_LOCK_D0   0x4U   // D0 RAM access protection and master select fields lock
                                       // bit
#define MEMCFG_DXLOCK_LOCK_D1   0x8U   // D1 RAM access protection and master select fields lock
                                       // bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the DxCOMMIT register
//
//*************************************************************************************************
#define MEMCFG_DXCOMMIT_COMMIT_D0   0x4U   // D0 RAM access protection and master select permanent
                                           // lock
#define MEMCFG_DXCOMMIT_COMMIT_D1   0x8U   // D1 RAM access protection and master select permanent
                                           // lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the DxACCPROT0 register
//
//*************************************************************************************************
#define MEMCFG_DXACCPROT0_FETCHPROT_D0   0x10000U     // Fetch Protection For D0 RAM
#define MEMCFG_DXACCPROT0_CPUWRPROT_D0   0x20000U     // CPU WR Protection For D0 RAM
#define MEMCFG_DXACCPROT0_FETCHPROT_D1   0x1000000U   // Fetch Protection For D1 RAM
#define MEMCFG_DXACCPROT0_CPUWRPROT_D1   0x2000000U   // CPU WR Protection For D1 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the DxTEST register
//
//*************************************************************************************************
#define MEMCFG_DXTEST_TEST_M0_S   0U
#define MEMCFG_DXTEST_TEST_M0_M   0x3U    // Selects the different modes for M0 RAM
#define MEMCFG_DXTEST_TEST_M1_S   2U
#define MEMCFG_DXTEST_TEST_M1_M   0xCU    // Selects the different modes for M1 RAM
#define MEMCFG_DXTEST_TEST_D0_S   4U
#define MEMCFG_DXTEST_TEST_D0_M   0x30U   // Selects the different modes for D0 RAM
#define MEMCFG_DXTEST_TEST_D1_S   6U
#define MEMCFG_DXTEST_TEST_D1_M   0xC0U   // Selects the different modes for D1 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the DxINIT register
//
//*************************************************************************************************
#define MEMCFG_DXINIT_INIT_M0   0x1U   // RAM Initialization control for M0 RAM.
#define MEMCFG_DXINIT_INIT_M1   0x2U   // RAM Initialization control for M1 RAM.
#define MEMCFG_DXINIT_INIT_D0   0x4U   // RAM Initialization control for D0 RAM.
#define MEMCFG_DXINIT_INIT_D1   0x8U   // RAM Initialization control for D1 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the DxINITDONE register
//
//*************************************************************************************************
#define MEMCFG_DXINITDONE_INITDONE_M0   0x1U   // RAM Initialization status for M0 RAM.
#define MEMCFG_DXINITDONE_INITDONE_M1   0x2U   // RAM Initialization status for M1 RAM.
#define MEMCFG_DXINITDONE_INITDONE_D0   0x4U   // RAM Initialization status for D0 RAM.
#define MEMCFG_DXINITDONE_INITDONE_D1   0x8U   // RAM Initialization status for D1 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxLOCK register
//
//*************************************************************************************************
#define MEMCFG_LSXLOCK_LOCK_LS0   0x1U    // LS0 RAM access protection and master select fields
                                          // lock bit
#define MEMCFG_LSXLOCK_LOCK_LS1   0x2U    // LS1 RAM access protection and master select fields
                                          // lock bit
#define MEMCFG_LSXLOCK_LOCK_LS2   0x4U    // LS2 RAM access protection and master select fields
                                          // lock bit
#define MEMCFG_LSXLOCK_LOCK_LS3   0x8U    // LS3 RAM access protection and master select fields
                                          // lock bit
#define MEMCFG_LSXLOCK_LOCK_LS4   0x10U   // LS4 RAM access protection and master select fields
                                          // lock bit
#define MEMCFG_LSXLOCK_LOCK_LS5   0x20U   // LS5 RAM access protection and master select fields
                                          // lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxCOMMIT register
//
//*************************************************************************************************
#define MEMCFG_LSXCOMMIT_COMMIT_LS0   0x1U    // LS0 RAM access protection and master select
                                              // permanent lock
#define MEMCFG_LSXCOMMIT_COMMIT_LS1   0x2U    // LS1 RAM access protection and master select
                                              // permanent lock
#define MEMCFG_LSXCOMMIT_COMMIT_LS2   0x4U    // LS2 RAM access protection and master select
                                              // permanent lock
#define MEMCFG_LSXCOMMIT_COMMIT_LS3   0x8U    // LS3 RAM access protection and master select
                                              // permanent lock
#define MEMCFG_LSXCOMMIT_COMMIT_LS4   0x10U   // LS4 RAM access protection and master select
                                              // permanent lock
#define MEMCFG_LSXCOMMIT_COMMIT_LS5   0x20U   // LS5 RAM access protection and master select
                                              // permanent lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxMSEL register
//
//*************************************************************************************************
#define MEMCFG_LSXMSEL_MSEL_LS0_S   0U
#define MEMCFG_LSXMSEL_MSEL_LS0_M   0x3U     // Master Select for LS0 RAM
#define MEMCFG_LSXMSEL_MSEL_LS1_S   2U
#define MEMCFG_LSXMSEL_MSEL_LS1_M   0xCU     // Master Select for LS1 RAM
#define MEMCFG_LSXMSEL_MSEL_LS2_S   4U
#define MEMCFG_LSXMSEL_MSEL_LS2_M   0x30U    // Master Select for LS2 RAM
#define MEMCFG_LSXMSEL_MSEL_LS3_S   6U
#define MEMCFG_LSXMSEL_MSEL_LS3_M   0xC0U    // Master Select for LS3 RAM
#define MEMCFG_LSXMSEL_MSEL_LS4_S   8U
#define MEMCFG_LSXMSEL_MSEL_LS4_M   0x300U   // Master Select for LS4 RAM
#define MEMCFG_LSXMSEL_MSEL_LS5_S   10U
#define MEMCFG_LSXMSEL_MSEL_LS5_M   0xC00U   // Master Select for LS5 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxCLAPGM register
//
//*************************************************************************************************
#define MEMCFG_LSXCLAPGM_CLAPGM_LS0   0x1U    // Selects LS0 RAM as program vs data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS1   0x2U    // Selects LS1 RAM as program vs data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS2   0x4U    // Selects LS2 RAM as program vs data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS3   0x8U    // Selects LS3 RAM as program vs data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS4   0x10U   // Selects LS4 RAM as program vs data memory for CLA
#define MEMCFG_LSXCLAPGM_CLAPGM_LS5   0x20U   // Selects LS5 RAM as program vs data memory for CLA

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxACCPROT0 register
//
//*************************************************************************************************
#define MEMCFG_LSXACCPROT0_FETCHPROT_LS0   0x1U         // Fetch Protection For LS0 RAM
#define MEMCFG_LSXACCPROT0_CPUWRPROT_LS0   0x2U         // CPU WR Protection For LS0 RAM
#define MEMCFG_LSXACCPROT0_FETCHPROT_LS1   0x100U       // Fetch Protection For LS1 RAM
#define MEMCFG_LSXACCPROT0_CPUWRPROT_LS1   0x200U       // CPU WR Protection For LS1 RAM
#define MEMCFG_LSXACCPROT0_FETCHPROT_LS2   0x10000U     // Fetch Protection For LS2 RAM
#define MEMCFG_LSXACCPROT0_CPUWRPROT_LS2   0x20000U     // CPU WR Protection For LS2 RAM
#define MEMCFG_LSXACCPROT0_FETCHPROT_LS3   0x1000000U   // Fetch Protection For LS3 RAM
#define MEMCFG_LSXACCPROT0_CPUWRPROT_LS3   0x2000000U   // CPU WR Protection For LS3 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxACCPROT1 register
//
//*************************************************************************************************
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS4   0x1U     // Fetch Protection For LS4 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS4   0x2U     // CPU WR Protection For LS4 RAM
#define MEMCFG_LSXACCPROT1_FETCHPROT_LS5   0x100U   // Fetch Protection For LS5 RAM
#define MEMCFG_LSXACCPROT1_CPUWRPROT_LS5   0x200U   // CPU WR Protection For LS5 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxTEST register
//
//*************************************************************************************************
#define MEMCFG_LSXTEST_TEST_LS0_S   0U
#define MEMCFG_LSXTEST_TEST_LS0_M   0x3U     // Selects the different modes for LS0 RAM
#define MEMCFG_LSXTEST_TEST_LS1_S   2U
#define MEMCFG_LSXTEST_TEST_LS1_M   0xCU     // Selects the different modes for LS1 RAM
#define MEMCFG_LSXTEST_TEST_LS2_S   4U
#define MEMCFG_LSXTEST_TEST_LS2_M   0x30U    // Selects the different modes for LS2 RAM
#define MEMCFG_LSXTEST_TEST_LS3_S   6U
#define MEMCFG_LSXTEST_TEST_LS3_M   0xC0U    // Selects the different modes for LS3 RAM
#define MEMCFG_LSXTEST_TEST_LS4_S   8U
#define MEMCFG_LSXTEST_TEST_LS4_M   0x300U   // Selects the different modes for LS4 RAM
#define MEMCFG_LSXTEST_TEST_LS5_S   10U
#define MEMCFG_LSXTEST_TEST_LS5_M   0xC00U   // Selects the different modes for LS5 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxINIT register
//
//*************************************************************************************************
#define MEMCFG_LSXINIT_INIT_LS0   0x1U    // RAM Initialization control for LS0 RAM.
#define MEMCFG_LSXINIT_INIT_LS1   0x2U    // RAM Initialization control for LS1 RAM.
#define MEMCFG_LSXINIT_INIT_LS2   0x4U    // RAM Initialization control for LS2 RAM.
#define MEMCFG_LSXINIT_INIT_LS3   0x8U    // RAM Initialization control for LS3 RAM.
#define MEMCFG_LSXINIT_INIT_LS4   0x10U   // RAM Initialization control for LS4 RAM.
#define MEMCFG_LSXINIT_INIT_LS5   0x20U   // RAM Initialization control for LS5 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the LSxINITDONE register
//
//*************************************************************************************************
#define MEMCFG_LSXINITDONE_INITDONE_LS0   0x1U    // RAM Initialization status for LS0 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS1   0x2U    // RAM Initialization status for LS1 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS2   0x4U    // RAM Initialization status for LS2 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS3   0x8U    // RAM Initialization status for LS3 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS4   0x10U   // RAM Initialization status for LS4 RAM.
#define MEMCFG_LSXINITDONE_INITDONE_LS5   0x20U   // RAM Initialization status for LS5 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxLOCK register
//
//*************************************************************************************************
#define MEMCFG_GSXLOCK_LOCK_GS0    0x1U      // GS0 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS1    0x2U      // GS1 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS2    0x4U      // GS2 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS3    0x8U      // GS3 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS4    0x10U     // GS4 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS5    0x20U     // GS5 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS6    0x40U     // GS6 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS7    0x80U     // GS7 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS8    0x100U    // GS8 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS9    0x200U    // GS9 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS10   0x400U    // GS10 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS11   0x800U    // GS11 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS12   0x1000U   // GS12 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS13   0x2000U   // GS13 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS14   0x4000U   // GS14 RAM access protection and master select fields
                                             // lock bit
#define MEMCFG_GSXLOCK_LOCK_GS15   0x8000U   // GS15 RAM access protection and master select fields
                                             // lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxCOMMIT register
//
//*************************************************************************************************
#define MEMCFG_GSXCOMMIT_COMMIT_GS0    0x1U      // GS0 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS1    0x2U      // GS1 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS2    0x4U      // GS2 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS3    0x8U      // GS3 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS4    0x10U     // GS4 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS5    0x20U     // GS5 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS6    0x40U     // GS6 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS7    0x80U     // GS7 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS8    0x100U    // GS8 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS9    0x200U    // GS9 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS10   0x400U    // GS10 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS11   0x800U    // GS11 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS12   0x1000U   // GS12 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS13   0x2000U   // GS13 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS14   0x4000U   // GS14 RAM access protection and master select
                                                 // permanent lock
#define MEMCFG_GSXCOMMIT_COMMIT_GS15   0x8000U   // GS15 RAM access protection and master select
                                                 // permanent lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxMSEL register
//
//*************************************************************************************************
#define MEMCFG_GSXMSEL_MSEL_GS0    0x1U      // Master Select for GS0 RAM
#define MEMCFG_GSXMSEL_MSEL_GS1    0x2U      // Master Select for GS1 RAM
#define MEMCFG_GSXMSEL_MSEL_GS2    0x4U      // Master Select for GS2 RAM
#define MEMCFG_GSXMSEL_MSEL_GS3    0x8U      // Master Select for GS3 RAM
#define MEMCFG_GSXMSEL_MSEL_GS4    0x10U     // Master Select for GS4 RAM
#define MEMCFG_GSXMSEL_MSEL_GS5    0x20U     // Master Select for GS5 RAM
#define MEMCFG_GSXMSEL_MSEL_GS6    0x40U     // Master Select for GS6 RAM
#define MEMCFG_GSXMSEL_MSEL_GS7    0x80U     // Master Select for GS7 RAM
#define MEMCFG_GSXMSEL_MSEL_GS8    0x100U    // Master Select for GS8 RAM
#define MEMCFG_GSXMSEL_MSEL_GS9    0x200U    // Master Select for GS9 RAM
#define MEMCFG_GSXMSEL_MSEL_GS10   0x400U    // Master Select for GS10 RAM
#define MEMCFG_GSXMSEL_MSEL_GS11   0x800U    // Master Select for GS11 RAM
#define MEMCFG_GSXMSEL_MSEL_GS12   0x1000U   // Master Select for GS12 RAM
#define MEMCFG_GSXMSEL_MSEL_GS13   0x2000U   // Master Select for GS13 RAM
#define MEMCFG_GSXMSEL_MSEL_GS14   0x4000U   // Master Select for GS14 RAM
#define MEMCFG_GSXMSEL_MSEL_GS15   0x8000U   // Master Select for GS15 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxACCPROT0 register
//
//*************************************************************************************************
#define MEMCFG_GSXACCPROT0_FETCHPROT_GS0   0x1U         // Fetch Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_CPUWRPROT_GS0   0x2U         // CPU WR Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_DMAWRPROT_GS0   0x4U         // DMA WR Protection For GS0 RAM
#define MEMCFG_GSXACCPROT0_FETCHPROT_GS1   0x100U       // Fetch Protection For GS1 RAM
#define MEMCFG_GSXACCPROT0_CPUWRPROT_GS1   0x200U       // CPU WR Protection For GS1 RAM
#define MEMCFG_GSXACCPROT0_DMAWRPROT_GS1   0x400U       // DMA WR Protection For GS1 RAM
#define MEMCFG_GSXACCPROT0_FETCHPROT_GS2   0x10000U     // Fetch Protection For GS2 RAM
#define MEMCFG_GSXACCPROT0_CPUWRPROT_GS2   0x20000U     // CPU WR Protection For GS2 RAM
#define MEMCFG_GSXACCPROT0_DMAWRPROT_GS2   0x40000U     // DMA WR Protection For GS2 RAM
#define MEMCFG_GSXACCPROT0_FETCHPROT_GS3   0x1000000U   // Fetch Protection For GS3 RAM
#define MEMCFG_GSXACCPROT0_CPUWRPROT_GS3   0x2000000U   // CPU WR Protection For GS3 RAM
#define MEMCFG_GSXACCPROT0_DMAWRPROT_GS3   0x4000000U   // DMA WR Protection For GS3 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxACCPROT1 register
//
//*************************************************************************************************
#define MEMCFG_GSXACCPROT1_FETCHPROT_GS4   0x1U         // Fetch Protection For GS4 RAM
#define MEMCFG_GSXACCPROT1_CPUWRPROT_GS4   0x2U         // CPU WR Protection For GS4 RAM
#define MEMCFG_GSXACCPROT1_DMAWRPROT_GS4   0x4U         // DMA WR Protection For GS4 RAM
#define MEMCFG_GSXACCPROT1_FETCHPROT_GS5   0x100U       // Fetch Protection For GS5 RAM
#define MEMCFG_GSXACCPROT1_CPUWRPROT_GS5   0x200U       // CPU WR Protection For GS5 RAM
#define MEMCFG_GSXACCPROT1_DMAWRPROT_GS5   0x400U       // DMA WR Protection For GS5RAM
#define MEMCFG_GSXACCPROT1_FETCHPROT_GS6   0x10000U     // Fetch Protection For GS6 RAM
#define MEMCFG_GSXACCPROT1_CPUWRPROT_GS6   0x20000U     // CPU WR Protection For GS6 RAM
#define MEMCFG_GSXACCPROT1_DMAWRPROT_GS6   0x40000U     // DMA WR Protection For GS6RAM
#define MEMCFG_GSXACCPROT1_FETCHPROT_GS7   0x1000000U   // Fetch Protection For GS7 RAM
#define MEMCFG_GSXACCPROT1_CPUWRPROT_GS7   0x2000000U   // CPU WR Protection For GS7 RAM
#define MEMCFG_GSXACCPROT1_DMAWRPROT_GS7   0x4000000U   // DMA WR Protection For GS7RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxACCPROT2 register
//
//*************************************************************************************************
#define MEMCFG_GSXACCPROT2_FETCHPROT_GS8    0x1U         // Fetch Protection For GS8 RAM
#define MEMCFG_GSXACCPROT2_CPUWRPROT_GS8    0x2U         // CPU WR Protection For GS8 RAM
#define MEMCFG_GSXACCPROT2_DMAWRPROT_GS8    0x4U         // DMA WR Protection For GS8 RAM
#define MEMCFG_GSXACCPROT2_FETCHPROT_GS9    0x100U       // Fetch Protection For GS9 RAM
#define MEMCFG_GSXACCPROT2_CPUWRPROT_GS9    0x200U       // CPU WR Protection For GS9 RAM
#define MEMCFG_GSXACCPROT2_DMAWRPROT_GS9    0x400U       // DMA WR Protection For GS9RAM
#define MEMCFG_GSXACCPROT2_FETCHPROT_GS10   0x10000U     // Fetch Protection For GS10 RAM
#define MEMCFG_GSXACCPROT2_CPUWRPROT_GS10   0x20000U     // CPU WR Protection For GS10 RAM
#define MEMCFG_GSXACCPROT2_DMAWRPROT_GS10   0x40000U     // DMA WR Protection For GS10RAM
#define MEMCFG_GSXACCPROT2_FETCHPROT_GS11   0x1000000U   // Fetch Protection For GS11 RAM
#define MEMCFG_GSXACCPROT2_CPUWRPROT_GS11   0x2000000U   // CPU WR Protection For GS11 RAM
#define MEMCFG_GSXACCPROT2_DMAWRPROT_GS11   0x4000000U   // DMA WR Protection For GS11RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxACCPROT3 register
//
//*************************************************************************************************
#define MEMCFG_GSXACCPROT3_FETCHPROT_GS12   0x1U         // Fetch Protection For GS12 RAM
#define MEMCFG_GSXACCPROT3_CPUWRPROT_GS12   0x2U         // CPU WR Protection For GS12 RAM
#define MEMCFG_GSXACCPROT3_DMAWRPROT_GS12   0x4U         // DMA WR Protection For GS12 RAM
#define MEMCFG_GSXACCPROT3_FETCHPROT_GS13   0x100U       // Fetch Protection For GS13 RAM
#define MEMCFG_GSXACCPROT3_CPUWRPROT_GS13   0x200U       // CPU WR Protection For GS13 RAM
#define MEMCFG_GSXACCPROT3_DMAWRPROT_GS13   0x400U       // DMA WR Protection For GS13RAM
#define MEMCFG_GSXACCPROT3_FETCHPROT_GS14   0x10000U     // Fetch Protection For GS14 RAM
#define MEMCFG_GSXACCPROT3_CPUWRPROT_GS14   0x20000U     // CPU WR Protection For GS14 RAM
#define MEMCFG_GSXACCPROT3_DMAWRPROT_GS14   0x40000U     // DMA WR Protection For GS14RAM
#define MEMCFG_GSXACCPROT3_FETCHPROT_GS15   0x1000000U   // Fetch Protection For GS15 RAM
#define MEMCFG_GSXACCPROT3_CPUWRPROT_GS15   0x2000000U   // CPU WR Protection For GS15 RAM
#define MEMCFG_GSXACCPROT3_DMAWRPROT_GS15   0x4000000U   // DMA WR Protection For GS15RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxTEST register
//
//*************************************************************************************************
#define MEMCFG_GSXTEST_TEST_GS0_S    0U
#define MEMCFG_GSXTEST_TEST_GS0_M    0x3U          // Selects the different modes for GS0 RAM
#define MEMCFG_GSXTEST_TEST_GS1_S    2U
#define MEMCFG_GSXTEST_TEST_GS1_M    0xCU          // Selects the different modes for GS1 RAM
#define MEMCFG_GSXTEST_TEST_GS2_S    4U
#define MEMCFG_GSXTEST_TEST_GS2_M    0x30U         // Selects the different modes for GS2 RAM
#define MEMCFG_GSXTEST_TEST_GS3_S    6U
#define MEMCFG_GSXTEST_TEST_GS3_M    0xC0U         // Selects the different modes for GS3 RAM
#define MEMCFG_GSXTEST_TEST_GS4_S    8U
#define MEMCFG_GSXTEST_TEST_GS4_M    0x300U        // Selects the different modes for GS4 RAM
#define MEMCFG_GSXTEST_TEST_GS5_S    10U
#define MEMCFG_GSXTEST_TEST_GS5_M    0xC00U        // Selects the different modes for GS5 RAM
#define MEMCFG_GSXTEST_TEST_GS6_S    12U
#define MEMCFG_GSXTEST_TEST_GS6_M    0x3000U       // Selects the different modes for GS6 RAM
#define MEMCFG_GSXTEST_TEST_GS7_S    14U
#define MEMCFG_GSXTEST_TEST_GS7_M    0xC000U       // Selects the different modes for GS7 RAM
#define MEMCFG_GSXTEST_TEST_GS8_S    16U
#define MEMCFG_GSXTEST_TEST_GS8_M    0x30000U      // Selects the different modes for GS8 RAM
#define MEMCFG_GSXTEST_TEST_GS9_S    18U
#define MEMCFG_GSXTEST_TEST_GS9_M    0xC0000U      // Selects the different modes for GS9 RAM
#define MEMCFG_GSXTEST_TEST_GS10_S   20U
#define MEMCFG_GSXTEST_TEST_GS10_M   0x300000U     // Selects the different modes for GS10 RAM
#define MEMCFG_GSXTEST_TEST_GS11_S   22U
#define MEMCFG_GSXTEST_TEST_GS11_M   0xC00000U     // Selects the different modes for GS11 RAM
#define MEMCFG_GSXTEST_TEST_GS12_S   24U
#define MEMCFG_GSXTEST_TEST_GS12_M   0x3000000U    // Selects the different modes for GS12 RAM
#define MEMCFG_GSXTEST_TEST_GS13_S   26U
#define MEMCFG_GSXTEST_TEST_GS13_M   0xC000000U    // Selects the different modes for GS13 RAM
#define MEMCFG_GSXTEST_TEST_GS14_S   28U
#define MEMCFG_GSXTEST_TEST_GS14_M   0x30000000U   // Selects the different modes for GS14 RAM
#define MEMCFG_GSXTEST_TEST_GS15_S   30U
#define MEMCFG_GSXTEST_TEST_GS15_M   0xC0000000U   // Selects the different modes for GS15 RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxINIT register
//
//*************************************************************************************************
#define MEMCFG_GSXINIT_INIT_GS0    0x1U      // RAM Initialization control for GS0 RAM.
#define MEMCFG_GSXINIT_INIT_GS1    0x2U      // RAM Initialization control for GS1 RAM.
#define MEMCFG_GSXINIT_INIT_GS2    0x4U      // RAM Initialization control for GS2 RAM.
#define MEMCFG_GSXINIT_INIT_GS3    0x8U      // RAM Initialization control for GS3 RAM.
#define MEMCFG_GSXINIT_INIT_GS4    0x10U     // RAM Initialization control for GS4 RAM.
#define MEMCFG_GSXINIT_INIT_GS5    0x20U     // RAM Initialization control for GS5 RAM.
#define MEMCFG_GSXINIT_INIT_GS6    0x40U     // RAM Initialization control for GS6 RAM.
#define MEMCFG_GSXINIT_INIT_GS7    0x80U     // RAM Initialization control for GS7 RAM.
#define MEMCFG_GSXINIT_INIT_GS8    0x100U    // RAM Initialization control for GS8 RAM.
#define MEMCFG_GSXINIT_INIT_GS9    0x200U    // RAM Initialization control for GS9 RAM.
#define MEMCFG_GSXINIT_INIT_GS10   0x400U    // RAM Initialization control for GS10 RAM.
#define MEMCFG_GSXINIT_INIT_GS11   0x800U    // RAM Initialization control for GS11 RAM.
#define MEMCFG_GSXINIT_INIT_GS12   0x1000U   // RAM Initialization control for GS12 RAM.
#define MEMCFG_GSXINIT_INIT_GS13   0x2000U   // RAM Initialization control for GS13 RAM.
#define MEMCFG_GSXINIT_INIT_GS14   0x4000U   // RAM Initialization control for GS14 RAM.
#define MEMCFG_GSXINIT_INIT_GS15   0x8000U   // RAM Initialization control for GS15 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the GSxINITDONE register
//
//*************************************************************************************************
#define MEMCFG_GSXINITDONE_INITDONE_GS0    0x1U      // RAM Initialization status for GS0 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS1    0x2U      // RAM Initialization status for GS1 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS2    0x4U      // RAM Initialization status for GS2 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS3    0x8U      // RAM Initialization status for GS3 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS4    0x10U     // RAM Initialization status for GS4 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS5    0x20U     // RAM Initialization status for GS5 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS6    0x40U     // RAM Initialization status for GS6 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS7    0x80U     // RAM Initialization status for GS7 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS8    0x100U    // RAM Initialization status for GS8 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS9    0x200U    // RAM Initialization status for GS9 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS10   0x400U    // RAM Initialization status for GS10 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS11   0x800U    // RAM Initialization status for GS11 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS12   0x1000U   // RAM Initialization status for GS12 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS13   0x2000U   // RAM Initialization status for GS13 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS14   0x4000U   // RAM Initialization status for GS14 RAM.
#define MEMCFG_GSXINITDONE_INITDONE_GS15   0x8000U   // RAM Initialization status for GS15 RAM.

//*************************************************************************************************
//
// The following are defines for the bit fields in the MSGxTEST register
//
//*************************************************************************************************
#define MEMCFG_MSGXTEST_TEST_CPUTOCPU_S    0U
#define MEMCFG_MSGXTEST_TEST_CPUTOCPU_M    0x3U    // CPU to CPU Mode Select
#define MEMCFG_MSGXTEST_TEST_CPUTOCLA1_S   2U
#define MEMCFG_MSGXTEST_TEST_CPUTOCLA1_M   0xCU    // CPU to CLA1 MSG RAM Mode Select
#define MEMCFG_MSGXTEST_TEST_CLA1TOCPU_S   4U
#define MEMCFG_MSGXTEST_TEST_CLA1TOCPU_M   0x30U   // CLA1 to CPU MSG RAM Mode Select

//*************************************************************************************************
//
// The following are defines for the bit fields in the MSGxINIT register
//
//*************************************************************************************************
#define MEMCFG_MSGXINIT_INIT_CPUTOCPU    0x1U   // Initialization control for CPU to CPU MSG RAM
#define MEMCFG_MSGXINIT_INIT_CPUTOCLA1   0x2U   // Initialization control for CPUTOCLA1 MSG RAM
#define MEMCFG_MSGXINIT_INIT_CLA1TOCPU   0x4U   // Initialization control for CLA1TOCPU MSG RAM

//*************************************************************************************************
//
// The following are defines for the bit fields in the MSGxINITDONE register
//
//*************************************************************************************************
#define MEMCFG_MSGXINITDONE_INITDONE_CPUTOCPU    0x1U   // Initialization status for CPU to CPU MSG
                                                        // RAM
#define MEMCFG_MSGXINITDONE_INITDONE_CPUTOCLA1   0x2U   // Initialization status for CPU to CLA1
                                                        // MSG RAM
#define MEMCFG_MSGXINITDONE_INITDONE_CLA1TOCPU   0x4U   // Initialization status for CLA1 to CPU
                                                        // MSG RAM


//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF1LOCK register
//
//*************************************************************************************************
#define MEMCFG_EMIF1LOCK_LOCK_EMIF1   0x1U   // EMIF1 access protection and master select fields
                                             // lock bit

//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF1COMMIT register
//
//*************************************************************************************************
#define MEMCFG_EMIF1COMMIT_COMMIT_EMIF1   0x1U   // EMIF1 access protection and master select
                                                 // permanent lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF1MSEL register
//
//*************************************************************************************************
#define MEMCFG_EMIF1MSEL_MSEL_EMIF1_S   0U
#define MEMCFG_EMIF1MSEL_MSEL_EMIF1_M   0x3U          // Master Select for EMIF1.
#define MEMCFG_EMIF1MSEL_KEY_S          4U
#define MEMCFG_EMIF1MSEL_KEY_M          0xFFFFFFF0U   // KEY to enable the write into MSEL_EMIF1
                                                      // bits

//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF1ACCPROT0 register
//
//*************************************************************************************************
#define MEMCFG_EMIF1ACCPROT0_FETCHPROT_EMIF1   0x1U   // Fetch Protection For EMIF1
#define MEMCFG_EMIF1ACCPROT0_CPUWRPROT_EMIF1   0x2U   // CPU WR Protection For EMIF1
#define MEMCFG_EMIF1ACCPROT0_DMAWRPROT_EMIF1   0x4U   // DMA WR Protection For EMIF1


//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF2LOCK register
//
//*************************************************************************************************
#define MEMCFG_EMIF2LOCK_LOCK_EMIF2   0x1U   // EMIF2 access protection and master select permanent
                                             // lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF2COMMIT register
//
//*************************************************************************************************
#define MEMCFG_EMIF2COMMIT_COMMIT_EMIF2   0x1U   // EMIF2 access protection and master select
                                                 // permanent lock

//*************************************************************************************************
//
// The following are defines for the bit fields in the EMIF2ACCPROT0 register
//
//*************************************************************************************************
#define MEMCFG_EMIF2ACCPROT0_FETCHPROT_EMIF2   0x1U   // Fetch Protection For EMIF2
#define MEMCFG_EMIF2ACCPROT0_CPUWRPROT_EMIF2   0x2U   // CPU WR Protection For EMIF2


//*************************************************************************************************
//
// The following are defines for the bit fields in the NMAVFLG register
//
//*************************************************************************************************
#define MEMCFG_NMAVFLG_CPUREAD     0x1U    // Non Master CPU Read Access Violation Flag
#define MEMCFG_NMAVFLG_CPUWRITE    0x2U    // Non Master CPU Write Access Violation Flag
#define MEMCFG_NMAVFLG_CPUFETCH    0x4U    // Non Master CPU Fetch Access Violation Flag
#define MEMCFG_NMAVFLG_DMAWRITE    0x8U    // Non Master DMA Write Access Violation Flag
#define MEMCFG_NMAVFLG_CLA1READ    0x10U   // Non Master CLA1 Read Access Violation Flag
#define MEMCFG_NMAVFLG_CLA1WRITE   0x20U   // Non Master CLA1 Write Access Violation Flag
#define MEMCFG_NMAVFLG_CLA1FETCH   0x40U   // Non Master CLA1 Fetch Access Violation Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMAVSET register
//
//*************************************************************************************************
#define MEMCFG_NMAVSET_CPUREAD     0x1U    // Non Master CPU Read Access Violation Flag Set
#define MEMCFG_NMAVSET_CPUWRITE    0x2U    // Non Master CPU Write Access Violation Flag Set
#define MEMCFG_NMAVSET_CPUFETCH    0x4U    // Non Master CPU Fetch Access Violation Flag Set
#define MEMCFG_NMAVSET_DMAWRITE    0x8U    // Non Master DMA Write Access Violation Flag Set
#define MEMCFG_NMAVSET_CLA1READ    0x10U   // Non Master CLA1 Read Access Violation Flag Set
#define MEMCFG_NMAVSET_CLA1WRITE   0x20U   // Non Master CLA1 Write Access Violation Flag Set
#define MEMCFG_NMAVSET_CLA1FETCH   0x40U   // Non Master CLA1 Fetch Access Violation Flag Set

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMAVCLR register
//
//*************************************************************************************************
#define MEMCFG_NMAVCLR_CPUREAD     0x1U    // Non Master CPU Read Access Violation Flag Clear
#define MEMCFG_NMAVCLR_CPUWRITE    0x2U    // Non Master CPU Write Access Violation Flag Clear
#define MEMCFG_NMAVCLR_CPUFETCH    0x4U    // Non Master CPU Fetch Access Violation Flag Clear
#define MEMCFG_NMAVCLR_DMAWRITE    0x8U    // Non Master DMA Write Access Violation Flag Clear
#define MEMCFG_NMAVCLR_CLA1READ    0x10U   // Non Master CLA1 Read Access Violation Flag Clear
#define MEMCFG_NMAVCLR_CLA1WRITE   0x20U   // Non Master CLA1 Write Access Violation Flag Clear
#define MEMCFG_NMAVCLR_CLA1FETCH   0x40U   // Non Master CLA1 Fetch Access Violation Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the NMAVINTEN register
//
//*************************************************************************************************
#define MEMCFG_NMAVINTEN_CPUREAD     0x1U    // Non Master CPU Read Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_CPUWRITE    0x2U    // Non Master CPU Write Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_CPUFETCH    0x4U    // Non Master CPU Fetch Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_DMAWRITE    0x8U    // Non Master DMA Write Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_CLA1READ    0x10U   // Non Master CLA1 Read Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_CLA1WRITE   0x20U   // Non Master CLA1 Write Access Violation Interrupt
                                             // Enable
#define MEMCFG_NMAVINTEN_CLA1FETCH   0x40U   // Non Master CLA1 Fetch Access Violation Interrupt
                                             // Enable

//*************************************************************************************************
//
// The following are defines for the bit fields in the MAVFLG register
//
//*************************************************************************************************
#define MEMCFG_MAVFLG_CPUFETCH   0x1U   // Master CPU Fetch Access Violation Flag
#define MEMCFG_MAVFLG_CPUWRITE   0x2U   // Master CPU Write Access Violation Flag
#define MEMCFG_MAVFLG_DMAWRITE   0x4U   // Master DMA Write Access Violation Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the MAVSET register
//
//*************************************************************************************************
#define MEMCFG_MAVSET_CPUFETCH   0x1U   // Master CPU Fetch Access Violation Flag Set
#define MEMCFG_MAVSET_CPUWRITE   0x2U   // Master CPU Write Access Violation Flag Set
#define MEMCFG_MAVSET_DMAWRITE   0x4U   // Master DMA Write Access Violation Flag Set

//*************************************************************************************************
//
// The following are defines for the bit fields in the MAVCLR register
//
//*************************************************************************************************
#define MEMCFG_MAVCLR_CPUFETCH   0x1U   // Master CPU Fetch Access Violation Flag Clear
#define MEMCFG_MAVCLR_CPUWRITE   0x2U   // Master CPU Write Access Violation Flag Clear
#define MEMCFG_MAVCLR_DMAWRITE   0x4U   // Master DMA Write Access Violation Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the MAVINTEN register
//
//*************************************************************************************************
#define MEMCFG_MAVINTEN_CPUFETCH   0x1U   // Master CPU Fetch Access Violation Interrupt Enable
#define MEMCFG_MAVINTEN_CPUWRITE   0x2U   // Master CPU Write Access Violation Interrupt Enable
#define MEMCFG_MAVINTEN_DMAWRITE   0x4U   // Master DMA Write Access Violation Interrupt Enable


//*************************************************************************************************
//
// The following are defines for the bit fields in the UCERRFLG register
//
//*************************************************************************************************
#define MEMCFG_UCERRFLG_CPURDERR    0x1U   // CPU Uncorrectable Read Error Flag
#define MEMCFG_UCERRFLG_DMARDERR    0x2U   // DMA Uncorrectable Read Error Flag
#define MEMCFG_UCERRFLG_CLA1RDERR   0x4U   // CLA1 Uncorrectable Read Error Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the UCERRSET register
//
//*************************************************************************************************
#define MEMCFG_UCERRSET_CPURDERR    0x1U   // CPU Uncorrectable Read Error Flag Set
#define MEMCFG_UCERRSET_DMARDERR    0x2U   // DMA Uncorrectable Read Error Flag Set
#define MEMCFG_UCERRSET_CLA1RDERR   0x4U   // CLA1 Uncorrectable Read Error Flag Set

//*************************************************************************************************
//
// The following are defines for the bit fields in the UCERRCLR register
//
//*************************************************************************************************
#define MEMCFG_UCERRCLR_CPURDERR    0x1U   // CPU Uncorrectable Read Error Flag Clear
#define MEMCFG_UCERRCLR_DMARDERR    0x2U   // DMA Uncorrectable Read Error Flag Clear
#define MEMCFG_UCERRCLR_CLA1RDERR   0x4U   // CLA1 Uncorrectable Read Error Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the CERRFLG register
//
//*************************************************************************************************
#define MEMCFG_CERRFLG_CPURDERR    0x1U   // CPU Correctable Read Error Flag
#define MEMCFG_CERRFLG_DMARDERR    0x2U   // DMA Correctable Read Error Flag
#define MEMCFG_CERRFLG_CLA1RDERR   0x4U   // CLA1 Correctable Read Error Flag

//*************************************************************************************************
//
// The following are defines for the bit fields in the CERRSET register
//
//*************************************************************************************************
#define MEMCFG_CERRSET_CPURDERR    0x1U   // CPU Correctable Read Error Flag Set
#define MEMCFG_CERRSET_DMARDERR    0x2U   // DMA Correctable Read Error Flag Set
#define MEMCFG_CERRSET_CLA1RDERR   0x4U   // CLA1 Correctable Read Error Flag Set

//*************************************************************************************************
//
// The following are defines for the bit fields in the CERRCLR register
//
//*************************************************************************************************
#define MEMCFG_CERRCLR_CPURDERR    0x1U   // CPU Correctable Read Error Flag Clear
#define MEMCFG_CERRCLR_DMARDERR    0x2U   // DMA Correctable Read Error Flag Clear
#define MEMCFG_CERRCLR_CLA1RDERR   0x4U   // CLA1 Correctable Read Error Flag Clear

//*************************************************************************************************
//
// The following are defines for the bit fields in the CEINTFLG register
//
//*************************************************************************************************
#define MEMCFG_CEINTFLG_CEINTFLAG   0x1U   // Total corrected error count exceeded threshold flag.

//*************************************************************************************************
//
// The following are defines for the bit fields in the CEINTCLR register
//
//*************************************************************************************************
#define MEMCFG_CEINTCLR_CEINTCLR   0x1U   // CPU Corrected Error Threshold Exceeded Error Clear.

//*************************************************************************************************
//
// The following are defines for the bit fields in the CEINTSET register
//
//*************************************************************************************************
#define MEMCFG_CEINTSET_CEINTSET   0x1U   // Total corrected error count exceeded flag set.

//*************************************************************************************************
//
// The following are defines for the bit fields in the CEINTEN register
//
//*************************************************************************************************
#define MEMCFG_CEINTEN_CEINTEN   0x1U   // CPU/DMA Correctable Error Interrupt Enable.


//*************************************************************************************************
//
// The following are defines for the bit fields in the ROMWAITSTATE register
//
//*************************************************************************************************
#define MEMCFG_ROMWAITSTATE_WSDISABLE   0x1U   // C28x ROM Wait State Enable/Disable Control


//*************************************************************************************************
//
// The following are defines for the bit fields in the ROMPREFETCH register
//
//*************************************************************************************************
#define MEMCFG_ROMPREFETCH_PFENABLE   0x1U   // ROM Prefetch Enable/Disable Control



#endif
