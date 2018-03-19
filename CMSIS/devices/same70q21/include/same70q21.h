/**
 * \file
 *
 * \brief Header file for ATSAME70Q21
 *
 * Copyright (c) 2017 Atmel Corporation, a wholly owned subsidiary of Microchip Technology Inc.
 *
 * \license_start
 *
 * \page License
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * \license_stop
 *
 */

/* file generated from device description version 2017-08-25T14:00:00Z */
#ifndef _SAME70Q21_H_
#define _SAME70Q21_H_

/** \addtogroup SAME70Q21_definitions SAME70Q21 definitions
  This file defines all structures and symbols for SAME70Q21:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - PIO definitions
 *  @{
 */

#ifdef __cplusplus
 extern "C" {
#endif

/** \defgroup Atmel_glob_defs Atmel Global Defines

    <strong>IO Type Qualifiers</strong> are used
    \li to specify the access to peripheral variables.
    \li for automatic generation of peripheral register debug information.

    \remark
    CMSIS core has a syntax that differs from this using i.e. __I, __O, or __IO followed by 'uint<size>_t' respective types.
    Default the header files will follow the CMSIS core syntax.
 *  @{
 */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#include <stdint.h>

/* IO definitions (access restrictions to peripheral registers) */
#ifndef __cplusplus
typedef volatile const uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile const uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile const uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#else
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile const unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
#endif
typedef volatile       uint32_t WoReg;   /**< Write only 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t WoReg16; /**< Write only 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t WoReg8;  /**< Write only  8-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */

#define CAST(type, value) ((type *)(value)) /**< Pointer Type Conversion Macro for C/C++ */
#define REG_ACCESS(type, address) (*(type*)(address)) /**< C code: Register value */
#else /* Assembler */
#define CAST(type, value) (value) /**< Pointer Type Conversion Macro for Assembler */
#define REG_ACCESS(type, address) (address) /**< Assembly code: Register address */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !defined(SKIP_INTEGER_LITERALS)

#if defined(_U_) || defined(_L_) || defined(_UL_)
  #error "Integer Literals macros already defined elsewhere"
#endif

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/* Macros that deal with adding suffixes to integer literal constants for C/C++ */
#define _U_(x) x ## U    /**< C code: Unsigned integer literal constant value */
#define _L_(x) x ## L    /**< C code: Long integer literal constant value */
#define _UL_(x) x ## UL  /**< C code: Unsigned Long integer literal constant value */

#else /* Assembler */

#define _U_(x) x    /**< Assembler: Unsigned integer literal constant value */
#define _L_(x) x    /**< Assembler: Long integer literal constant value */
#define _UL_(x) x   /**< Assembler: Unsigned Long integer literal constant value */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#endif /* SKIP_INTEGER_LITERALS */
/** @}  end of Atmel Global Defines */

/** \addtogroup SAME70Q21_cmsis CMSIS Definitions
 *  @{
 */
/* ************************************************************************** */
/*   CMSIS DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
/** Interrupt Number Definition */
typedef enum IRQn
{
/******  CORTEX-M7 Processor Exceptions Numbers ******************************/
  Reset_IRQn                = -15, /**< 1   Reset Vector, invoked on Power up and warm reset  */
  NonMaskableInt_IRQn       = -14, /**< 2   Non maskable Interrupt, cannot be stopped or preempted  */
  HardFault_IRQn            = -13, /**< 3   Hard Fault, all classes of Fault     */
  MemoryManagement_IRQn     = -12, /**< 4   Memory Management, MPU mismatch, including Access Violation and No Match  */
  BusFault_IRQn             = -11, /**< 5   Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault  */
  UsageFault_IRQn           = -10, /**< 6   Usage Fault, i.e. Undef Instruction, Illegal State Transition  */
  SVCall_IRQn               = -5 , /**< 11  System Service Call via SVC instruction  */
  DebugMonitor_IRQn         = -4 , /**< 12  Debug Monitor                        */
  PendSV_IRQn               = -2 , /**< 14  Pendable request for system service  */
  SysTick_IRQn              = -1 , /**< 15  System Tick Timer                    */
/******  SAME70Q21 specific Interrupt Numbers ***********************************/
  SUPC_IRQn                 = 0  , /**< 0   SAME70Q21 Supply Controller (SUPC)  */
  RSTC_IRQn                 = 1  , /**< 1   SAME70Q21 Reset Controller (RSTC)   */
  RTC_IRQn                  = 2  , /**< 2   SAME70Q21 Real-time Clock (RTC)     */
  RTT_IRQn                  = 3  , /**< 3   SAME70Q21 Real-time Timer (RTT)     */
  WDT_IRQn                  = 4  , /**< 4   SAME70Q21 Watchdog Timer (WDT)      */
  PMC_IRQn                  = 5  , /**< 5   SAME70Q21 Power Management Controller (PMC) */
  EFC_IRQn                  = 6  , /**< 6   SAME70Q21 Embedded Flash Controller (EFC) */
  UART0_IRQn                = 7  , /**< 7   SAME70Q21 Universal Asynchronous Receiver Transmitter (UART0) */
  UART1_IRQn                = 8  , /**< 8   SAME70Q21 Universal Asynchronous Receiver Transmitter (UART1) */
  PIOA_IRQn                 = 10 , /**< 10  SAME70Q21 Parallel Input/Output Controller (PIOA) */
  PIOB_IRQn                 = 11 , /**< 11  SAME70Q21 Parallel Input/Output Controller (PIOB) */
  PIOC_IRQn                 = 12 , /**< 12  SAME70Q21 Parallel Input/Output Controller (PIOC) */
  USART0_IRQn               = 13 , /**< 13  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART0) */
  USART1_IRQn               = 14 , /**< 14  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART1) */
  USART2_IRQn               = 15 , /**< 15  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART2) */
  PIOD_IRQn                 = 16 , /**< 16  SAME70Q21 Parallel Input/Output Controller (PIOD) */
  PIOE_IRQn                 = 17 , /**< 17  SAME70Q21 Parallel Input/Output Controller (PIOE) */
  HSMCI_IRQn                = 18 , /**< 18  SAME70Q21 High Speed MultiMedia Card Interface (HSMCI) */
  TWIHS0_IRQn               = 19 , /**< 19  SAME70Q21 Two-wire Interface High Speed (TWIHS0) */
  TWIHS1_IRQn               = 20 , /**< 20  SAME70Q21 Two-wire Interface High Speed (TWIHS1) */
  SPI0_IRQn                 = 21 , /**< 21  SAME70Q21 Serial Peripheral Interface (SPI0) */
  SSC_IRQn                  = 22 , /**< 22  SAME70Q21 Synchronous Serial Controller (SSC) */
  TC0_IRQn                  = 23 , /**< 23  SAME70Q21 Timer Counter (TC0)       */
  TC1_IRQn                  = 24 , /**< 24  SAME70Q21 Timer Counter (TC0)       */
  TC2_IRQn                  = 25 , /**< 25  SAME70Q21 Timer Counter (TC0)       */
  TC3_IRQn                  = 26 , /**< 26  SAME70Q21 Timer Counter (TC1)       */
  TC4_IRQn                  = 27 , /**< 27  SAME70Q21 Timer Counter (TC1)       */
  TC5_IRQn                  = 28 , /**< 28  SAME70Q21 Timer Counter (TC1)       */
  AFEC0_IRQn                = 29 , /**< 29  SAME70Q21 Analog Front-End Controller (AFEC0) */
  DACC_IRQn                 = 30 , /**< 30  SAME70Q21 Digital-to-Analog Converter Controller (DACC) */
  PWM0_IRQn                 = 31 , /**< 31  SAME70Q21 Pulse Width Modulation Controller (PWM0) */
  ICM_IRQn                  = 32 , /**< 32  SAME70Q21 Integrity Check Monitor (ICM) */
  ACC_IRQn                  = 33 , /**< 33  SAME70Q21 Analog Comparator Controller (ACC) */
  USBHS_IRQn                = 34 , /**< 34  SAME70Q21 USB High-Speed Interface (USBHS) */
  MCAN0_IRQn                = 35 , /**< 35  SAME70Q21 Controller Area Network (MCAN0) */
  MCAN1_IRQn                = 37 , /**< 37  SAME70Q21 Controller Area Network (MCAN1) */
  GMAC_IRQn                 = 39 , /**< 39  SAME70Q21 Gigabit Ethernet MAC (GMAC) */
  AFEC1_IRQn                = 40 , /**< 40  SAME70Q21 Analog Front-End Controller (AFEC1) */
  TWIHS2_IRQn               = 41 , /**< 41  SAME70Q21 Two-wire Interface High Speed (TWIHS2) */
  SPI1_IRQn                 = 42 , /**< 42  SAME70Q21 Serial Peripheral Interface (SPI1) */
  QSPI_IRQn                 = 43 , /**< 43  SAME70Q21 Quad Serial Peripheral Interface (QSPI) */
  UART2_IRQn                = 44 , /**< 44  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART2) */
  UART3_IRQn                = 45 , /**< 45  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART3) */
  UART4_IRQn                = 46 , /**< 46  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART4) */
  TC6_IRQn                  = 47 , /**< 47  SAME70Q21 Timer Counter (TC2)       */
  TC7_IRQn                  = 48 , /**< 48  SAME70Q21 Timer Counter (TC2)       */
  TC8_IRQn                  = 49 , /**< 49  SAME70Q21 Timer Counter (TC2)       */
  TC9_IRQn                  = 50 , /**< 50  SAME70Q21 Timer Counter (TC3)       */
  TC10_IRQn                 = 51 , /**< 51  SAME70Q21 Timer Counter (TC3)       */
  TC11_IRQn                 = 52 , /**< 52  SAME70Q21 Timer Counter (TC3)       */
  AES_IRQn                  = 56 , /**< 56  SAME70Q21 Advanced Encryption Standard (AES) */
  TRNG_IRQn                 = 57 , /**< 57  SAME70Q21 True Random Number Generator (TRNG) */
  XDMAC_IRQn                = 58 , /**< 58  SAME70Q21 Extensible DMA Controller (XDMAC) */
  ISI_IRQn                  = 59 , /**< 59  SAME70Q21 Image Sensor Interface (ISI) */
  PWM1_IRQn                 = 60 , /**< 60  SAME70Q21 Pulse Width Modulation Controller (PWM1) */
  RSWDT_IRQn                = 63 , /**< 63  SAME70Q21 Reinforced Safety Watchdog Timer (RSWDT) */
  SDRAMC_IRQn               = 62 , /**< 62  SAME70Q21 SDRAM Controller (SDRAMC) */

  PERIPH_COUNT_IRQn        = 64  /**< Number of peripheral IDs */
} IRQn_Type;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
typedef struct _DeviceVectors
{
  /* Stack pointer */
  void* pvStack;
  /* Cortex-M handlers */
  void* pfnReset_Handler;                        /* -15 Reset Vector, invoked on Power up and warm reset  */
  void* pfnNonMaskableInt_Handler;               /* -14 Non maskable Interrupt, cannot be stopped or preempted  */
  void* pfnHardFault_Handler;                    /* -13 Hard Fault, all classes of Fault     */
  void* pfnMemoryManagement_Handler;             /* -12 Memory Management, MPU mismatch, including Access Violation and No Match  */
  void* pfnBusFault_Handler;                     /* -11 Bus Fault, Pre-Fetch-, Memory Access Fault, other address/memory related Fault  */
  void* pfnUsageFault_Handler;                   /* -10 Usage Fault, i.e. Undef Instruction, Illegal State Transition  */
  void* pvReservedC9;
  void* pvReservedC8;
  void* pvReservedC7;
  void* pvReservedC6;
  void* pfnSVCall_Handler;                       /*  -5 System Service Call via SVC instruction  */
  void* pfnDebugMonitor_Handler;                 /*  -4 Debug Monitor                        */
  void* pvReservedC3;
  void* pfnPendSV_Handler;                       /*  -2 Pendable request for system service  */
  void* pfnSysTick_Handler;                      /*  -1 System Tick Timer                    */

  /* Peripheral handlers */
  void* pfnSUPC_Handler;                         /* 0   SAME70Q21 Supply Controller (SUPC) */
  void* pfnRSTC_Handler;                         /* 1   SAME70Q21 Reset Controller (RSTC) */
  void* pfnRTC_Handler;                          /* 2   SAME70Q21 Real-time Clock (RTC) */
  void* pfnRTT_Handler;                          /* 3   SAME70Q21 Real-time Timer (RTT) */
  void* pfnWDT_Handler;                          /* 4   SAME70Q21 Watchdog Timer (WDT) */
  void* pfnPMC_Handler;                          /* 5   SAME70Q21 Power Management Controller (PMC) */
  void* pfnEFC_Handler;                          /* 6   SAME70Q21 Embedded Flash Controller (EFC) */
  void* pfnUART0_Handler;                        /* 7   SAME70Q21 Universal Asynchronous Receiver Transmitter (UART0) */
  void* pfnUART1_Handler;                        /* 8   SAME70Q21 Universal Asynchronous Receiver Transmitter (UART1) */
  void* pvReserved9;
  void* pfnPIOA_Handler;                         /* 10  SAME70Q21 Parallel Input/Output Controller (PIOA) */
  void* pfnPIOB_Handler;                         /* 11  SAME70Q21 Parallel Input/Output Controller (PIOB) */
  void* pfnPIOC_Handler;                         /* 12  SAME70Q21 Parallel Input/Output Controller (PIOC) */
  void* pfnUSART0_Handler;                       /* 13  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART0) */
  void* pfnUSART1_Handler;                       /* 14  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART1) */
  void* pfnUSART2_Handler;                       /* 15  SAME70Q21 Universal Synchronous Asynchronous Receiver Transmitter (USART2) */
  void* pfnPIOD_Handler;                         /* 16  SAME70Q21 Parallel Input/Output Controller (PIOD) */
  void* pfnPIOE_Handler;                         /* 17  SAME70Q21 Parallel Input/Output Controller (PIOE) */
  void* pfnHSMCI_Handler;                        /* 18  SAME70Q21 High Speed MultiMedia Card Interface (HSMCI) */
  void* pfnTWIHS0_Handler;                       /* 19  SAME70Q21 Two-wire Interface High Speed (TWIHS0) */
  void* pfnTWIHS1_Handler;                       /* 20  SAME70Q21 Two-wire Interface High Speed (TWIHS1) */
  void* pfnSPI0_Handler;                         /* 21  SAME70Q21 Serial Peripheral Interface (SPI0) */
  void* pfnSSC_Handler;                          /* 22  SAME70Q21 Synchronous Serial Controller (SSC) */
  void* pfnTC0_Handler;                          /* 23  SAME70Q21 Timer Counter (TC0)  */
  void* pfnTC1_Handler;                          /* 24  SAME70Q21 Timer Counter (TC0)  */
  void* pfnTC2_Handler;                          /* 25  SAME70Q21 Timer Counter (TC0)  */
  void* pfnTC3_Handler;                          /* 26  SAME70Q21 Timer Counter (TC1)  */
  void* pfnTC4_Handler;                          /* 27  SAME70Q21 Timer Counter (TC1)  */
  void* pfnTC5_Handler;                          /* 28  SAME70Q21 Timer Counter (TC1)  */
  void* pfnAFEC0_Handler;                        /* 29  SAME70Q21 Analog Front-End Controller (AFEC0) */
  void* pfnDACC_Handler;                         /* 30  SAME70Q21 Digital-to-Analog Converter Controller (DACC) */
  void* pfnPWM0_Handler;                         /* 31  SAME70Q21 Pulse Width Modulation Controller (PWM0) */
  void* pfnICM_Handler;                          /* 32  SAME70Q21 Integrity Check Monitor (ICM) */
  void* pfnACC_Handler;                          /* 33  SAME70Q21 Analog Comparator Controller (ACC) */
  void* pfnUSBHS_Handler;                        /* 34  SAME70Q21 USB High-Speed Interface (USBHS) */
  void* pfnMCAN0_Handler;                        /* 35  SAME70Q21 Controller Area Network (MCAN0) */
  void* pvReserved36;
  void* pfnMCAN1_Handler;                        /* 37  SAME70Q21 Controller Area Network (MCAN1) */
  void* pvReserved38;
  void* pfnGMAC_Handler;                         /* 39  SAME70Q21 Gigabit Ethernet MAC (GMAC) */
  void* pfnAFEC1_Handler;                        /* 40  SAME70Q21 Analog Front-End Controller (AFEC1) */
  void* pfnTWIHS2_Handler;                       /* 41  SAME70Q21 Two-wire Interface High Speed (TWIHS2) */
  void* pfnSPI1_Handler;                         /* 42  SAME70Q21 Serial Peripheral Interface (SPI1) */
  void* pfnQSPI_Handler;                         /* 43  SAME70Q21 Quad Serial Peripheral Interface (QSPI) */
  void* pfnUART2_Handler;                        /* 44  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART2) */
  void* pfnUART3_Handler;                        /* 45  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART3) */
  void* pfnUART4_Handler;                        /* 46  SAME70Q21 Universal Asynchronous Receiver Transmitter (UART4) */
  void* pfnTC6_Handler;                          /* 47  SAME70Q21 Timer Counter (TC2)  */
  void* pfnTC7_Handler;                          /* 48  SAME70Q21 Timer Counter (TC2)  */
  void* pfnTC8_Handler;                          /* 49  SAME70Q21 Timer Counter (TC2)  */
  void* pfnTC9_Handler;                          /* 50  SAME70Q21 Timer Counter (TC3)  */
  void* pfnTC10_Handler;                         /* 51  SAME70Q21 Timer Counter (TC3)  */
  void* pfnTC11_Handler;                         /* 52  SAME70Q21 Timer Counter (TC3)  */
  void* pvReserved53;
  void* pvReserved54;
  void* pvReserved55;
  void* pfnAES_Handler;                          /* 56  SAME70Q21 Advanced Encryption Standard (AES) */
  void* pfnTRNG_Handler;                         /* 57  SAME70Q21 True Random Number Generator (TRNG) */
  void* pfnXDMAC_Handler;                        /* 58  SAME70Q21 Extensible DMA Controller (XDMAC) */
  void* pfnISI_Handler;                          /* 59  SAME70Q21 Image Sensor Interface (ISI) */
  void* pfnPWM1_Handler;                         /* 60  SAME70Q21 Pulse Width Modulation Controller (PWM1) */
  void* pvReserved61;
  void* pfnSDRAMC_Handler;                       /* 62  SAME70Q21 SDRAM Controller (SDRAMC) */
  void* pfnRSWDT_Handler;                        /* 63  SAME70Q21 Reinforced Safety Watchdog Timer (RSWDT) */
} DeviceVectors;
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */

#if !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#if !defined DONT_USE_PREDEFINED_CORE_HANDLERS

/* CORTEX-M7 core handlers */
void Reset_Handler                 ( void );
void NonMaskableInt_Handler        ( void );
void HardFault_Handler             ( void );
void MemoryManagement_Handler      ( void );
void BusFault_Handler              ( void );
void UsageFault_Handler            ( void );
void SVCall_Handler                ( void );
void DebugMonitor_Handler          ( void );
void PendSV_Handler                ( void );
void SysTick_Handler               ( void );
#endif /* DONT_USE_PREDEFINED_CORE_HANDLERS */

#if !defined DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS

/* Peripherals handlers */
void ACC_Handler                   ( void );
void AES_Handler                   ( void );
void AFEC0_Handler                 ( void );
void AFEC1_Handler                 ( void );
void DACC_Handler                  ( void );
void EFC_Handler                   ( void );
void GMAC_Handler                  ( void );
void HSMCI_Handler                 ( void );
void ICM_Handler                   ( void );
void ISI_Handler                   ( void );
void MCAN0_Handler                 ( void );
void MCAN1_Handler                 ( void );
void PIOA_Handler                  ( void );
void PIOB_Handler                  ( void );
void PIOC_Handler                  ( void );
void PIOD_Handler                  ( void );
void PIOE_Handler                  ( void );
void PMC_Handler                   ( void );
void PWM0_Handler                  ( void );
void PWM1_Handler                  ( void );
void QSPI_Handler                  ( void );
void RSTC_Handler                  ( void );
void RSWDT_Handler                 ( void );
void RTC_Handler                   ( void );
void RTT_Handler                   ( void );
void SDRAMC_Handler                ( void );
void SPI0_Handler                  ( void );
void SPI1_Handler                  ( void );
void SSC_Handler                   ( void );
void SUPC_Handler                  ( void );
void TC0_Handler                   ( void );
void TC10_Handler                  ( void );
void TC11_Handler                  ( void );
void TC1_Handler                   ( void );
void TC2_Handler                   ( void );
void TC3_Handler                   ( void );
void TC4_Handler                   ( void );
void TC5_Handler                   ( void );
void TC6_Handler                   ( void );
void TC7_Handler                   ( void );
void TC8_Handler                   ( void );
void TC9_Handler                   ( void );
void TRNG_Handler                  ( void );
void TWIHS0_Handler                ( void );
void TWIHS1_Handler                ( void );
void TWIHS2_Handler                ( void );
void UART0_Handler                 ( void );
void UART1_Handler                 ( void );
void UART2_Handler                 ( void );
void UART3_Handler                 ( void );
void UART4_Handler                 ( void );
void USART0_Handler                ( void );
void USART1_Handler                ( void );
void USART2_Handler                ( void );
void USBHS_Handler                 ( void );
void WDT_Handler                   ( void );
void XDMAC_Handler                 ( void );
#endif /* DONT_USE_PREDEFINED_PERIPHERALS_HANDLERS */
#endif /* !(defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */


/*
 * \brief Configuration of the CORTEX-M7 Processor and Core Peripherals
 */

#define __CM7_REV                 0x0101 /**< CM7 Core Revision                                                         */
#define __NVIC_PRIO_BITS               3 /**< Number of Bits used for Priority Levels                                   */
#define __Vendor_SysTickConfig         0 /**< Set to 1 if different SysTick Config is used                              */
#define __MPU_PRESENT                  1 /**< MPU present or not                                                        */
#define __VTOR_PRESENT                 1 /**< Vector Table Offset Register present or not                               */
#define __FPU_PRESENT                  1 /**< FPU present or not                                                        */
#define __FPU_DP                       1 /**< Double Precision FPU                                                      */
#define __ICACHE_PRESENT               1 /**< Instruction Cache present                                                 */
#define __DCACHE_PRESENT               1 /**< Data Cache present                                                        */
#define __ITCM_PRESENT                 1 /**< Instruction TCM present                                                   */
#define __DTCM_PRESENT                 1 /**< Data TCM present                                                          */
#define __DEBUG_LVL                    1
#define __TRACE_LVL                    1
#define LITTLE_ENDIAN                  1
#define __ARCH_ARM                     1
#define __ARCH_ARM_CORTEX_M            1
#define __DEVICE_IS_SAM                1

/*
 * \brief CMSIS includes
 */
#include <core_cm7.h>

/** @}  end of SAME70Q21_cmsis CMSIS Definitions */

/** \defgroup SAME70Q21_api Peripheral Software API
 *  @{
 */

/* ************************************************************************** */
/**  SOFTWARE PERIPHERAL API DEFINITION FOR SAME70Q21 */
/* ************************************************************************** */
#include "component/acc.h"
#include "component/aes.h"
#include "component/afec.h"
#include "component/chipid.h"
#include "component/dacc.h"
#include "component/efc.h"
#include "component/gmac.h"
#include "component/gpbr.h"
#include "component/hsmci.h"
#include "component/icm.h"
#include "component/isi.h"
#include "component/matrix.h"
#include "component/mcan.h"
#include "component/pio.h"
#include "component/pmc.h"
#include "component/pwm.h"
#include "component/qspi.h"
#include "component/rstc.h"
#include "component/rswdt.h"
#include "component/rtc.h"
#include "component/rtt.h"
#include "component/sdramc.h"
#include "component/smc.h"
#include "component/spi.h"
#include "component/ssc.h"
#include "component/supc.h"
#include "component/tc.h"
#include "component/trng.h"
#include "component/twihs.h"
#include "component/uart.h"
#include "component/usart.h"
#include "component/usbhs.h"
#include "component/utmi.h"
#include "component/wdt.h"
#include "component/xdmac.h"
/** @}  end of Peripheral Software API */

/** \defgroup SAME70Q21_reg Registers Access Definitions
 *  @{
 */

/* ************************************************************************** */
/*   REGISTER ACCESS DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#include "instance/acc.h"
#include "instance/aes.h"
#include "instance/afec0.h"
#include "instance/afec1.h"
#include "instance/chipid.h"
#include "instance/dacc.h"
#include "instance/efc.h"
#include "instance/gmac.h"
#include "instance/gpbr.h"
#include "instance/hsmci.h"
#include "instance/icm.h"
#include "instance/isi.h"
#include "instance/matrix.h"
#include "instance/mcan0.h"
#include "instance/mcan1.h"
#include "instance/pioa.h"
#include "instance/piob.h"
#include "instance/pioc.h"
#include "instance/piod.h"
#include "instance/pioe.h"
#include "instance/pmc.h"
#include "instance/pwm0.h"
#include "instance/pwm1.h"
#include "instance/qspi.h"
#include "instance/rstc.h"
#include "instance/rswdt.h"
#include "instance/rtc.h"
#include "instance/rtt.h"
#include "instance/sdramc.h"
#include "instance/smc.h"
#include "instance/spi0.h"
#include "instance/spi1.h"
#include "instance/ssc.h"
#include "instance/supc.h"
#include "instance/tc0.h"
#include "instance/tc1.h"
#include "instance/tc2.h"
#include "instance/tc3.h"
#include "instance/trng.h"
#include "instance/twihs0.h"
#include "instance/twihs1.h"
#include "instance/twihs2.h"
#include "instance/uart0.h"
#include "instance/uart1.h"
#include "instance/uart2.h"
#include "instance/uart3.h"
#include "instance/uart4.h"
#include "instance/usart0.h"
#include "instance/usart1.h"
#include "instance/usart2.h"
#include "instance/usbhs.h"
#include "instance/utmi.h"
#include "instance/wdt.h"
#include "instance/xdmac.h"
/** @}  end of Registers Access Definitions */

/** \addtogroup SAME70Q21_id Peripheral Ids Definitions
 *  @{
 */

/* ************************************************************************** */
/*  PERIPHERAL ID DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#define ID_SUPC         (  0) /**< \brief Supply Controller (SUPC) */
#define ID_RSTC         (  1) /**< \brief Reset Controller (RSTC) */
#define ID_RTC          (  2) /**< \brief Real-time Clock (RTC) */
#define ID_RTT          (  3) /**< \brief Real-time Timer (RTT) */
#define ID_WDT          (  4) /**< \brief Watchdog Timer (WDT) */
#define ID_PMC          (  5) /**< \brief Power Management Controller (PMC) */
#define ID_EFC          (  6) /**< \brief Embedded Flash Controller (EFC) */
#define ID_UART0        (  7) /**< \brief Universal Asynchronous Receiver Transmitter (UART0) */
#define ID_UART1        (  8) /**< \brief Universal Asynchronous Receiver Transmitter (UART1) */
#define ID_SMC          (  9) /**< \brief Static Memory Controller (SMC) */
#define ID_PIOA         ( 10) /**< \brief Parallel Input/Output Controller (PIOA) */
#define ID_PIOB         ( 11) /**< \brief Parallel Input/Output Controller (PIOB) */
#define ID_PIOC         ( 12) /**< \brief Parallel Input/Output Controller (PIOC) */
#define ID_USART0       ( 13) /**< \brief Universal Synchronous Asynchronous Receiver Transmitter (USART0) */
#define ID_USART1       ( 14) /**< \brief Universal Synchronous Asynchronous Receiver Transmitter (USART1) */
#define ID_USART2       ( 15) /**< \brief Universal Synchronous Asynchronous Receiver Transmitter (USART2) */
#define ID_PIOD         ( 16) /**< \brief Parallel Input/Output Controller (PIOD) */
#define ID_PIOE         ( 17) /**< \brief Parallel Input/Output Controller (PIOE) */
#define ID_HSMCI        ( 18) /**< \brief High Speed MultiMedia Card Interface (HSMCI) */
#define ID_TWIHS0       ( 19) /**< \brief Two-wire Interface High Speed (TWIHS0) */
#define ID_TWIHS1       ( 20) /**< \brief Two-wire Interface High Speed (TWIHS1) */
#define ID_SPI0         ( 21) /**< \brief Serial Peripheral Interface (SPI0) */
#define ID_SSC          ( 22) /**< \brief Synchronous Serial Controller (SSC) */
#define ID_TC0_CHANNEL0 ( 23) /**< \brief Timer Counter (TC0_CHANNEL0) */
#define ID_TC0_CHANNEL1 ( 24) /**< \brief Timer Counter (TC0_CHANNEL1) */
#define ID_TC0_CHANNEL2 ( 25) /**< \brief Timer Counter (TC0_CHANNEL2) */
#define ID_TC1_CHANNEL0 ( 26) /**< \brief Timer Counter (TC1_CHANNEL0) */
#define ID_TC1_CHANNEL1 ( 27) /**< \brief Timer Counter (TC1_CHANNEL1) */
#define ID_TC1_CHANNEL2 ( 28) /**< \brief Timer Counter (TC1_CHANNEL2) */
#define ID_AFEC0        ( 29) /**< \brief Analog Front-End Controller (AFEC0) */
#define ID_DACC         ( 30) /**< \brief Digital-to-Analog Converter Controller (DACC) */
#define ID_PWM0         ( 31) /**< \brief Pulse Width Modulation Controller (PWM0) */
#define ID_ICM          ( 32) /**< \brief Integrity Check Monitor (ICM) */
#define ID_ACC          ( 33) /**< \brief Analog Comparator Controller (ACC) */
#define ID_USBHS        ( 34) /**< \brief USB High-Speed Interface (USBHS) */
#define ID_MCAN0        ( 35) /**< \brief Controller Area Network (MCAN0) */
#define ID_MCAN1        ( 37) /**< \brief Controller Area Network (MCAN1) */
#define ID_GMAC         ( 39) /**< \brief Gigabit Ethernet MAC (GMAC) */
#define ID_AFEC1        ( 40) /**< \brief Analog Front-End Controller (AFEC1) */
#define ID_TWIHS2       ( 41) /**< \brief Two-wire Interface High Speed (TWIHS2) */
#define ID_SPI1         ( 42) /**< \brief Serial Peripheral Interface (SPI1) */
#define ID_QSPI         ( 43) /**< \brief Quad Serial Peripheral Interface (QSPI) */
#define ID_UART2        ( 44) /**< \brief Universal Asynchronous Receiver Transmitter (UART2) */
#define ID_UART3        ( 45) /**< \brief Universal Asynchronous Receiver Transmitter (UART3) */
#define ID_UART4        ( 46) /**< \brief Universal Asynchronous Receiver Transmitter (UART4) */
#define ID_TC2_CHANNEL0 ( 47) /**< \brief Timer Counter (TC2_CHANNEL0) */
#define ID_TC2_CHANNEL1 ( 48) /**< \brief Timer Counter (TC2_CHANNEL1) */
#define ID_TC2_CHANNEL2 ( 49) /**< \brief Timer Counter (TC2_CHANNEL2) */
#define ID_TC3_CHANNEL0 ( 50) /**< \brief Timer Counter (TC3_CHANNEL0) */
#define ID_TC3_CHANNEL1 ( 51) /**< \brief Timer Counter (TC3_CHANNEL1) */
#define ID_TC3_CHANNEL2 ( 52) /**< \brief Timer Counter (TC3_CHANNEL2) */
#define ID_AES          ( 56) /**< \brief Advanced Encryption Standard (AES) */
#define ID_TRNG         ( 57) /**< \brief True Random Number Generator (TRNG) */
#define ID_XDMAC        ( 58) /**< \brief Extensible DMA Controller (XDMAC) */
#define ID_ISI          ( 59) /**< \brief Image Sensor Interface (ISI) */
#define ID_PWM1         ( 60) /**< \brief Pulse Width Modulation Controller (PWM1) */
#define ID_SDRAMC       ( 62) /**< \brief SDRAM Controller (SDRAMC) */
#define ID_RSWDT        ( 63) /**< \brief Reinforced Safety Watchdog Timer (RSWDT) */

#define ID_PERIPH_COUNT ( 64) /**< \brief Number of peripheral IDs */
/** @}  end of Peripheral Ids Definitions */

/** \addtogroup legacy_SAME70Q21_id Legacy Peripheral Ids Definitions
 *  @{
 */

/* ************************************************************************** */
/*  LEGACY PERIPHERAL ID DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#define ID_TC0                   TC0_INSTANCE_ID_CHANNEL0
#define ID_TC1                   TC0_INSTANCE_ID_CHANNEL1
#define ID_TC2                   TC0_INSTANCE_ID_CHANNEL2
#define ID_TC3                   TC1_INSTANCE_ID_CHANNEL0
#define ID_TC4                   TC1_INSTANCE_ID_CHANNEL1
#define ID_TC5                   TC1_INSTANCE_ID_CHANNEL2
#define ID_TC6                   TC2_INSTANCE_ID_CHANNEL0
#define ID_TC7                   TC2_INSTANCE_ID_CHANNEL1
#define ID_TC8                   TC2_INSTANCE_ID_CHANNEL2
#define ID_TC9                   TC3_INSTANCE_ID_CHANNEL0
#define ID_TC10                  TC3_INSTANCE_ID_CHANNEL1
#define ID_TC11                  TC3_INSTANCE_ID_CHANNEL2
/** @}  end of Legacy Peripheral Ids Definitions */

/** \addtogroup SAME70Q21_base Peripheral Base Address Definitions
 *  @{
 */

/* ************************************************************************** */
/*   BASE ADDRESS DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#if (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__))
#define ACC                    (0x40044000)                   /**< \brief (ACC       ) Base Address */
#define AES                    (0x4006C000)                   /**< \brief (AES       ) Base Address */
#define AFEC0                  (0x4003C000)                   /**< \brief (AFEC0     ) Base Address */
#define AFEC1                  (0x40064000)                   /**< \brief (AFEC1     ) Base Address */
#define CHIPID                 (0x400E0940)                   /**< \brief (CHIPID    ) Base Address */
#define DACC                   (0x40040000)                   /**< \brief (DACC      ) Base Address */
#define EFC                    (0x400E0C00)                   /**< \brief (EFC       ) Base Address */
#define GMAC                   (0x40050000)                   /**< \brief (GMAC      ) Base Address */
#define GPBR                   (0x400E1890)                   /**< \brief (GPBR      ) Base Address */
#define HSMCI                  (0x40000000)                   /**< \brief (HSMCI     ) Base Address */
#define ICM                    (0x40048000)                   /**< \brief (ICM       ) Base Address */
#define ISI                    (0x4004C000)                   /**< \brief (ISI       ) Base Address */
#define MATRIX                 (0x40088000)                   /**< \brief (MATRIX    ) Base Address */
#define MCAN0                  (0x40030000)                   /**< \brief (MCAN0     ) Base Address */
#define MCAN1                  (0x40034000)                   /**< \brief (MCAN1     ) Base Address */
#define PIOA                   (0x400E0E00)                   /**< \brief (PIOA      ) Base Address */
#define PIOB                   (0x400E1000)                   /**< \brief (PIOB      ) Base Address */
#define PIOC                   (0x400E1200)                   /**< \brief (PIOC      ) Base Address */
#define PIOD                   (0x400E1400)                   /**< \brief (PIOD      ) Base Address */
#define PIOE                   (0x400E1600)                   /**< \brief (PIOE      ) Base Address */
#define PMC                    (0x400E0600)                   /**< \brief (PMC       ) Base Address */
#define PWM0                   (0x40020000)                   /**< \brief (PWM0      ) Base Address */
#define PWM1                   (0x4005C000)                   /**< \brief (PWM1      ) Base Address */
#define QSPI                   (0x4007C000)                   /**< \brief (QSPI      ) Base Address */
#define RSTC                   (0x400E1800)                   /**< \brief (RSTC      ) Base Address */
#define RSWDT                  (0x400E1900)                   /**< \brief (RSWDT     ) Base Address */
#define RTC                    (0x400E1860)                   /**< \brief (RTC       ) Base Address */
#define RTT                    (0x400E1830)                   /**< \brief (RTT       ) Base Address */
#define SDRAMC                 (0x40084000)                   /**< \brief (SDRAMC    ) Base Address */
#define SMC                    (0x40080000)                   /**< \brief (SMC       ) Base Address */
#define SPI0                   (0x40008000)                   /**< \brief (SPI0      ) Base Address */
#define SPI1                   (0x40058000)                   /**< \brief (SPI1      ) Base Address */
#define SSC                    (0x40004000)                   /**< \brief (SSC       ) Base Address */
#define SUPC                   (0x400E1810)                   /**< \brief (SUPC      ) Base Address */
#define TC0                    (0x4000C000)                   /**< \brief (TC0       ) Base Address */
#define TC1                    (0x40010000)                   /**< \brief (TC1       ) Base Address */
#define TC2                    (0x40014000)                   /**< \brief (TC2       ) Base Address */
#define TC3                    (0x40054000)                   /**< \brief (TC3       ) Base Address */
#define TRNG                   (0x40070000)                   /**< \brief (TRNG      ) Base Address */
#define TWIHS0                 (0x40018000)                   /**< \brief (TWIHS0    ) Base Address */
#define TWIHS1                 (0x4001C000)                   /**< \brief (TWIHS1    ) Base Address */
#define TWIHS2                 (0x40060000)                   /**< \brief (TWIHS2    ) Base Address */
#define UART0                  (0x400E0800)                   /**< \brief (UART0     ) Base Address */
#define UART1                  (0x400E0A00)                   /**< \brief (UART1     ) Base Address */
#define UART2                  (0x400E1A00)                   /**< \brief (UART2     ) Base Address */
#define UART3                  (0x400E1C00)                   /**< \brief (UART3     ) Base Address */
#define UART4                  (0x400E1E00)                   /**< \brief (UART4     ) Base Address */
#define USART0                 (0x40024000)                   /**< \brief (USART0    ) Base Address */
#define USART1                 (0x40028000)                   /**< \brief (USART1    ) Base Address */
#define USART2                 (0x4002C000)                   /**< \brief (USART2    ) Base Address */
#define USBHS                  (0x40038000)                   /**< \brief (USBHS     ) Base Address */
#define UTMI                   (0x400E0400)                   /**< \brief (UTMI      ) Base Address */
#define WDT                    (0x400E1850)                   /**< \brief (WDT       ) Base Address */
#define XDMAC                  (0x40078000)                   /**< \brief (XDMAC     ) Base Address */

#else /* For C/C++ compiler */

#define ACC                    ((Acc *)0x40044000U)           /**< \brief (ACC       ) Base Address */
#define ACC_INST_NUM           1                              /**< \brief (ACC       ) Number of instances */
#define ACC_INSTS              { ACC }                        /**< \brief (ACC       ) Instances List */

#define AES                    ((Aes *)0x4006C000U)           /**< \brief (AES       ) Base Address */
#define AES_INST_NUM           1                              /**< \brief (AES       ) Number of instances */
#define AES_INSTS              { AES }                        /**< \brief (AES       ) Instances List */

#define AFEC0                  ((Afec *)0x4003C000U)          /**< \brief (AFEC0     ) Base Address */
#define AFEC1                  ((Afec *)0x40064000U)          /**< \brief (AFEC1     ) Base Address */
#define AFEC_INST_NUM          2                              /**< \brief (AFEC      ) Number of instances */
#define AFEC_INSTS             { AFEC0, AFEC1 }               /**< \brief (AFEC      ) Instances List */

#define CHIPID                 ((Chipid *)0x400E0940U)        /**< \brief (CHIPID    ) Base Address */
#define CHIPID_INST_NUM        1                              /**< \brief (CHIPID    ) Number of instances */
#define CHIPID_INSTS           { CHIPID }                     /**< \brief (CHIPID    ) Instances List */

#define DACC                   ((Dacc *)0x40040000U)          /**< \brief (DACC      ) Base Address */
#define DACC_INST_NUM          1                              /**< \brief (DACC      ) Number of instances */
#define DACC_INSTS             { DACC }                       /**< \brief (DACC      ) Instances List */

#define EFC                    ((Efc *)0x400E0C00U)           /**< \brief (EFC       ) Base Address */
#define EFC_INST_NUM           1                              /**< \brief (EFC       ) Number of instances */
#define EFC_INSTS              { EFC }                        /**< \brief (EFC       ) Instances List */

#define GMAC                   ((Gmac *)0x40050000U)          /**< \brief (GMAC      ) Base Address */
#define GMAC_INST_NUM          1                              /**< \brief (GMAC      ) Number of instances */
#define GMAC_INSTS             { GMAC }                       /**< \brief (GMAC      ) Instances List */

#define GPBR                   ((Gpbr *)0x400E1890U)          /**< \brief (GPBR      ) Base Address */
#define GPBR_INST_NUM          1                              /**< \brief (GPBR      ) Number of instances */
#define GPBR_INSTS             { GPBR }                       /**< \brief (GPBR      ) Instances List */

#define HSMCI                  ((Hsmci *)0x40000000U)         /**< \brief (HSMCI     ) Base Address */
#define HSMCI_INST_NUM         1                              /**< \brief (HSMCI     ) Number of instances */
#define HSMCI_INSTS            { HSMCI }                      /**< \brief (HSMCI     ) Instances List */

#define ICM                    ((Icm *)0x40048000U)           /**< \brief (ICM       ) Base Address */
#define ICM_INST_NUM           1                              /**< \brief (ICM       ) Number of instances */
#define ICM_INSTS              { ICM }                        /**< \brief (ICM       ) Instances List */

#define ISI                    ((Isi *)0x4004C000U)           /**< \brief (ISI       ) Base Address */
#define ISI_INST_NUM           1                              /**< \brief (ISI       ) Number of instances */
#define ISI_INSTS              { ISI }                        /**< \brief (ISI       ) Instances List */

#define MATRIX                 ((Matrix *)0x40088000U)        /**< \brief (MATRIX    ) Base Address */
#define MATRIX_INST_NUM        1                              /**< \brief (MATRIX    ) Number of instances */
#define MATRIX_INSTS           { MATRIX }                     /**< \brief (MATRIX    ) Instances List */

#define MCAN0                  ((Mcan *)0x40030000U)          /**< \brief (MCAN0     ) Base Address */
#define MCAN1                  ((Mcan *)0x40034000U)          /**< \brief (MCAN1     ) Base Address */
#define MCAN_INST_NUM          2                              /**< \brief (MCAN      ) Number of instances */
#define MCAN_INSTS             { MCAN0, MCAN1 }               /**< \brief (MCAN      ) Instances List */

#define PIOA                   ((Pio *)0x400E0E00U)           /**< \brief (PIOA      ) Base Address */
#define PIOB                   ((Pio *)0x400E1000U)           /**< \brief (PIOB      ) Base Address */
#define PIOC                   ((Pio *)0x400E1200U)           /**< \brief (PIOC      ) Base Address */
#define PIOD                   ((Pio *)0x400E1400U)           /**< \brief (PIOD      ) Base Address */
#define PIOE                   ((Pio *)0x400E1600U)           /**< \brief (PIOE      ) Base Address */
#define PIO_INST_NUM           5                              /**< \brief (PIO       ) Number of instances */
#define PIO_INSTS              { PIOA, PIOB, PIOC, PIOD, PIOE } /**< \brief (PIO       ) Instances List */

#define PMC                    ((Pmc *)0x400E0600U)           /**< \brief (PMC       ) Base Address */
#define PMC_INST_NUM           1                              /**< \brief (PMC       ) Number of instances */
#define PMC_INSTS              { PMC }                        /**< \brief (PMC       ) Instances List */

#define PWM0                   ((Pwm *)0x40020000U)           /**< \brief (PWM0      ) Base Address */
#define PWM1                   ((Pwm *)0x4005C000U)           /**< \brief (PWM1      ) Base Address */
#define PWM_INST_NUM           2                              /**< \brief (PWM       ) Number of instances */
#define PWM_INSTS              { PWM0, PWM1 }                 /**< \brief (PWM       ) Instances List */

#define QSPI                   ((Qspi *)0x4007C000U)          /**< \brief (QSPI      ) Base Address */
#define QSPI_INST_NUM          1                              /**< \brief (QSPI      ) Number of instances */
#define QSPI_INSTS             { QSPI }                       /**< \brief (QSPI      ) Instances List */

#define RSTC                   ((Rstc *)0x400E1800U)          /**< \brief (RSTC      ) Base Address */
#define RSTC_INST_NUM          1                              /**< \brief (RSTC      ) Number of instances */
#define RSTC_INSTS             { RSTC }                       /**< \brief (RSTC      ) Instances List */

#define RSWDT                  ((Rswdt *)0x400E1900U)         /**< \brief (RSWDT     ) Base Address */
#define RSWDT_INST_NUM         1                              /**< \brief (RSWDT     ) Number of instances */
#define RSWDT_INSTS            { RSWDT }                      /**< \brief (RSWDT     ) Instances List */

#define RTC                    ((Rtc *)0x400E1860U)           /**< \brief (RTC       ) Base Address */
#define RTC_INST_NUM           1                              /**< \brief (RTC       ) Number of instances */
#define RTC_INSTS              { RTC }                        /**< \brief (RTC       ) Instances List */

#define RTT                    ((Rtt *)0x400E1830U)           /**< \brief (RTT       ) Base Address */
#define RTT_INST_NUM           1                              /**< \brief (RTT       ) Number of instances */
#define RTT_INSTS              { RTT }                        /**< \brief (RTT       ) Instances List */

#define SDRAMC                 ((Sdramc *)0x40084000U)        /**< \brief (SDRAMC    ) Base Address */
#define SDRAMC_INST_NUM        1                              /**< \brief (SDRAMC    ) Number of instances */
#define SDRAMC_INSTS           { SDRAMC }                     /**< \brief (SDRAMC    ) Instances List */

#define SMC                    ((Smc *)0x40080000U)           /**< \brief (SMC       ) Base Address */
#define SMC_INST_NUM           1                              /**< \brief (SMC       ) Number of instances */
#define SMC_INSTS              { SMC }                        /**< \brief (SMC       ) Instances List */

#define SPI0                   ((Spi *)0x40008000U)           /**< \brief (SPI0      ) Base Address */
#define SPI1                   ((Spi *)0x40058000U)           /**< \brief (SPI1      ) Base Address */
#define SPI_INST_NUM           2                              /**< \brief (SPI       ) Number of instances */
#define SPI_INSTS              { SPI0, SPI1 }                 /**< \brief (SPI       ) Instances List */

#define SSC                    ((Ssc *)0x40004000U)           /**< \brief (SSC       ) Base Address */
#define SSC_INST_NUM           1                              /**< \brief (SSC       ) Number of instances */
#define SSC_INSTS              { SSC }                        /**< \brief (SSC       ) Instances List */

#define SUPC                   ((Supc *)0x400E1810U)          /**< \brief (SUPC      ) Base Address */
#define SUPC_INST_NUM          1                              /**< \brief (SUPC      ) Number of instances */
#define SUPC_INSTS             { SUPC }                       /**< \brief (SUPC      ) Instances List */

#define TC0                    ((Tc *)0x4000C000U)            /**< \brief (TC0       ) Base Address */
#define TC1                    ((Tc *)0x40010000U)            /**< \brief (TC1       ) Base Address */
#define TC2                    ((Tc *)0x40014000U)            /**< \brief (TC2       ) Base Address */
#define TC3                    ((Tc *)0x40054000U)            /**< \brief (TC3       ) Base Address */
#define TC_INST_NUM            4                              /**< \brief (TC        ) Number of instances */
#define TC_INSTS               { TC0, TC1, TC2, TC3 }         /**< \brief (TC        ) Instances List */

#define TRNG                   ((Trng *)0x40070000U)          /**< \brief (TRNG      ) Base Address */
#define TRNG_INST_NUM          1                              /**< \brief (TRNG      ) Number of instances */
#define TRNG_INSTS             { TRNG }                       /**< \brief (TRNG      ) Instances List */

#define TWIHS0                 ((Twihs *)0x40018000U)         /**< \brief (TWIHS0    ) Base Address */
#define TWIHS1                 ((Twihs *)0x4001C000U)         /**< \brief (TWIHS1    ) Base Address */
#define TWIHS2                 ((Twihs *)0x40060000U)         /**< \brief (TWIHS2    ) Base Address */
#define TWIHS_INST_NUM         3                              /**< \brief (TWIHS     ) Number of instances */
#define TWIHS_INSTS            { TWIHS0, TWIHS1, TWIHS2 }     /**< \brief (TWIHS     ) Instances List */

#define UART0                  ((Uart *)0x400E0800U)          /**< \brief (UART0     ) Base Address */
#define UART1                  ((Uart *)0x400E0A00U)          /**< \brief (UART1     ) Base Address */
#define UART2                  ((Uart *)0x400E1A00U)          /**< \brief (UART2     ) Base Address */
#define UART3                  ((Uart *)0x400E1C00U)          /**< \brief (UART3     ) Base Address */
#define UART4                  ((Uart *)0x400E1E00U)          /**< \brief (UART4     ) Base Address */
#define UART_INST_NUM          5                              /**< \brief (UART      ) Number of instances */
#define UART_INSTS             { UART0, UART1, UART2, UART3, UART4 } /**< \brief (UART      ) Instances List */

#define USART0                 ((Usart *)0x40024000U)         /**< \brief (USART0    ) Base Address */
#define USART1                 ((Usart *)0x40028000U)         /**< \brief (USART1    ) Base Address */
#define USART2                 ((Usart *)0x4002C000U)         /**< \brief (USART2    ) Base Address */
#define USART_INST_NUM         3                              /**< \brief (USART     ) Number of instances */
#define USART_INSTS            { USART0, USART1, USART2 }     /**< \brief (USART     ) Instances List */

#define USBHS                  ((Usbhs *)0x40038000U)         /**< \brief (USBHS     ) Base Address */
#define USBHS_INST_NUM         1                              /**< \brief (USBHS     ) Number of instances */
#define USBHS_INSTS            { USBHS }                      /**< \brief (USBHS     ) Instances List */

#define UTMI                   ((Utmi *)0x400E0400U)          /**< \brief (UTMI      ) Base Address */
#define UTMI_INST_NUM          1                              /**< \brief (UTMI      ) Number of instances */
#define UTMI_INSTS             { UTMI }                       /**< \brief (UTMI      ) Instances List */

#define WDT                    ((Wdt *)0x400E1850U)           /**< \brief (WDT       ) Base Address */
#define WDT_INST_NUM           1                              /**< \brief (WDT       ) Number of instances */
#define WDT_INSTS              { WDT }                        /**< \brief (WDT       ) Instances List */

#define XDMAC                  ((Xdmac *)0x40078000U)         /**< \brief (XDMAC     ) Base Address */
#define XDMAC_INST_NUM         1                              /**< \brief (XDMAC     ) Number of instances */
#define XDMAC_INSTS            { XDMAC }                      /**< \brief (XDMAC     ) Instances List */

#endif /* (defined(__ASSEMBLER__) || defined(__IAR_SYSTEMS_ASM__)) */
/** @}  end of Peripheral Base Address Definitions */

/** \addtogroup SAME70Q21_pio Peripheral Pio Definitions
 *  @{
 */

/* ************************************************************************** */
/*   PIO DEFINITIONS FOR SAME70Q21*/
/* ************************************************************************** */
/* ========== Peripheral I/O pin numbers ========== */
#define PIN_PA0                     (  0)  /**< Pin Number for PA0 */
#define PIN_PA1                     (  1)  /**< Pin Number for PA1 */
#define PIN_PA2                     (  2)  /**< Pin Number for PA2 */
#define PIN_PA3                     (  3)  /**< Pin Number for PA3 */
#define PIN_PA4                     (  4)  /**< Pin Number for PA4 */
#define PIN_PA5                     (  5)  /**< Pin Number for PA5 */
#define PIN_PA6                     (  6)  /**< Pin Number for PA6 */
#define PIN_PA7                     (  7)  /**< Pin Number for PA7 */
#define PIN_PA8                     (  8)  /**< Pin Number for PA8 */
#define PIN_PA9                     (  9)  /**< Pin Number for PA9 */
#define PIN_PA10                    ( 10)  /**< Pin Number for PA10 */
#define PIN_PA11                    ( 11)  /**< Pin Number for PA11 */
#define PIN_PA12                    ( 12)  /**< Pin Number for PA12 */
#define PIN_PA13                    ( 13)  /**< Pin Number for PA13 */
#define PIN_PA14                    ( 14)  /**< Pin Number for PA14 */
#define PIN_PA15                    ( 15)  /**< Pin Number for PA15 */
#define PIN_PA16                    ( 16)  /**< Pin Number for PA16 */
#define PIN_PA17                    ( 17)  /**< Pin Number for PA17 */
#define PIN_PA18                    ( 18)  /**< Pin Number for PA18 */
#define PIN_PA19                    ( 19)  /**< Pin Number for PA19 */
#define PIN_PA20                    ( 20)  /**< Pin Number for PA20 */
#define PIN_PA21                    ( 21)  /**< Pin Number for PA21 */
#define PIN_PA22                    ( 22)  /**< Pin Number for PA22 */
#define PIN_PA23                    ( 23)  /**< Pin Number for PA23 */
#define PIN_PA24                    ( 24)  /**< Pin Number for PA24 */
#define PIN_PA25                    ( 25)  /**< Pin Number for PA25 */
#define PIN_PA26                    ( 26)  /**< Pin Number for PA26 */
#define PIN_PA27                    ( 27)  /**< Pin Number for PA27 */
#define PIN_PA28                    ( 28)  /**< Pin Number for PA28 */
#define PIN_PA29                    ( 29)  /**< Pin Number for PA29 */
#define PIN_PA30                    ( 30)  /**< Pin Number for PA30 */
#define PIN_PA31                    ( 31)  /**< Pin Number for PA31 */
#define PIN_PB0                     ( 32)  /**< Pin Number for PB0 */
#define PIN_PB1                     ( 33)  /**< Pin Number for PB1 */
#define PIN_PB2                     ( 34)  /**< Pin Number for PB2 */
#define PIN_PB3                     ( 35)  /**< Pin Number for PB3 */
#define PIN_PB4                     ( 36)  /**< Pin Number for PB4 */
#define PIN_PB5                     ( 37)  /**< Pin Number for PB5 */
#define PIN_PB6                     ( 38)  /**< Pin Number for PB6 */
#define PIN_PB7                     ( 39)  /**< Pin Number for PB7 */
#define PIN_PB8                     ( 40)  /**< Pin Number for PB8 */
#define PIN_PB9                     ( 41)  /**< Pin Number for PB9 */
#define PIN_PB12                    ( 44)  /**< Pin Number for PB12 */
#define PIN_PB13                    ( 45)  /**< Pin Number for PB13 */
#define PIN_PC0                     ( 64)  /**< Pin Number for PC0 */
#define PIN_PC1                     ( 65)  /**< Pin Number for PC1 */
#define PIN_PC2                     ( 66)  /**< Pin Number for PC2 */
#define PIN_PC3                     ( 67)  /**< Pin Number for PC3 */
#define PIN_PC4                     ( 68)  /**< Pin Number for PC4 */
#define PIN_PC5                     ( 69)  /**< Pin Number for PC5 */
#define PIN_PC6                     ( 70)  /**< Pin Number for PC6 */
#define PIN_PC7                     ( 71)  /**< Pin Number for PC7 */
#define PIN_PC8                     ( 72)  /**< Pin Number for PC8 */
#define PIN_PC9                     ( 73)  /**< Pin Number for PC9 */
#define PIN_PC10                    ( 74)  /**< Pin Number for PC10 */
#define PIN_PC11                    ( 75)  /**< Pin Number for PC11 */
#define PIN_PC12                    ( 76)  /**< Pin Number for PC12 */
#define PIN_PC13                    ( 77)  /**< Pin Number for PC13 */
#define PIN_PC14                    ( 78)  /**< Pin Number for PC14 */
#define PIN_PC15                    ( 79)  /**< Pin Number for PC15 */
#define PIN_PC16                    ( 80)  /**< Pin Number for PC16 */
#define PIN_PC17                    ( 81)  /**< Pin Number for PC17 */
#define PIN_PC18                    ( 82)  /**< Pin Number for PC18 */
#define PIN_PC19                    ( 83)  /**< Pin Number for PC19 */
#define PIN_PC20                    ( 84)  /**< Pin Number for PC20 */
#define PIN_PC21                    ( 85)  /**< Pin Number for PC21 */
#define PIN_PC22                    ( 86)  /**< Pin Number for PC22 */
#define PIN_PC23                    ( 87)  /**< Pin Number for PC23 */
#define PIN_PC24                    ( 88)  /**< Pin Number for PC24 */
#define PIN_PC25                    ( 89)  /**< Pin Number for PC25 */
#define PIN_PC26                    ( 90)  /**< Pin Number for PC26 */
#define PIN_PC27                    ( 91)  /**< Pin Number for PC27 */
#define PIN_PC28                    ( 92)  /**< Pin Number for PC28 */
#define PIN_PC29                    ( 93)  /**< Pin Number for PC29 */
#define PIN_PC30                    ( 94)  /**< Pin Number for PC30 */
#define PIN_PC31                    ( 95)  /**< Pin Number for PC31 */
#define PIN_PD0                     ( 96)  /**< Pin Number for PD0 */
#define PIN_PD1                     ( 97)  /**< Pin Number for PD1 */
#define PIN_PD2                     ( 98)  /**< Pin Number for PD2 */
#define PIN_PD3                     ( 99)  /**< Pin Number for PD3 */
#define PIN_PD4                     (100)  /**< Pin Number for PD4 */
#define PIN_PD5                     (101)  /**< Pin Number for PD5 */
#define PIN_PD6                     (102)  /**< Pin Number for PD6 */
#define PIN_PD7                     (103)  /**< Pin Number for PD7 */
#define PIN_PD8                     (104)  /**< Pin Number for PD8 */
#define PIN_PD9                     (105)  /**< Pin Number for PD9 */
#define PIN_PD10                    (106)  /**< Pin Number for PD10 */
#define PIN_PD11                    (107)  /**< Pin Number for PD11 */
#define PIN_PD12                    (108)  /**< Pin Number for PD12 */
#define PIN_PD13                    (109)  /**< Pin Number for PD13 */
#define PIN_PD14                    (110)  /**< Pin Number for PD14 */
#define PIN_PD15                    (111)  /**< Pin Number for PD15 */
#define PIN_PD16                    (112)  /**< Pin Number for PD16 */
#define PIN_PD17                    (113)  /**< Pin Number for PD17 */
#define PIN_PD18                    (114)  /**< Pin Number for PD18 */
#define PIN_PD19                    (115)  /**< Pin Number for PD19 */
#define PIN_PD20                    (116)  /**< Pin Number for PD20 */
#define PIN_PD21                    (117)  /**< Pin Number for PD21 */
#define PIN_PD22                    (118)  /**< Pin Number for PD22 */
#define PIN_PD23                    (119)  /**< Pin Number for PD23 */
#define PIN_PD24                    (120)  /**< Pin Number for PD24 */
#define PIN_PD25                    (121)  /**< Pin Number for PD25 */
#define PIN_PD26                    (122)  /**< Pin Number for PD26 */
#define PIN_PD27                    (123)  /**< Pin Number for PD27 */
#define PIN_PD28                    (124)  /**< Pin Number for PD28 */
#define PIN_PD29                    (125)  /**< Pin Number for PD29 */
#define PIN_PD30                    (126)  /**< Pin Number for PD30 */
#define PIN_PD31                    (127)  /**< Pin Number for PD31 */
#define PIN_PE0                     (128)  /**< Pin Number for PE0 */
#define PIN_PE1                     (129)  /**< Pin Number for PE1 */
#define PIN_PE2                     (130)  /**< Pin Number for PE2 */
#define PIN_PE3                     (131)  /**< Pin Number for PE3 */
#define PIN_PE4                     (132)  /**< Pin Number for PE4 */
#define PIN_PE5                     (133)  /**< Pin Number for PE5 */


/* ========== Peripheral I/O masks ========== */
#define PIO_PA0                     (_U_(1) << 0) /**< PIO Mask for PA0 */
#define PIO_PA1                     (_U_(1) << 1) /**< PIO Mask for PA1 */
#define PIO_PA2                     (_U_(1) << 2) /**< PIO Mask for PA2 */
#define PIO_PA3                     (_U_(1) << 3) /**< PIO Mask for PA3 */
#define PIO_PA4                     (_U_(1) << 4) /**< PIO Mask for PA4 */
#define PIO_PA5                     (_U_(1) << 5) /**< PIO Mask for PA5 */
#define PIO_PA6                     (_U_(1) << 6) /**< PIO Mask for PA6 */
#define PIO_PA7                     (_U_(1) << 7) /**< PIO Mask for PA7 */
#define PIO_PA8                     (_U_(1) << 8) /**< PIO Mask for PA8 */
#define PIO_PA9                     (_U_(1) << 9) /**< PIO Mask for PA9 */
#define PIO_PA10                    (_U_(1) << 10) /**< PIO Mask for PA10 */
#define PIO_PA11                    (_U_(1) << 11) /**< PIO Mask for PA11 */
#define PIO_PA12                    (_U_(1) << 12) /**< PIO Mask for PA12 */
#define PIO_PA13                    (_U_(1) << 13) /**< PIO Mask for PA13 */
#define PIO_PA14                    (_U_(1) << 14) /**< PIO Mask for PA14 */
#define PIO_PA15                    (_U_(1) << 15) /**< PIO Mask for PA15 */
#define PIO_PA16                    (_U_(1) << 16) /**< PIO Mask for PA16 */
#define PIO_PA17                    (_U_(1) << 17) /**< PIO Mask for PA17 */
#define PIO_PA18                    (_U_(1) << 18) /**< PIO Mask for PA18 */
#define PIO_PA19                    (_U_(1) << 19) /**< PIO Mask for PA19 */
#define PIO_PA20                    (_U_(1) << 20) /**< PIO Mask for PA20 */
#define PIO_PA21                    (_U_(1) << 21) /**< PIO Mask for PA21 */
#define PIO_PA22                    (_U_(1) << 22) /**< PIO Mask for PA22 */
#define PIO_PA23                    (_U_(1) << 23) /**< PIO Mask for PA23 */
#define PIO_PA24                    (_U_(1) << 24) /**< PIO Mask for PA24 */
#define PIO_PA25                    (_U_(1) << 25) /**< PIO Mask for PA25 */
#define PIO_PA26                    (_U_(1) << 26) /**< PIO Mask for PA26 */
#define PIO_PA27                    (_U_(1) << 27) /**< PIO Mask for PA27 */
#define PIO_PA28                    (_U_(1) << 28) /**< PIO Mask for PA28 */
#define PIO_PA29                    (_U_(1) << 29) /**< PIO Mask for PA29 */
#define PIO_PA30                    (_U_(1) << 30) /**< PIO Mask for PA30 */
#define PIO_PA31                    (_U_(1) << 31) /**< PIO Mask for PA31 */
#define PIO_PB0                     (_U_(1) << 0) /**< PIO Mask for PB0 */
#define PIO_PB1                     (_U_(1) << 1) /**< PIO Mask for PB1 */
#define PIO_PB2                     (_U_(1) << 2) /**< PIO Mask for PB2 */
#define PIO_PB3                     (_U_(1) << 3) /**< PIO Mask for PB3 */
#define PIO_PB4                     (_U_(1) << 4) /**< PIO Mask for PB4 */
#define PIO_PB5                     (_U_(1) << 5) /**< PIO Mask for PB5 */
#define PIO_PB6                     (_U_(1) << 6) /**< PIO Mask for PB6 */
#define PIO_PB7                     (_U_(1) << 7) /**< PIO Mask for PB7 */
#define PIO_PB8                     (_U_(1) << 8) /**< PIO Mask for PB8 */
#define PIO_PB9                     (_U_(1) << 9) /**< PIO Mask for PB9 */
#define PIO_PB12                    (_U_(1) << 12) /**< PIO Mask for PB12 */
#define PIO_PB13                    (_U_(1) << 13) /**< PIO Mask for PB13 */
#define PIO_PC0                     (_U_(1) << 0) /**< PIO Mask for PC0 */
#define PIO_PC1                     (_U_(1) << 1) /**< PIO Mask for PC1 */
#define PIO_PC2                     (_U_(1) << 2) /**< PIO Mask for PC2 */
#define PIO_PC3                     (_U_(1) << 3) /**< PIO Mask for PC3 */
#define PIO_PC4                     (_U_(1) << 4) /**< PIO Mask for PC4 */
#define PIO_PC5                     (_U_(1) << 5) /**< PIO Mask for PC5 */
#define PIO_PC6                     (_U_(1) << 6) /**< PIO Mask for PC6 */
#define PIO_PC7                     (_U_(1) << 7) /**< PIO Mask for PC7 */
#define PIO_PC8                     (_U_(1) << 8) /**< PIO Mask for PC8 */
#define PIO_PC9                     (_U_(1) << 9) /**< PIO Mask for PC9 */
#define PIO_PC10                    (_U_(1) << 10) /**< PIO Mask for PC10 */
#define PIO_PC11                    (_U_(1) << 11) /**< PIO Mask for PC11 */
#define PIO_PC12                    (_U_(1) << 12) /**< PIO Mask for PC12 */
#define PIO_PC13                    (_U_(1) << 13) /**< PIO Mask for PC13 */
#define PIO_PC14                    (_U_(1) << 14) /**< PIO Mask for PC14 */
#define PIO_PC15                    (_U_(1) << 15) /**< PIO Mask for PC15 */
#define PIO_PC16                    (_U_(1) << 16) /**< PIO Mask for PC16 */
#define PIO_PC17                    (_U_(1) << 17) /**< PIO Mask for PC17 */
#define PIO_PC18                    (_U_(1) << 18) /**< PIO Mask for PC18 */
#define PIO_PC19                    (_U_(1) << 19) /**< PIO Mask for PC19 */
#define PIO_PC20                    (_U_(1) << 20) /**< PIO Mask for PC20 */
#define PIO_PC21                    (_U_(1) << 21) /**< PIO Mask for PC21 */
#define PIO_PC22                    (_U_(1) << 22) /**< PIO Mask for PC22 */
#define PIO_PC23                    (_U_(1) << 23) /**< PIO Mask for PC23 */
#define PIO_PC24                    (_U_(1) << 24) /**< PIO Mask for PC24 */
#define PIO_PC25                    (_U_(1) << 25) /**< PIO Mask for PC25 */
#define PIO_PC26                    (_U_(1) << 26) /**< PIO Mask for PC26 */
#define PIO_PC27                    (_U_(1) << 27) /**< PIO Mask for PC27 */
#define PIO_PC28                    (_U_(1) << 28) /**< PIO Mask for PC28 */
#define PIO_PC29                    (_U_(1) << 29) /**< PIO Mask for PC29 */
#define PIO_PC30                    (_U_(1) << 30) /**< PIO Mask for PC30 */
#define PIO_PC31                    (_U_(1) << 31) /**< PIO Mask for PC31 */
#define PIO_PD0                     (_U_(1) << 0) /**< PIO Mask for PD0 */
#define PIO_PD1                     (_U_(1) << 1) /**< PIO Mask for PD1 */
#define PIO_PD2                     (_U_(1) << 2) /**< PIO Mask for PD2 */
#define PIO_PD3                     (_U_(1) << 3) /**< PIO Mask for PD3 */
#define PIO_PD4                     (_U_(1) << 4) /**< PIO Mask for PD4 */
#define PIO_PD5                     (_U_(1) << 5) /**< PIO Mask for PD5 */
#define PIO_PD6                     (_U_(1) << 6) /**< PIO Mask for PD6 */
#define PIO_PD7                     (_U_(1) << 7) /**< PIO Mask for PD7 */
#define PIO_PD8                     (_U_(1) << 8) /**< PIO Mask for PD8 */
#define PIO_PD9                     (_U_(1) << 9) /**< PIO Mask for PD9 */
#define PIO_PD10                    (_U_(1) << 10) /**< PIO Mask for PD10 */
#define PIO_PD11                    (_U_(1) << 11) /**< PIO Mask for PD11 */
#define PIO_PD12                    (_U_(1) << 12) /**< PIO Mask for PD12 */
#define PIO_PD13                    (_U_(1) << 13) /**< PIO Mask for PD13 */
#define PIO_PD14                    (_U_(1) << 14) /**< PIO Mask for PD14 */
#define PIO_PD15                    (_U_(1) << 15) /**< PIO Mask for PD15 */
#define PIO_PD16                    (_U_(1) << 16) /**< PIO Mask for PD16 */
#define PIO_PD17                    (_U_(1) << 17) /**< PIO Mask for PD17 */
#define PIO_PD18                    (_U_(1) << 18) /**< PIO Mask for PD18 */
#define PIO_PD19                    (_U_(1) << 19) /**< PIO Mask for PD19 */
#define PIO_PD20                    (_U_(1) << 20) /**< PIO Mask for PD20 */
#define PIO_PD21                    (_U_(1) << 21) /**< PIO Mask for PD21 */
#define PIO_PD22                    (_U_(1) << 22) /**< PIO Mask for PD22 */
#define PIO_PD23                    (_U_(1) << 23) /**< PIO Mask for PD23 */
#define PIO_PD24                    (_U_(1) << 24) /**< PIO Mask for PD24 */
#define PIO_PD25                    (_U_(1) << 25) /**< PIO Mask for PD25 */
#define PIO_PD26                    (_U_(1) << 26) /**< PIO Mask for PD26 */
#define PIO_PD27                    (_U_(1) << 27) /**< PIO Mask for PD27 */
#define PIO_PD28                    (_U_(1) << 28) /**< PIO Mask for PD28 */
#define PIO_PD29                    (_U_(1) << 29) /**< PIO Mask for PD29 */
#define PIO_PD30                    (_U_(1) << 30) /**< PIO Mask for PD30 */
#define PIO_PD31                    (_U_(1) << 31) /**< PIO Mask for PD31 */
#define PIO_PE0                     (_U_(1) << 0) /**< PIO Mask for PE0 */
#define PIO_PE1                     (_U_(1) << 1) /**< PIO Mask for PE1 */
#define PIO_PE2                     (_U_(1) << 2) /**< PIO Mask for PE2 */
#define PIO_PE3                     (_U_(1) << 3) /**< PIO Mask for PE3 */
#define PIO_PE4                     (_U_(1) << 4) /**< PIO Mask for PE4 */
#define PIO_PE5                     (_U_(1) << 5) /**< PIO Mask for PE5 */


/* ========== Peripheral I/O indexes ========== */
#define PIO_PA0_IDX                 (  0)  /**< PIO Index Number for PA0 */
#define PIO_PA1_IDX                 (  1)  /**< PIO Index Number for PA1 */
#define PIO_PA2_IDX                 (  2)  /**< PIO Index Number for PA2 */
#define PIO_PA3_IDX                 (  3)  /**< PIO Index Number for PA3 */
#define PIO_PA4_IDX                 (  4)  /**< PIO Index Number for PA4 */
#define PIO_PA5_IDX                 (  5)  /**< PIO Index Number for PA5 */
#define PIO_PA6_IDX                 (  6)  /**< PIO Index Number for PA6 */
#define PIO_PA7_IDX                 (  7)  /**< PIO Index Number for PA7 */
#define PIO_PA8_IDX                 (  8)  /**< PIO Index Number for PA8 */
#define PIO_PA9_IDX                 (  9)  /**< PIO Index Number for PA9 */
#define PIO_PA10_IDX                ( 10)  /**< PIO Index Number for PA10 */
#define PIO_PA11_IDX                ( 11)  /**< PIO Index Number for PA11 */
#define PIO_PA12_IDX                ( 12)  /**< PIO Index Number for PA12 */
#define PIO_PA13_IDX                ( 13)  /**< PIO Index Number for PA13 */
#define PIO_PA14_IDX                ( 14)  /**< PIO Index Number for PA14 */
#define PIO_PA15_IDX                ( 15)  /**< PIO Index Number for PA15 */
#define PIO_PA16_IDX                ( 16)  /**< PIO Index Number for PA16 */
#define PIO_PA17_IDX                ( 17)  /**< PIO Index Number for PA17 */
#define PIO_PA18_IDX                ( 18)  /**< PIO Index Number for PA18 */
#define PIO_PA19_IDX                ( 19)  /**< PIO Index Number for PA19 */
#define PIO_PA20_IDX                ( 20)  /**< PIO Index Number for PA20 */
#define PIO_PA21_IDX                ( 21)  /**< PIO Index Number for PA21 */
#define PIO_PA22_IDX                ( 22)  /**< PIO Index Number for PA22 */
#define PIO_PA23_IDX                ( 23)  /**< PIO Index Number for PA23 */
#define PIO_PA24_IDX                ( 24)  /**< PIO Index Number for PA24 */
#define PIO_PA25_IDX                ( 25)  /**< PIO Index Number for PA25 */
#define PIO_PA26_IDX                ( 26)  /**< PIO Index Number for PA26 */
#define PIO_PA27_IDX                ( 27)  /**< PIO Index Number for PA27 */
#define PIO_PA28_IDX                ( 28)  /**< PIO Index Number for PA28 */
#define PIO_PA29_IDX                ( 29)  /**< PIO Index Number for PA29 */
#define PIO_PA30_IDX                ( 30)  /**< PIO Index Number for PA30 */
#define PIO_PA31_IDX                ( 31)  /**< PIO Index Number for PA31 */
#define PIO_PB0_IDX                 ( 32)  /**< PIO Index Number for PB0 */
#define PIO_PB1_IDX                 ( 33)  /**< PIO Index Number for PB1 */
#define PIO_PB2_IDX                 ( 34)  /**< PIO Index Number for PB2 */
#define PIO_PB3_IDX                 ( 35)  /**< PIO Index Number for PB3 */
#define PIO_PB4_IDX                 ( 36)  /**< PIO Index Number for PB4 */
#define PIO_PB5_IDX                 ( 37)  /**< PIO Index Number for PB5 */
#define PIO_PB6_IDX                 ( 38)  /**< PIO Index Number for PB6 */
#define PIO_PB7_IDX                 ( 39)  /**< PIO Index Number for PB7 */
#define PIO_PB8_IDX                 ( 40)  /**< PIO Index Number for PB8 */
#define PIO_PB9_IDX                 ( 41)  /**< PIO Index Number for PB9 */
#define PIO_PB12_IDX                ( 44)  /**< PIO Index Number for PB12 */
#define PIO_PB13_IDX                ( 45)  /**< PIO Index Number for PB13 */
#define PIO_PC0_IDX                 ( 64)  /**< PIO Index Number for PC0 */
#define PIO_PC1_IDX                 ( 65)  /**< PIO Index Number for PC1 */
#define PIO_PC2_IDX                 ( 66)  /**< PIO Index Number for PC2 */
#define PIO_PC3_IDX                 ( 67)  /**< PIO Index Number for PC3 */
#define PIO_PC4_IDX                 ( 68)  /**< PIO Index Number for PC4 */
#define PIO_PC5_IDX                 ( 69)  /**< PIO Index Number for PC5 */
#define PIO_PC6_IDX                 ( 70)  /**< PIO Index Number for PC6 */
#define PIO_PC7_IDX                 ( 71)  /**< PIO Index Number for PC7 */
#define PIO_PC8_IDX                 ( 72)  /**< PIO Index Number for PC8 */
#define PIO_PC9_IDX                 ( 73)  /**< PIO Index Number for PC9 */
#define PIO_PC10_IDX                ( 74)  /**< PIO Index Number for PC10 */
#define PIO_PC11_IDX                ( 75)  /**< PIO Index Number for PC11 */
#define PIO_PC12_IDX                ( 76)  /**< PIO Index Number for PC12 */
#define PIO_PC13_IDX                ( 77)  /**< PIO Index Number for PC13 */
#define PIO_PC14_IDX                ( 78)  /**< PIO Index Number for PC14 */
#define PIO_PC15_IDX                ( 79)  /**< PIO Index Number for PC15 */
#define PIO_PC16_IDX                ( 80)  /**< PIO Index Number for PC16 */
#define PIO_PC17_IDX                ( 81)  /**< PIO Index Number for PC17 */
#define PIO_PC18_IDX                ( 82)  /**< PIO Index Number for PC18 */
#define PIO_PC19_IDX                ( 83)  /**< PIO Index Number for PC19 */
#define PIO_PC20_IDX                ( 84)  /**< PIO Index Number for PC20 */
#define PIO_PC21_IDX                ( 85)  /**< PIO Index Number for PC21 */
#define PIO_PC22_IDX                ( 86)  /**< PIO Index Number for PC22 */
#define PIO_PC23_IDX                ( 87)  /**< PIO Index Number for PC23 */
#define PIO_PC24_IDX                ( 88)  /**< PIO Index Number for PC24 */
#define PIO_PC25_IDX                ( 89)  /**< PIO Index Number for PC25 */
#define PIO_PC26_IDX                ( 90)  /**< PIO Index Number for PC26 */
#define PIO_PC27_IDX                ( 91)  /**< PIO Index Number for PC27 */
#define PIO_PC28_IDX                ( 92)  /**< PIO Index Number for PC28 */
#define PIO_PC29_IDX                ( 93)  /**< PIO Index Number for PC29 */
#define PIO_PC30_IDX                ( 94)  /**< PIO Index Number for PC30 */
#define PIO_PC31_IDX                ( 95)  /**< PIO Index Number for PC31 */
#define PIO_PD0_IDX                 ( 96)  /**< PIO Index Number for PD0 */
#define PIO_PD1_IDX                 ( 97)  /**< PIO Index Number for PD1 */
#define PIO_PD2_IDX                 ( 98)  /**< PIO Index Number for PD2 */
#define PIO_PD3_IDX                 ( 99)  /**< PIO Index Number for PD3 */
#define PIO_PD4_IDX                 (100)  /**< PIO Index Number for PD4 */
#define PIO_PD5_IDX                 (101)  /**< PIO Index Number for PD5 */
#define PIO_PD6_IDX                 (102)  /**< PIO Index Number for PD6 */
#define PIO_PD7_IDX                 (103)  /**< PIO Index Number for PD7 */
#define PIO_PD8_IDX                 (104)  /**< PIO Index Number for PD8 */
#define PIO_PD9_IDX                 (105)  /**< PIO Index Number for PD9 */
#define PIO_PD10_IDX                (106)  /**< PIO Index Number for PD10 */
#define PIO_PD11_IDX                (107)  /**< PIO Index Number for PD11 */
#define PIO_PD12_IDX                (108)  /**< PIO Index Number for PD12 */
#define PIO_PD13_IDX                (109)  /**< PIO Index Number for PD13 */
#define PIO_PD14_IDX                (110)  /**< PIO Index Number for PD14 */
#define PIO_PD15_IDX                (111)  /**< PIO Index Number for PD15 */
#define PIO_PD16_IDX                (112)  /**< PIO Index Number for PD16 */
#define PIO_PD17_IDX                (113)  /**< PIO Index Number for PD17 */
#define PIO_PD18_IDX                (114)  /**< PIO Index Number for PD18 */
#define PIO_PD19_IDX                (115)  /**< PIO Index Number for PD19 */
#define PIO_PD20_IDX                (116)  /**< PIO Index Number for PD20 */
#define PIO_PD21_IDX                (117)  /**< PIO Index Number for PD21 */
#define PIO_PD22_IDX                (118)  /**< PIO Index Number for PD22 */
#define PIO_PD23_IDX                (119)  /**< PIO Index Number for PD23 */
#define PIO_PD24_IDX                (120)  /**< PIO Index Number for PD24 */
#define PIO_PD25_IDX                (121)  /**< PIO Index Number for PD25 */
#define PIO_PD26_IDX                (122)  /**< PIO Index Number for PD26 */
#define PIO_PD27_IDX                (123)  /**< PIO Index Number for PD27 */
#define PIO_PD28_IDX                (124)  /**< PIO Index Number for PD28 */
#define PIO_PD29_IDX                (125)  /**< PIO Index Number for PD29 */
#define PIO_PD30_IDX                (126)  /**< PIO Index Number for PD30 */
#define PIO_PD31_IDX                (127)  /**< PIO Index Number for PD31 */
#define PIO_PE0_IDX                 (128)  /**< PIO Index Number for PE0 */
#define PIO_PE1_IDX                 (129)  /**< PIO Index Number for PE1 */
#define PIO_PE2_IDX                 (130)  /**< PIO Index Number for PE2 */
#define PIO_PE3_IDX                 (131)  /**< PIO Index Number for PE3 */
#define PIO_PE4_IDX                 (132)  /**< PIO Index Number for PE4 */
#define PIO_PE5_IDX                 (133)  /**< PIO Index Number for PE5 */

/* ========== PIO definition for AFEC0 peripheral ========== */
#define PIN_PA8B_AFEC0_ADTRG                       _L_(8)       /**< AFEC0 signal: ADTRG on PA8 mux B*/
#define MUX_PA8B_AFEC0_ADTRG                       _L_(1)       /**< AFEC0 signal line function value: ADTRG */
#define PIO_PA8B_AFEC0_ADTRG                       (_UL_(1) << 8)

#define PIN_PD30X1_AFEC0_AD0                       _L_(126)     /**< AFEC0 signal: AD0 on PD30 mux X1*/
#define PIO_PD30X1_AFEC0_AD0                       (_UL_(1) << 30)

#define PIN_PA21X1_AFEC0_AD1                       _L_(21)      /**< AFEC0 signal: AD1 on PA21 mux X1*/
#define PIO_PA21X1_AFEC0_AD1                       (_UL_(1) << 21)

#define PIN_PA21X1_AFEC0_PIODCEN2                  _L_(21)      /**< AFEC0 signal: PIODCEN2 on PA21 mux X1*/
#define PIO_PA21X1_AFEC0_PIODCEN2                  (_UL_(1) << 21)

#define PIN_PB3X1_AFEC0_AD2                        _L_(35)      /**< AFEC0 signal: AD2 on PB3 mux X1*/
#define PIO_PB3X1_AFEC0_AD2                        (_UL_(1) << 3)

#define PIN_PB3X1_AFEC0_WKUP12                     _L_(35)      /**< AFEC0 signal: WKUP12 on PB3 mux X1*/
#define PIO_PB3X1_AFEC0_WKUP12                     (_UL_(1) << 3)

#define PIN_PE5X1_AFEC0_AD3                        _L_(133)     /**< AFEC0 signal: AD3 on PE5 mux X1*/
#define PIO_PE5X1_AFEC0_AD3                        (_UL_(1) << 5)

#define PIN_PE4X1_AFEC0_AD4                        _L_(132)     /**< AFEC0 signal: AD4 on PE4 mux X1*/
#define PIO_PE4X1_AFEC0_AD4                        (_UL_(1) << 4)

#define PIN_PB2X1_AFEC0_AD5                        _L_(34)      /**< AFEC0 signal: AD5 on PB2 mux X1*/
#define PIO_PB2X1_AFEC0_AD5                        (_UL_(1) << 2)

#define PIN_PA17X1_AFEC0_AD6                       _L_(17)      /**< AFEC0 signal: AD6 on PA17 mux X1*/
#define PIO_PA17X1_AFEC0_AD6                       (_UL_(1) << 17)

#define PIN_PA18X1_AFEC0_AD7                       _L_(18)      /**< AFEC0 signal: AD7 on PA18 mux X1*/
#define PIO_PA18X1_AFEC0_AD7                       (_UL_(1) << 18)

#define PIN_PA19X1_AFEC0_AD8                       _L_(19)      /**< AFEC0 signal: AD8 on PA19 mux X1*/
#define PIO_PA19X1_AFEC0_AD8                       (_UL_(1) << 19)

#define PIN_PA19X1_AFEC0_WKUP9                     _L_(19)      /**< AFEC0 signal: WKUP9 on PA19 mux X1*/
#define PIO_PA19X1_AFEC0_WKUP9                     (_UL_(1) << 19)

#define PIN_PA20X1_AFEC0_AD9                       _L_(20)      /**< AFEC0 signal: AD9 on PA20 mux X1*/
#define PIO_PA20X1_AFEC0_AD9                       (_UL_(1) << 20)

#define PIN_PA20X1_AFEC0_WKUP10                    _L_(20)      /**< AFEC0 signal: WKUP10 on PA20 mux X1*/
#define PIO_PA20X1_AFEC0_WKUP10                    (_UL_(1) << 20)

#define PIN_PB0X1_AFEC0_AD10                       _L_(32)      /**< AFEC0 signal: AD10 on PB0 mux X1*/
#define PIO_PB0X1_AFEC0_AD10                       (_UL_(1) << 0)

#define PIN_PB0X1_AFEC0_RTCOUT0                    _L_(32)      /**< AFEC0 signal: RTCOUT0 on PB0 mux X1*/
#define PIO_PB0X1_AFEC0_RTCOUT0                    (_UL_(1) << 0)

/* ========== PIO definition for AFEC1 peripheral ========== */
#define PIN_PD9C_AFEC1_ADTRG                       _L_(105)     /**< AFEC1 signal: ADTRG on PD9 mux C*/
#define MUX_PD9C_AFEC1_ADTRG                       _L_(2)       /**< AFEC1 signal line function value: ADTRG */
#define PIO_PD9C_AFEC1_ADTRG                       (_UL_(1) << 9)

#define PIN_PB1X1_AFEC1_AD0                        _L_(33)      /**< AFEC1 signal: AD0 on PB1 mux X1*/
#define PIO_PB1X1_AFEC1_AD0                        (_UL_(1) << 1)

#define PIN_PB1X1_AFEC1_RTCOUT1                    _L_(33)      /**< AFEC1 signal: RTCOUT1 on PB1 mux X1*/
#define PIO_PB1X1_AFEC1_RTCOUT1                    (_UL_(1) << 1)

#define PIN_PC13X1_AFEC1_AD1                       _L_(77)      /**< AFEC1 signal: AD1 on PC13 mux X1*/
#define PIO_PC13X1_AFEC1_AD1                       (_UL_(1) << 13)

#define PIN_PC15X1_AFEC1_AD2                       _L_(79)      /**< AFEC1 signal: AD2 on PC15 mux X1*/
#define PIO_PC15X1_AFEC1_AD2                       (_UL_(1) << 15)

#define PIN_PC12X1_AFEC1_AD3                       _L_(76)      /**< AFEC1 signal: AD3 on PC12 mux X1*/
#define PIO_PC12X1_AFEC1_AD3                       (_UL_(1) << 12)

#define PIN_PC29X1_AFEC1_AD4                       _L_(93)      /**< AFEC1 signal: AD4 on PC29 mux X1*/
#define PIO_PC29X1_AFEC1_AD4                       (_UL_(1) << 29)

#define PIN_PC30X1_AFEC1_AD5                       _L_(94)      /**< AFEC1 signal: AD5 on PC30 mux X1*/
#define PIO_PC30X1_AFEC1_AD5                       (_UL_(1) << 30)

#define PIN_PC31X1_AFEC1_AD6                       _L_(95)      /**< AFEC1 signal: AD6 on PC31 mux X1*/
#define PIO_PC31X1_AFEC1_AD6                       (_UL_(1) << 31)

#define PIN_PC26X1_AFEC1_AD7                       _L_(90)      /**< AFEC1 signal: AD7 on PC26 mux X1*/
#define PIO_PC26X1_AFEC1_AD7                       (_UL_(1) << 26)

#define PIN_PC27X1_AFEC1_AD8                       _L_(91)      /**< AFEC1 signal: AD8 on PC27 mux X1*/
#define PIO_PC27X1_AFEC1_AD8                       (_UL_(1) << 27)

#define PIN_PC0X1_AFEC1_AD9                        _L_(64)      /**< AFEC1 signal: AD9 on PC0 mux X1*/
#define PIO_PC0X1_AFEC1_AD9                        (_UL_(1) << 0)

#define PIN_PE3X1_AFEC1_AD10                       _L_(131)     /**< AFEC1 signal: AD10 on PE3 mux X1*/
#define PIO_PE3X1_AFEC1_AD10                       (_UL_(1) << 3)

#define PIN_PE0X1_AFEC1_AD11                       _L_(128)     /**< AFEC1 signal: AD11 on PE0 mux X1*/
#define PIO_PE0X1_AFEC1_AD11                       (_UL_(1) << 0)

/* ========== PIO definition for DACC peripheral ========== */
#define PIN_PB13X1_DACC_DAC0                       _L_(45)      /**< DACC signal: DAC0 on PB13 mux X1*/
#define PIO_PB13X1_DACC_DAC0                       (_UL_(1) << 13)

#define PIN_PD0X1_DACC_DAC1                        _L_(96)      /**< DACC signal: DAC1 on PD0 mux X1*/
#define PIO_PD0X1_DACC_DAC1                        (_UL_(1) << 0)

#define PIN_PA2C_DACC_DATRG                        _L_(2)       /**< DACC signal: DATRG on PA2 mux C*/
#define MUX_PA2C_DACC_DATRG                        _L_(2)       /**< DACC signal line function value: DATRG */
#define PIO_PA2C_DACC_DATRG                        (_UL_(1) << 2)

/* ========== PIO definition for GMAC peripheral ========== */
#define PIN_PD13A_GMAC_GCOL                        _L_(109)     /**< GMAC signal: GCOL on PD13 mux A*/
#define MUX_PD13A_GMAC_GCOL                        _L_(0)       /**< GMAC signal line function value: GCOL */
#define PIO_PD13A_GMAC_GCOL                        (_UL_(1) << 13)

#define PIN_PD10A_GMAC_GCRS                        _L_(106)     /**< GMAC signal: GCRS on PD10 mux A*/
#define MUX_PD10A_GMAC_GCRS                        _L_(0)       /**< GMAC signal line function value: GCRS */
#define PIO_PD10A_GMAC_GCRS                        (_UL_(1) << 10)

#define PIN_PD8A_GMAC_GMDC                         _L_(104)     /**< GMAC signal: GMDC on PD8 mux A*/
#define MUX_PD8A_GMAC_GMDC                         _L_(0)       /**< GMAC signal line function value: GMDC */
#define PIO_PD8A_GMAC_GMDC                         (_UL_(1) << 8)

#define PIN_PD9A_GMAC_GMDIO                        _L_(105)     /**< GMAC signal: GMDIO on PD9 mux A*/
#define MUX_PD9A_GMAC_GMDIO                        _L_(0)       /**< GMAC signal line function value: GMDIO */
#define PIO_PD9A_GMAC_GMDIO                        (_UL_(1) << 9)

#define PIN_PD14A_GMAC_GRXCK                       _L_(110)     /**< GMAC signal: GRXCK on PD14 mux A*/
#define MUX_PD14A_GMAC_GRXCK                       _L_(0)       /**< GMAC signal line function value: GRXCK */
#define PIO_PD14A_GMAC_GRXCK                       (_UL_(1) << 14)

#define PIN_PD4A_GMAC_GRXDV                        _L_(100)     /**< GMAC signal: GRXDV on PD4 mux A*/
#define MUX_PD4A_GMAC_GRXDV                        _L_(0)       /**< GMAC signal line function value: GRXDV */
#define PIO_PD4A_GMAC_GRXDV                        (_UL_(1) << 4)

#define PIN_PD7A_GMAC_GRXER                        _L_(103)     /**< GMAC signal: GRXER on PD7 mux A*/
#define MUX_PD7A_GMAC_GRXER                        _L_(0)       /**< GMAC signal line function value: GRXER */
#define PIO_PD7A_GMAC_GRXER                        (_UL_(1) << 7)

#define PIN_PD5A_GMAC_GRX0                         _L_(101)     /**< GMAC signal: GRX0 on PD5 mux A*/
#define MUX_PD5A_GMAC_GRX0                         _L_(0)       /**< GMAC signal line function value: GRX0 */
#define PIO_PD5A_GMAC_GRX0                         (_UL_(1) << 5)

#define PIN_PD6A_GMAC_GRX1                         _L_(102)     /**< GMAC signal: GRX1 on PD6 mux A*/
#define MUX_PD6A_GMAC_GRX1                         _L_(0)       /**< GMAC signal line function value: GRX1 */
#define PIO_PD6A_GMAC_GRX1                         (_UL_(1) << 6)

#define PIN_PD11A_GMAC_GRX2                        _L_(107)     /**< GMAC signal: GRX2 on PD11 mux A*/
#define MUX_PD11A_GMAC_GRX2                        _L_(0)       /**< GMAC signal line function value: GRX2 */
#define PIO_PD11A_GMAC_GRX2                        (_UL_(1) << 11)

#define PIN_PD12A_GMAC_GRX3                        _L_(108)     /**< GMAC signal: GRX3 on PD12 mux A*/
#define MUX_PD12A_GMAC_GRX3                        _L_(0)       /**< GMAC signal line function value: GRX3 */
#define PIO_PD12A_GMAC_GRX3                        (_UL_(1) << 12)

#define PIN_PB1B_GMAC_GTSUCOMP                     _L_(33)      /**< GMAC signal: GTSUCOMP on PB1 mux B*/
#define MUX_PB1B_GMAC_GTSUCOMP                     _L_(1)       /**< GMAC signal line function value: GTSUCOMP */
#define PIO_PB1B_GMAC_GTSUCOMP                     (_UL_(1) << 1)

#define PIN_PB12B_GMAC_GTSUCOMP                    _L_(44)      /**< GMAC signal: GTSUCOMP on PB12 mux B*/
#define MUX_PB12B_GMAC_GTSUCOMP                    _L_(1)       /**< GMAC signal line function value: GTSUCOMP */
#define PIO_PB12B_GMAC_GTSUCOMP                    (_UL_(1) << 12)

#define PIN_PD11C_GMAC_GTSUCOMP                    _L_(107)     /**< GMAC signal: GTSUCOMP on PD11 mux C*/
#define MUX_PD11C_GMAC_GTSUCOMP                    _L_(2)       /**< GMAC signal line function value: GTSUCOMP */
#define PIO_PD11C_GMAC_GTSUCOMP                    (_UL_(1) << 11)

#define PIN_PD20C_GMAC_GTSUCOMP                    _L_(116)     /**< GMAC signal: GTSUCOMP on PD20 mux C*/
#define MUX_PD20C_GMAC_GTSUCOMP                    _L_(2)       /**< GMAC signal line function value: GTSUCOMP */
#define PIO_PD20C_GMAC_GTSUCOMP                    (_UL_(1) << 20)

#define PIN_PD0A_GMAC_GTXCK                        _L_(96)      /**< GMAC signal: GTXCK on PD0 mux A*/
#define MUX_PD0A_GMAC_GTXCK                        _L_(0)       /**< GMAC signal line function value: GTXCK */
#define PIO_PD0A_GMAC_GTXCK                        (_UL_(1) << 0)

#define PIN_PD1A_GMAC_GTXEN                        _L_(97)      /**< GMAC signal: GTXEN on PD1 mux A*/
#define MUX_PD1A_GMAC_GTXEN                        _L_(0)       /**< GMAC signal line function value: GTXEN */
#define PIO_PD1A_GMAC_GTXEN                        (_UL_(1) << 1)

#define PIN_PD17A_GMAC_GTXER                       _L_(113)     /**< GMAC signal: GTXER on PD17 mux A*/
#define MUX_PD17A_GMAC_GTXER                       _L_(0)       /**< GMAC signal line function value: GTXER */
#define PIO_PD17A_GMAC_GTXER                       (_UL_(1) << 17)

#define PIN_PD2A_GMAC_GTX0                         _L_(98)      /**< GMAC signal: GTX0 on PD2 mux A*/
#define MUX_PD2A_GMAC_GTX0                         _L_(0)       /**< GMAC signal line function value: GTX0 */
#define PIO_PD2A_GMAC_GTX0                         (_UL_(1) << 2)

#define PIN_PD3A_GMAC_GTX1                         _L_(99)      /**< GMAC signal: GTX1 on PD3 mux A*/
#define MUX_PD3A_GMAC_GTX1                         _L_(0)       /**< GMAC signal line function value: GTX1 */
#define PIO_PD3A_GMAC_GTX1                         (_UL_(1) << 3)

#define PIN_PD15A_GMAC_GTX2                        _L_(111)     /**< GMAC signal: GTX2 on PD15 mux A*/
#define MUX_PD15A_GMAC_GTX2                        _L_(0)       /**< GMAC signal line function value: GTX2 */
#define PIO_PD15A_GMAC_GTX2                        (_UL_(1) << 15)

#define PIN_PD16A_GMAC_GTX3                        _L_(112)     /**< GMAC signal: GTX3 on PD16 mux A*/
#define MUX_PD16A_GMAC_GTX3                        _L_(0)       /**< GMAC signal line function value: GTX3 */
#define PIO_PD16A_GMAC_GTX3                        (_UL_(1) << 16)

/* ========== PIO definition for HSMCI peripheral ========== */
#define PIN_PA28C_HSMCI_MCCDA                      _L_(28)      /**< HSMCI signal: MCCDA on PA28 mux C*/
#define MUX_PA28C_HSMCI_MCCDA                      _L_(2)       /**< HSMCI signal line function value: MCCDA */
#define PIO_PA28C_HSMCI_MCCDA                      (_UL_(1) << 28)

#define PIN_PA25D_HSMCI_MCCK                       _L_(25)      /**< HSMCI signal: MCCK on PA25 mux D*/
#define MUX_PA25D_HSMCI_MCCK                       _L_(3)       /**< HSMCI signal line function value: MCCK */
#define PIO_PA25D_HSMCI_MCCK                       (_UL_(1) << 25)

#define PIN_PA30C_HSMCI_MCDA0                      _L_(30)      /**< HSMCI signal: MCDA0 on PA30 mux C*/
#define MUX_PA30C_HSMCI_MCDA0                      _L_(2)       /**< HSMCI signal line function value: MCDA0 */
#define PIO_PA30C_HSMCI_MCDA0                      (_UL_(1) << 30)

#define PIN_PA31C_HSMCI_MCDA1                      _L_(31)      /**< HSMCI signal: MCDA1 on PA31 mux C*/
#define MUX_PA31C_HSMCI_MCDA1                      _L_(2)       /**< HSMCI signal line function value: MCDA1 */
#define PIO_PA31C_HSMCI_MCDA1                      (_UL_(1) << 31)

#define PIN_PA26C_HSMCI_MCDA2                      _L_(26)      /**< HSMCI signal: MCDA2 on PA26 mux C*/
#define MUX_PA26C_HSMCI_MCDA2                      _L_(2)       /**< HSMCI signal line function value: MCDA2 */
#define PIO_PA26C_HSMCI_MCDA2                      (_UL_(1) << 26)

#define PIN_PA27C_HSMCI_MCDA3                      _L_(27)      /**< HSMCI signal: MCDA3 on PA27 mux C*/
#define MUX_PA27C_HSMCI_MCDA3                      _L_(2)       /**< HSMCI signal line function value: MCDA3 */
#define PIO_PA27C_HSMCI_MCDA3                      (_UL_(1) << 27)

/* ========== PIO definition for ISI peripheral ========== */
#define PIN_PD22D_ISI_D0                           _L_(118)     /**< ISI signal: D0 on PD22 mux D*/
#define MUX_PD22D_ISI_D0                           _L_(3)       /**< ISI signal line function value: D0 */
#define PIO_PD22D_ISI_D0                           (_UL_(1) << 22)

#define PIN_PD21D_ISI_D1                           _L_(117)     /**< ISI signal: D1 on PD21 mux D*/
#define MUX_PD21D_ISI_D1                           _L_(3)       /**< ISI signal line function value: D1 */
#define PIO_PD21D_ISI_D1                           (_UL_(1) << 21)

#define PIN_PB3D_ISI_D2                            _L_(35)      /**< ISI signal: D2 on PB3 mux D*/
#define MUX_PB3D_ISI_D2                            _L_(3)       /**< ISI signal line function value: D2 */
#define PIO_PB3D_ISI_D2                            (_UL_(1) << 3)

#define PIN_PA9B_ISI_D3                            _L_(9)       /**< ISI signal: D3 on PA9 mux B*/
#define MUX_PA9B_ISI_D3                            _L_(1)       /**< ISI signal line function value: D3 */
#define PIO_PA9B_ISI_D3                            (_UL_(1) << 9)

#define PIN_PA5B_ISI_D4                            _L_(5)       /**< ISI signal: D4 on PA5 mux B*/
#define MUX_PA5B_ISI_D4                            _L_(1)       /**< ISI signal line function value: D4 */
#define PIO_PA5B_ISI_D4                            (_UL_(1) << 5)

#define PIN_PD11D_ISI_D5                           _L_(107)     /**< ISI signal: D5 on PD11 mux D*/
#define MUX_PD11D_ISI_D5                           _L_(3)       /**< ISI signal line function value: D5 */
#define PIO_PD11D_ISI_D5                           (_UL_(1) << 11)

#define PIN_PD12D_ISI_D6                           _L_(108)     /**< ISI signal: D6 on PD12 mux D*/
#define MUX_PD12D_ISI_D6                           _L_(3)       /**< ISI signal line function value: D6 */
#define PIO_PD12D_ISI_D6                           (_UL_(1) << 12)

#define PIN_PA27D_ISI_D7                           _L_(27)      /**< ISI signal: D7 on PA27 mux D*/
#define MUX_PA27D_ISI_D7                           _L_(3)       /**< ISI signal line function value: D7 */
#define PIO_PA27D_ISI_D7                           (_UL_(1) << 27)

#define PIN_PD27D_ISI_D8                           _L_(123)     /**< ISI signal: D8 on PD27 mux D*/
#define MUX_PD27D_ISI_D8                           _L_(3)       /**< ISI signal line function value: D8 */
#define PIO_PD27D_ISI_D8                           (_UL_(1) << 27)

#define PIN_PD28D_ISI_D9                           _L_(124)     /**< ISI signal: D9 on PD28 mux D*/
#define MUX_PD28D_ISI_D9                           _L_(3)       /**< ISI signal line function value: D9 */
#define PIO_PD28D_ISI_D9                           (_UL_(1) << 28)

#define PIN_PD30D_ISI_D10                          _L_(126)     /**< ISI signal: D10 on PD30 mux D*/
#define MUX_PD30D_ISI_D10                          _L_(3)       /**< ISI signal line function value: D10 */
#define PIO_PD30D_ISI_D10                          (_UL_(1) << 30)

#define PIN_PD31D_ISI_D11                          _L_(127)     /**< ISI signal: D11 on PD31 mux D*/
#define MUX_PD31D_ISI_D11                          _L_(3)       /**< ISI signal line function value: D11 */
#define PIO_PD31D_ISI_D11                          (_UL_(1) << 31)

#define PIN_PD24D_ISI_HSYNC                        _L_(120)     /**< ISI signal: HSYNC on PD24 mux D*/
#define MUX_PD24D_ISI_HSYNC                        _L_(3)       /**< ISI signal line function value: HSYNC */
#define PIO_PD24D_ISI_HSYNC                        (_UL_(1) << 24)

#define PIN_PA24D_ISI_PCK                          _L_(24)      /**< ISI signal: PCK on PA24 mux D*/
#define MUX_PA24D_ISI_PCK                          _L_(3)       /**< ISI signal line function value: PCK */
#define PIO_PA24D_ISI_PCK                          (_UL_(1) << 24)

#define PIN_PD25D_ISI_VSYNC                        _L_(121)     /**< ISI signal: VSYNC on PD25 mux D*/
#define MUX_PD25D_ISI_VSYNC                        _L_(3)       /**< ISI signal line function value: VSYNC */
#define PIO_PD25D_ISI_VSYNC                        (_UL_(1) << 25)

/* ========== PIO definition for MCAN0 peripheral ========== */
#define PIN_PB3A_MCAN0_CANRX0                      _L_(35)      /**< MCAN0 signal: CANRX0 on PB3 mux A*/
#define MUX_PB3A_MCAN0_CANRX0                      _L_(0)       /**< MCAN0 signal line function value: CANRX0 */
#define PIO_PB3A_MCAN0_CANRX0                      (_UL_(1) << 3)

#define PIN_PB2A_MCAN0_CANTX0                      _L_(34)      /**< MCAN0 signal: CANTX0 on PB2 mux A*/
#define MUX_PB2A_MCAN0_CANTX0                      _L_(0)       /**< MCAN0 signal line function value: CANTX0 */
#define PIO_PB2A_MCAN0_CANTX0                      (_UL_(1) << 2)

/* ========== PIO definition for MCAN1 peripheral ========== */
#define PIN_PC12C_MCAN1_CANRX1                     _L_(76)      /**< MCAN1 signal: CANRX1 on PC12 mux C*/
#define MUX_PC12C_MCAN1_CANRX1                     _L_(2)       /**< MCAN1 signal line function value: CANRX1 */
#define PIO_PC12C_MCAN1_CANRX1                     (_UL_(1) << 12)

#define PIN_PD28B_MCAN1_CANRX1                     _L_(124)     /**< MCAN1 signal: CANRX1 on PD28 mux B*/
#define MUX_PD28B_MCAN1_CANRX1                     _L_(1)       /**< MCAN1 signal line function value: CANRX1 */
#define PIO_PD28B_MCAN1_CANRX1                     (_UL_(1) << 28)

#define PIN_PC14C_MCAN1_CANTX1                     _L_(78)      /**< MCAN1 signal: CANTX1 on PC14 mux C*/
#define MUX_PC14C_MCAN1_CANTX1                     _L_(2)       /**< MCAN1 signal line function value: CANTX1 */
#define PIO_PC14C_MCAN1_CANTX1                     (_UL_(1) << 14)

#define PIN_PD12B_MCAN1_CANTX1                     _L_(108)     /**< MCAN1 signal: CANTX1 on PD12 mux B*/
#define MUX_PD12B_MCAN1_CANTX1                     _L_(1)       /**< MCAN1 signal line function value: CANTX1 */
#define PIO_PD12B_MCAN1_CANTX1                     (_UL_(1) << 12)

/* ========== PIO definition for PMC peripheral ========== */
#define PIN_PA6B_PMC_PCK0                          _L_(6)       /**< PMC signal: PCK0 on PA6 mux B*/
#define MUX_PA6B_PMC_PCK0                          _L_(1)       /**< PMC signal line function value: PCK0 */
#define PIO_PA6B_PMC_PCK0                          (_UL_(1) << 6)

#define PIN_PB12D_PMC_PCK0                         _L_(44)      /**< PMC signal: PCK0 on PB12 mux D*/
#define MUX_PB12D_PMC_PCK0                         _L_(3)       /**< PMC signal line function value: PCK0 */
#define PIO_PB12D_PMC_PCK0                         (_UL_(1) << 12)

#define PIN_PB13B_PMC_PCK0                         _L_(45)      /**< PMC signal: PCK0 on PB13 mux B*/
#define MUX_PB13B_PMC_PCK0                         _L_(1)       /**< PMC signal line function value: PCK0 */
#define PIO_PB13B_PMC_PCK0                         (_UL_(1) << 13)

#define PIN_PA17B_PMC_PCK1                         _L_(17)      /**< PMC signal: PCK1 on PA17 mux B*/
#define MUX_PA17B_PMC_PCK1                         _L_(1)       /**< PMC signal line function value: PCK1 */
#define PIO_PA17B_PMC_PCK1                         (_UL_(1) << 17)

#define PIN_PA21B_PMC_PCK1                         _L_(21)      /**< PMC signal: PCK1 on PA21 mux B*/
#define MUX_PA21B_PMC_PCK1                         _L_(1)       /**< PMC signal line function value: PCK1 */
#define PIO_PA21B_PMC_PCK1                         (_UL_(1) << 21)

#define PIN_PA3C_PMC_PCK2                          _L_(3)       /**< PMC signal: PCK2 on PA3 mux C*/
#define MUX_PA3C_PMC_PCK2                          _L_(2)       /**< PMC signal line function value: PCK2 */
#define PIO_PA3C_PMC_PCK2                          (_UL_(1) << 3)

#define PIN_PA18B_PMC_PCK2                         _L_(18)      /**< PMC signal: PCK2 on PA18 mux B*/
#define MUX_PA18B_PMC_PCK2                         _L_(1)       /**< PMC signal line function value: PCK2 */
#define PIO_PA18B_PMC_PCK2                         (_UL_(1) << 18)

#define PIN_PA31B_PMC_PCK2                         _L_(31)      /**< PMC signal: PCK2 on PA31 mux B*/
#define MUX_PA31B_PMC_PCK2                         _L_(1)       /**< PMC signal line function value: PCK2 */
#define PIO_PA31B_PMC_PCK2                         (_UL_(1) << 31)

#define PIN_PB3B_PMC_PCK2                          _L_(35)      /**< PMC signal: PCK2 on PB3 mux B*/
#define MUX_PB3B_PMC_PCK2                          _L_(1)       /**< PMC signal line function value: PCK2 */
#define PIO_PB3B_PMC_PCK2                          (_UL_(1) << 3)

#define PIN_PD31C_PMC_PCK2                         _L_(127)     /**< PMC signal: PCK2 on PD31 mux C*/
#define MUX_PD31C_PMC_PCK2                         _L_(2)       /**< PMC signal line function value: PCK2 */
#define PIO_PD31C_PMC_PCK2                         (_UL_(1) << 31)

/* ========== PIO definition for PWM0 peripheral ========== */
#define PIN_PA10B_PWM0_PWMEXTRG0                   _L_(10)      /**< PWM0 signal: PWMEXTRG0 on PA10 mux B*/
#define MUX_PA10B_PWM0_PWMEXTRG0                   _L_(1)       /**< PWM0 signal line function value: PWMEXTRG0 */
#define PIO_PA10B_PWM0_PWMEXTRG0                   (_UL_(1) << 10)

#define PIN_PA22B_PWM0_PWMEXTRG1                   _L_(22)      /**< PWM0 signal: PWMEXTRG1 on PA22 mux B*/
#define MUX_PA22B_PWM0_PWMEXTRG1                   _L_(1)       /**< PWM0 signal line function value: PWMEXTRG1 */
#define PIO_PA22B_PWM0_PWMEXTRG1                   (_UL_(1) << 22)

#define PIN_PA9C_PWM0_PWMFI0                       _L_(9)       /**< PWM0 signal: PWMFI0 on PA9 mux C*/
#define MUX_PA9C_PWM0_PWMFI0                       _L_(2)       /**< PWM0 signal line function value: PWMFI0 */
#define PIO_PA9C_PWM0_PWMFI0                       (_UL_(1) << 9)

#define PIN_PD8B_PWM0_PWMFI1                       _L_(104)     /**< PWM0 signal: PWMFI1 on PD8 mux B*/
#define MUX_PD8B_PWM0_PWMFI1                       _L_(1)       /**< PWM0 signal line function value: PWMFI1 */
#define PIO_PD8B_PWM0_PWMFI1                       (_UL_(1) << 8)

#define PIN_PD9B_PWM0_PWMFI2                       _L_(105)     /**< PWM0 signal: PWMFI2 on PD9 mux B*/
#define MUX_PD9B_PWM0_PWMFI2                       _L_(1)       /**< PWM0 signal line function value: PWMFI2 */
#define PIO_PD9B_PWM0_PWMFI2                       (_UL_(1) << 9)

#define PIN_PA0A_PWM0_PWMH0                        _L_(0)       /**< PWM0 signal: PWMH0 on PA0 mux A*/
#define MUX_PA0A_PWM0_PWMH0                        _L_(0)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PA0A_PWM0_PWMH0                        (_UL_(1) << 0)

#define PIN_PA11B_PWM0_PWMH0                       _L_(11)      /**< PWM0 signal: PWMH0 on PA11 mux B*/
#define MUX_PA11B_PWM0_PWMH0                       _L_(1)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PA11B_PWM0_PWMH0                       (_UL_(1) << 11)

#define PIN_PA23B_PWM0_PWMH0                       _L_(23)      /**< PWM0 signal: PWMH0 on PA23 mux B*/
#define MUX_PA23B_PWM0_PWMH0                       _L_(1)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PA23B_PWM0_PWMH0                       (_UL_(1) << 23)

#define PIN_PB0A_PWM0_PWMH0                        _L_(32)      /**< PWM0 signal: PWMH0 on PB0 mux A*/
#define MUX_PB0A_PWM0_PWMH0                        _L_(0)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PB0A_PWM0_PWMH0                        (_UL_(1) << 0)

#define PIN_PD11B_PWM0_PWMH0                       _L_(107)     /**< PWM0 signal: PWMH0 on PD11 mux B*/
#define MUX_PD11B_PWM0_PWMH0                       _L_(1)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PD11B_PWM0_PWMH0                       (_UL_(1) << 11)

#define PIN_PD20A_PWM0_PWMH0                       _L_(116)     /**< PWM0 signal: PWMH0 on PD20 mux A*/
#define MUX_PD20A_PWM0_PWMH0                       _L_(0)       /**< PWM0 signal line function value: PWMH0 */
#define PIO_PD20A_PWM0_PWMH0                       (_UL_(1) << 20)

#define PIN_PA2A_PWM0_PWMH1                        _L_(2)       /**< PWM0 signal: PWMH1 on PA2 mux A*/
#define MUX_PA2A_PWM0_PWMH1                        _L_(0)       /**< PWM0 signal line function value: PWMH1 */
#define PIO_PA2A_PWM0_PWMH1                        (_UL_(1) << 2)

#define PIN_PA12B_PWM0_PWMH1                       _L_(12)      /**< PWM0 signal: PWMH1 on PA12 mux B*/
#define MUX_PA12B_PWM0_PWMH1                       _L_(1)       /**< PWM0 signal line function value: PWMH1 */
#define PIO_PA12B_PWM0_PWMH1                       (_UL_(1) << 12)

#define PIN_PA24B_PWM0_PWMH1                       _L_(24)      /**< PWM0 signal: PWMH1 on PA24 mux B*/
#define MUX_PA24B_PWM0_PWMH1                       _L_(1)       /**< PWM0 signal line function value: PWMH1 */
#define PIO_PA24B_PWM0_PWMH1                       (_UL_(1) << 24)

#define PIN_PB1A_PWM0_PWMH1                        _L_(33)      /**< PWM0 signal: PWMH1 on PB1 mux A*/
#define MUX_PB1A_PWM0_PWMH1                        _L_(0)       /**< PWM0 signal line function value: PWMH1 */
#define PIO_PB1A_PWM0_PWMH1                        (_UL_(1) << 1)

#define PIN_PD21A_PWM0_PWMH1                       _L_(117)     /**< PWM0 signal: PWMH1 on PD21 mux A*/
#define MUX_PD21A_PWM0_PWMH1                       _L_(0)       /**< PWM0 signal line function value: PWMH1 */
#define PIO_PD21A_PWM0_PWMH1                       (_UL_(1) << 21)

#define PIN_PA13B_PWM0_PWMH2                       _L_(13)      /**< PWM0 signal: PWMH2 on PA13 mux B*/
#define MUX_PA13B_PWM0_PWMH2                       _L_(1)       /**< PWM0 signal line function value: PWMH2 */
#define PIO_PA13B_PWM0_PWMH2                       (_UL_(1) << 13)

#define PIN_PA25B_PWM0_PWMH2                       _L_(25)      /**< PWM0 signal: PWMH2 on PA25 mux B*/
#define MUX_PA25B_PWM0_PWMH2                       _L_(1)       /**< PWM0 signal line function value: PWMH2 */
#define PIO_PA25B_PWM0_PWMH2                       (_UL_(1) << 25)

#define PIN_PB4B_PWM0_PWMH2                        _L_(36)      /**< PWM0 signal: PWMH2 on PB4 mux B*/
#define MUX_PB4B_PWM0_PWMH2                        _L_(1)       /**< PWM0 signal line function value: PWMH2 */
#define PIO_PB4B_PWM0_PWMH2                        (_UL_(1) << 4)

#define PIN_PC19B_PWM0_PWMH2                       _L_(83)      /**< PWM0 signal: PWMH2 on PC19 mux B*/
#define MUX_PC19B_PWM0_PWMH2                       _L_(1)       /**< PWM0 signal line function value: PWMH2 */
#define PIO_PC19B_PWM0_PWMH2                       (_UL_(1) << 19)

#define PIN_PD22A_PWM0_PWMH2                       _L_(118)     /**< PWM0 signal: PWMH2 on PD22 mux A*/
#define MUX_PD22A_PWM0_PWMH2                       _L_(0)       /**< PWM0 signal line function value: PWMH2 */
#define PIO_PD22A_PWM0_PWMH2                       (_UL_(1) << 22)

#define PIN_PA7B_PWM0_PWMH3                        _L_(7)       /**< PWM0 signal: PWMH3 on PA7 mux B*/
#define MUX_PA7B_PWM0_PWMH3                        _L_(1)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PA7B_PWM0_PWMH3                        (_UL_(1) << 7)

#define PIN_PA14B_PWM0_PWMH3                       _L_(14)      /**< PWM0 signal: PWMH3 on PA14 mux B*/
#define MUX_PA14B_PWM0_PWMH3                       _L_(1)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PA14B_PWM0_PWMH3                       (_UL_(1) << 14)

#define PIN_PA17C_PWM0_PWMH3                       _L_(17)      /**< PWM0 signal: PWMH3 on PA17 mux C*/
#define MUX_PA17C_PWM0_PWMH3                       _L_(2)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PA17C_PWM0_PWMH3                       (_UL_(1) << 17)

#define PIN_PC13B_PWM0_PWMH3                       _L_(77)      /**< PWM0 signal: PWMH3 on PC13 mux B*/
#define MUX_PC13B_PWM0_PWMH3                       _L_(1)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PC13B_PWM0_PWMH3                       (_UL_(1) << 13)

#define PIN_PC21B_PWM0_PWMH3                       _L_(85)      /**< PWM0 signal: PWMH3 on PC21 mux B*/
#define MUX_PC21B_PWM0_PWMH3                       _L_(1)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PC21B_PWM0_PWMH3                       (_UL_(1) << 21)

#define PIN_PD23A_PWM0_PWMH3                       _L_(119)     /**< PWM0 signal: PWMH3 on PD23 mux A*/
#define MUX_PD23A_PWM0_PWMH3                       _L_(0)       /**< PWM0 signal line function value: PWMH3 */
#define PIO_PD23A_PWM0_PWMH3                       (_UL_(1) << 23)

#define PIN_PA1A_PWM0_PWML0                        _L_(1)       /**< PWM0 signal: PWML0 on PA1 mux A*/
#define MUX_PA1A_PWM0_PWML0                        _L_(0)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PA1A_PWM0_PWML0                        (_UL_(1) << 1)

#define PIN_PA19B_PWM0_PWML0                       _L_(19)      /**< PWM0 signal: PWML0 on PA19 mux B*/
#define MUX_PA19B_PWM0_PWML0                       _L_(1)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PA19B_PWM0_PWML0                       (_UL_(1) << 19)

#define PIN_PB5B_PWM0_PWML0                        _L_(37)      /**< PWM0 signal: PWML0 on PB5 mux B*/
#define MUX_PB5B_PWM0_PWML0                        _L_(1)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PB5B_PWM0_PWML0                        (_UL_(1) << 5)

#define PIN_PC0B_PWM0_PWML0                        _L_(64)      /**< PWM0 signal: PWML0 on PC0 mux B*/
#define MUX_PC0B_PWM0_PWML0                        _L_(1)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PC0B_PWM0_PWML0                        (_UL_(1) << 0)

#define PIN_PD10B_PWM0_PWML0                       _L_(106)     /**< PWM0 signal: PWML0 on PD10 mux B*/
#define MUX_PD10B_PWM0_PWML0                       _L_(1)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PD10B_PWM0_PWML0                       (_UL_(1) << 10)

#define PIN_PD24A_PWM0_PWML0                       _L_(120)     /**< PWM0 signal: PWML0 on PD24 mux A*/
#define MUX_PD24A_PWM0_PWML0                       _L_(0)       /**< PWM0 signal line function value: PWML0 */
#define PIO_PD24A_PWM0_PWML0                       (_UL_(1) << 24)

#define PIN_PA20B_PWM0_PWML1                       _L_(20)      /**< PWM0 signal: PWML1 on PA20 mux B*/
#define MUX_PA20B_PWM0_PWML1                       _L_(1)       /**< PWM0 signal line function value: PWML1 */
#define PIO_PA20B_PWM0_PWML1                       (_UL_(1) << 20)

#define PIN_PB12A_PWM0_PWML1                       _L_(44)      /**< PWM0 signal: PWML1 on PB12 mux A*/
#define MUX_PB12A_PWM0_PWML1                       _L_(0)       /**< PWM0 signal line function value: PWML1 */
#define PIO_PB12A_PWM0_PWML1                       (_UL_(1) << 12)

#define PIN_PC1B_PWM0_PWML1                        _L_(65)      /**< PWM0 signal: PWML1 on PC1 mux B*/
#define MUX_PC1B_PWM0_PWML1                        _L_(1)       /**< PWM0 signal line function value: PWML1 */
#define PIO_PC1B_PWM0_PWML1                        (_UL_(1) << 1)

#define PIN_PC18B_PWM0_PWML1                       _L_(82)      /**< PWM0 signal: PWML1 on PC18 mux B*/
#define MUX_PC18B_PWM0_PWML1                       _L_(1)       /**< PWM0 signal line function value: PWML1 */
#define PIO_PC18B_PWM0_PWML1                       (_UL_(1) << 18)

#define PIN_PD25A_PWM0_PWML1                       _L_(121)     /**< PWM0 signal: PWML1 on PD25 mux A*/
#define MUX_PD25A_PWM0_PWML1                       _L_(0)       /**< PWM0 signal line function value: PWML1 */
#define PIO_PD25A_PWM0_PWML1                       (_UL_(1) << 25)

#define PIN_PA16C_PWM0_PWML2                       _L_(16)      /**< PWM0 signal: PWML2 on PA16 mux C*/
#define MUX_PA16C_PWM0_PWML2                       _L_(2)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PA16C_PWM0_PWML2                       (_UL_(1) << 16)

#define PIN_PA30A_PWM0_PWML2                       _L_(30)      /**< PWM0 signal: PWML2 on PA30 mux A*/
#define MUX_PA30A_PWM0_PWML2                       _L_(0)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PA30A_PWM0_PWML2                       (_UL_(1) << 30)

#define PIN_PB13A_PWM0_PWML2                       _L_(45)      /**< PWM0 signal: PWML2 on PB13 mux A*/
#define MUX_PB13A_PWM0_PWML2                       _L_(0)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PB13A_PWM0_PWML2                       (_UL_(1) << 13)

#define PIN_PC2B_PWM0_PWML2                        _L_(66)      /**< PWM0 signal: PWML2 on PC2 mux B*/
#define MUX_PC2B_PWM0_PWML2                        _L_(1)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PC2B_PWM0_PWML2                        (_UL_(1) << 2)

#define PIN_PC20B_PWM0_PWML2                       _L_(84)      /**< PWM0 signal: PWML2 on PC20 mux B*/
#define MUX_PC20B_PWM0_PWML2                       _L_(1)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PC20B_PWM0_PWML2                       (_UL_(1) << 20)

#define PIN_PD26A_PWM0_PWML2                       _L_(122)     /**< PWM0 signal: PWML2 on PD26 mux A*/
#define MUX_PD26A_PWM0_PWML2                       _L_(0)       /**< PWM0 signal line function value: PWML2 */
#define PIO_PD26A_PWM0_PWML2                       (_UL_(1) << 26)

#define PIN_PA15C_PWM0_PWML3                       _L_(15)      /**< PWM0 signal: PWML3 on PA15 mux C*/
#define MUX_PA15C_PWM0_PWML3                       _L_(2)       /**< PWM0 signal line function value: PWML3 */
#define PIO_PA15C_PWM0_PWML3                       (_UL_(1) << 15)

#define PIN_PC3B_PWM0_PWML3                        _L_(67)      /**< PWM0 signal: PWML3 on PC3 mux B*/
#define MUX_PC3B_PWM0_PWML3                        _L_(1)       /**< PWM0 signal line function value: PWML3 */
#define PIO_PC3B_PWM0_PWML3                        (_UL_(1) << 3)

#define PIN_PC15B_PWM0_PWML3                       _L_(79)      /**< PWM0 signal: PWML3 on PC15 mux B*/
#define MUX_PC15B_PWM0_PWML3                       _L_(1)       /**< PWM0 signal line function value: PWML3 */
#define PIO_PC15B_PWM0_PWML3                       (_UL_(1) << 15)

#define PIN_PC22B_PWM0_PWML3                       _L_(86)      /**< PWM0 signal: PWML3 on PC22 mux B*/
#define MUX_PC22B_PWM0_PWML3                       _L_(1)       /**< PWM0 signal line function value: PWML3 */
#define PIO_PC22B_PWM0_PWML3                       (_UL_(1) << 22)

#define PIN_PD27A_PWM0_PWML3                       _L_(123)     /**< PWM0 signal: PWML3 on PD27 mux A*/
#define MUX_PD27A_PWM0_PWML3                       _L_(0)       /**< PWM0 signal line function value: PWML3 */
#define PIO_PD27A_PWM0_PWML3                       (_UL_(1) << 27)

/* ========== PIO definition for PWM1 peripheral ========== */
#define PIN_PA30B_PWM1_PWMEXTRG0                   _L_(30)      /**< PWM1 signal: PWMEXTRG0 on PA30 mux B*/
#define MUX_PA30B_PWM1_PWMEXTRG0                   _L_(1)       /**< PWM1 signal line function value: PWMEXTRG0 */
#define PIO_PA30B_PWM1_PWMEXTRG0                   (_UL_(1) << 30)

#define PIN_PA18A_PWM1_PWMEXTRG1                   _L_(18)      /**< PWM1 signal: PWMEXTRG1 on PA18 mux A*/
#define MUX_PA18A_PWM1_PWMEXTRG1                   _L_(0)       /**< PWM1 signal line function value: PWMEXTRG1 */
#define PIO_PA18A_PWM1_PWMEXTRG1                   (_UL_(1) << 18)

#define PIN_PA21C_PWM1_PWMFI0                      _L_(21)      /**< PWM1 signal: PWMFI0 on PA21 mux C*/
#define MUX_PA21C_PWM1_PWMFI0                      _L_(2)       /**< PWM1 signal line function value: PWMFI0 */
#define PIO_PA21C_PWM1_PWMFI0                      (_UL_(1) << 21)

#define PIN_PA26D_PWM1_PWMFI1                      _L_(26)      /**< PWM1 signal: PWMFI1 on PA26 mux D*/
#define MUX_PA26D_PWM1_PWMFI1                      _L_(3)       /**< PWM1 signal line function value: PWMFI1 */
#define PIO_PA26D_PWM1_PWMFI1                      (_UL_(1) << 26)

#define PIN_PA28D_PWM1_PWMFI2                      _L_(28)      /**< PWM1 signal: PWMFI2 on PA28 mux D*/
#define MUX_PA28D_PWM1_PWMFI2                      _L_(3)       /**< PWM1 signal line function value: PWMFI2 */
#define PIO_PA28D_PWM1_PWMFI2                      (_UL_(1) << 28)

#define PIN_PA12C_PWM1_PWMH0                       _L_(12)      /**< PWM1 signal: PWMH0 on PA12 mux C*/
#define MUX_PA12C_PWM1_PWMH0                       _L_(2)       /**< PWM1 signal line function value: PWMH0 */
#define PIO_PA12C_PWM1_PWMH0                       (_UL_(1) << 12)

#define PIN_PD1B_PWM1_PWMH0                        _L_(97)      /**< PWM1 signal: PWMH0 on PD1 mux B*/
#define MUX_PD1B_PWM1_PWMH0                        _L_(1)       /**< PWM1 signal line function value: PWMH0 */
#define PIO_PD1B_PWM1_PWMH0                        (_UL_(1) << 1)

#define PIN_PA14C_PWM1_PWMH1                       _L_(14)      /**< PWM1 signal: PWMH1 on PA14 mux C*/
#define MUX_PA14C_PWM1_PWMH1                       _L_(2)       /**< PWM1 signal line function value: PWMH1 */
#define PIO_PA14C_PWM1_PWMH1                       (_UL_(1) << 14)

#define PIN_PD3B_PWM1_PWMH1                        _L_(99)      /**< PWM1 signal: PWMH1 on PD3 mux B*/
#define MUX_PD3B_PWM1_PWMH1                        _L_(1)       /**< PWM1 signal line function value: PWMH1 */
#define PIO_PD3B_PWM1_PWMH1                        (_UL_(1) << 3)

#define PIN_PA31D_PWM1_PWMH2                       _L_(31)      /**< PWM1 signal: PWMH2 on PA31 mux D*/
#define MUX_PA31D_PWM1_PWMH2                       _L_(3)       /**< PWM1 signal line function value: PWMH2 */
#define PIO_PA31D_PWM1_PWMH2                       (_UL_(1) << 31)

#define PIN_PD5B_PWM1_PWMH2                        _L_(101)     /**< PWM1 signal: PWMH2 on PD5 mux B*/
#define MUX_PD5B_PWM1_PWMH2                        _L_(1)       /**< PWM1 signal line function value: PWMH2 */
#define PIO_PD5B_PWM1_PWMH2                        (_UL_(1) << 5)

#define PIN_PA8A_PWM1_PWMH3                        _L_(8)       /**< PWM1 signal: PWMH3 on PA8 mux A*/
#define MUX_PA8A_PWM1_PWMH3                        _L_(0)       /**< PWM1 signal line function value: PWMH3 */
#define PIO_PA8A_PWM1_PWMH3                        (_UL_(1) << 8)

#define PIN_PD7B_PWM1_PWMH3                        _L_(103)     /**< PWM1 signal: PWMH3 on PD7 mux B*/
#define MUX_PD7B_PWM1_PWMH3                        _L_(1)       /**< PWM1 signal line function value: PWMH3 */
#define PIO_PD7B_PWM1_PWMH3                        (_UL_(1) << 7)

#define PIN_PA11C_PWM1_PWML0                       _L_(11)      /**< PWM1 signal: PWML0 on PA11 mux C*/
#define MUX_PA11C_PWM1_PWML0                       _L_(2)       /**< PWM1 signal line function value: PWML0 */
#define PIO_PA11C_PWM1_PWML0                       (_UL_(1) << 11)

#define PIN_PD0B_PWM1_PWML0                        _L_(96)      /**< PWM1 signal: PWML0 on PD0 mux B*/
#define MUX_PD0B_PWM1_PWML0                        _L_(1)       /**< PWM1 signal line function value: PWML0 */
#define PIO_PD0B_PWM1_PWML0                        (_UL_(1) << 0)

#define PIN_PA13C_PWM1_PWML1                       _L_(13)      /**< PWM1 signal: PWML1 on PA13 mux C*/
#define MUX_PA13C_PWM1_PWML1                       _L_(2)       /**< PWM1 signal line function value: PWML1 */
#define PIO_PA13C_PWM1_PWML1                       (_UL_(1) << 13)

#define PIN_PD2B_PWM1_PWML1                        _L_(98)      /**< PWM1 signal: PWML1 on PD2 mux B*/
#define MUX_PD2B_PWM1_PWML1                        _L_(1)       /**< PWM1 signal line function value: PWML1 */
#define PIO_PD2B_PWM1_PWML1                        (_UL_(1) << 2)

#define PIN_PA23D_PWM1_PWML2                       _L_(23)      /**< PWM1 signal: PWML2 on PA23 mux D*/
#define MUX_PA23D_PWM1_PWML2                       _L_(3)       /**< PWM1 signal line function value: PWML2 */
#define PIO_PA23D_PWM1_PWML2                       (_UL_(1) << 23)

#define PIN_PD4B_PWM1_PWML2                        _L_(100)     /**< PWM1 signal: PWML2 on PD4 mux B*/
#define MUX_PD4B_PWM1_PWML2                        _L_(1)       /**< PWM1 signal line function value: PWML2 */
#define PIO_PD4B_PWM1_PWML2                        (_UL_(1) << 4)

#define PIN_PA5A_PWM1_PWML3                        _L_(5)       /**< PWM1 signal: PWML3 on PA5 mux A*/
#define MUX_PA5A_PWM1_PWML3                        _L_(0)       /**< PWM1 signal line function value: PWML3 */
#define PIO_PA5A_PWM1_PWML3                        (_UL_(1) << 5)

#define PIN_PD6B_PWM1_PWML3                        _L_(102)     /**< PWM1 signal: PWML3 on PD6 mux B*/
#define MUX_PD6B_PWM1_PWML3                        _L_(1)       /**< PWM1 signal line function value: PWML3 */
#define PIO_PD6B_PWM1_PWML3                        (_UL_(1) << 6)

/* ========== PIO definition for QSPI peripheral ========== */
#define PIN_PA11A_QSPI_QCS                         _L_(11)      /**< QSPI signal: QCS on PA11 mux A*/
#define MUX_PA11A_QSPI_QCS                         _L_(0)       /**< QSPI signal line function value: QCS */
#define PIO_PA11A_QSPI_QCS                         (_UL_(1) << 11)

#define PIN_PA13A_QSPI_QIO0                        _L_(13)      /**< QSPI signal: QIO0 on PA13 mux A*/
#define MUX_PA13A_QSPI_QIO0                        _L_(0)       /**< QSPI signal line function value: QIO0 */
#define PIO_PA13A_QSPI_QIO0                        (_UL_(1) << 13)

#define PIN_PA12A_QSPI_QIO1                        _L_(12)      /**< QSPI signal: QIO1 on PA12 mux A*/
#define MUX_PA12A_QSPI_QIO1                        _L_(0)       /**< QSPI signal line function value: QIO1 */
#define PIO_PA12A_QSPI_QIO1                        (_UL_(1) << 12)

#define PIN_PA17A_QSPI_QIO2                        _L_(17)      /**< QSPI signal: QIO2 on PA17 mux A*/
#define MUX_PA17A_QSPI_QIO2                        _L_(0)       /**< QSPI signal line function value: QIO2 */
#define PIO_PA17A_QSPI_QIO2                        (_UL_(1) << 17)

#define PIN_PD31A_QSPI_QIO3                        _L_(127)     /**< QSPI signal: QIO3 on PD31 mux A*/
#define MUX_PD31A_QSPI_QIO3                        _L_(0)       /**< QSPI signal line function value: QIO3 */
#define PIO_PD31A_QSPI_QIO3                        (_UL_(1) << 31)

#define PIN_PA14A_QSPI_QSCK                        _L_(14)      /**< QSPI signal: QSCK on PA14 mux A*/
#define MUX_PA14A_QSPI_QSCK                        _L_(0)       /**< QSPI signal line function value: QSCK */
#define PIO_PA14A_QSPI_QSCK                        (_UL_(1) << 14)

/* ========== PIO definition for SDRAMC peripheral ========== */
#define PIN_PC18A_SDRAMC_A0                        _L_(82)      /**< SDRAMC signal: A0 on PC18 mux A*/
#define MUX_PC18A_SDRAMC_A0                        _L_(0)       /**< SDRAMC signal line function value: A0 */
#define PIO_PC18A_SDRAMC_A0                        (_UL_(1) << 18)

#define PIN_PC18A_SDRAMC_NBS0                      _L_(82)      /**< SDRAMC signal: NBS0 on PC18 mux A*/
#define MUX_PC18A_SDRAMC_NBS0                      _L_(0)       /**< SDRAMC signal line function value: NBS0 */
#define PIO_PC18A_SDRAMC_NBS0                      (_UL_(1) << 18)

#define PIN_PC19A_SDRAMC_A1                        _L_(83)      /**< SDRAMC signal: A1 on PC19 mux A*/
#define MUX_PC19A_SDRAMC_A1                        _L_(0)       /**< SDRAMC signal line function value: A1 */
#define PIO_PC19A_SDRAMC_A1                        (_UL_(1) << 19)

#define PIN_PC20A_SDRAMC_A2                        _L_(84)      /**< SDRAMC signal: A2 on PC20 mux A*/
#define MUX_PC20A_SDRAMC_A2                        _L_(0)       /**< SDRAMC signal line function value: A2 */
#define PIO_PC20A_SDRAMC_A2                        (_UL_(1) << 20)

#define PIN_PC21A_SDRAMC_A3                        _L_(85)      /**< SDRAMC signal: A3 on PC21 mux A*/
#define MUX_PC21A_SDRAMC_A3                        _L_(0)       /**< SDRAMC signal line function value: A3 */
#define PIO_PC21A_SDRAMC_A3                        (_UL_(1) << 21)

#define PIN_PC22A_SDRAMC_A4                        _L_(86)      /**< SDRAMC signal: A4 on PC22 mux A*/
#define MUX_PC22A_SDRAMC_A4                        _L_(0)       /**< SDRAMC signal line function value: A4 */
#define PIO_PC22A_SDRAMC_A4                        (_UL_(1) << 22)

#define PIN_PC23A_SDRAMC_A5                        _L_(87)      /**< SDRAMC signal: A5 on PC23 mux A*/
#define MUX_PC23A_SDRAMC_A5                        _L_(0)       /**< SDRAMC signal line function value: A5 */
#define PIO_PC23A_SDRAMC_A5                        (_UL_(1) << 23)

#define PIN_PC24A_SDRAMC_A6                        _L_(88)      /**< SDRAMC signal: A6 on PC24 mux A*/
#define MUX_PC24A_SDRAMC_A6                        _L_(0)       /**< SDRAMC signal line function value: A6 */
#define PIO_PC24A_SDRAMC_A6                        (_UL_(1) << 24)

#define PIN_PC25A_SDRAMC_A7                        _L_(89)      /**< SDRAMC signal: A7 on PC25 mux A*/
#define MUX_PC25A_SDRAMC_A7                        _L_(0)       /**< SDRAMC signal line function value: A7 */
#define PIO_PC25A_SDRAMC_A7                        (_UL_(1) << 25)

#define PIN_PC26A_SDRAMC_A8                        _L_(90)      /**< SDRAMC signal: A8 on PC26 mux A*/
#define MUX_PC26A_SDRAMC_A8                        _L_(0)       /**< SDRAMC signal line function value: A8 */
#define PIO_PC26A_SDRAMC_A8                        (_UL_(1) << 26)

#define PIN_PC27A_SDRAMC_A9                        _L_(91)      /**< SDRAMC signal: A9 on PC27 mux A*/
#define MUX_PC27A_SDRAMC_A9                        _L_(0)       /**< SDRAMC signal line function value: A9 */
#define PIO_PC27A_SDRAMC_A9                        (_UL_(1) << 27)

#define PIN_PC28A_SDRAMC_A10                       _L_(92)      /**< SDRAMC signal: A10 on PC28 mux A*/
#define MUX_PC28A_SDRAMC_A10                       _L_(0)       /**< SDRAMC signal line function value: A10 */
#define PIO_PC28A_SDRAMC_A10                       (_UL_(1) << 28)

#define PIN_PC29A_SDRAMC_A11                       _L_(93)      /**< SDRAMC signal: A11 on PC29 mux A*/
#define MUX_PC29A_SDRAMC_A11                       _L_(0)       /**< SDRAMC signal line function value: A11 */
#define PIO_PC29A_SDRAMC_A11                       (_UL_(1) << 29)

#define PIN_PC30A_SDRAMC_A12                       _L_(94)      /**< SDRAMC signal: A12 on PC30 mux A*/
#define MUX_PC30A_SDRAMC_A12                       _L_(0)       /**< SDRAMC signal line function value: A12 */
#define PIO_PC30A_SDRAMC_A12                       (_UL_(1) << 30)

#define PIN_PC31A_SDRAMC_A13                       _L_(95)      /**< SDRAMC signal: A13 on PC31 mux A*/
#define MUX_PC31A_SDRAMC_A13                       _L_(0)       /**< SDRAMC signal line function value: A13 */
#define PIO_PC31A_SDRAMC_A13                       (_UL_(1) << 31)

#define PIN_PA18C_SDRAMC_A14                       _L_(18)      /**< SDRAMC signal: A14 on PA18 mux C*/
#define MUX_PA18C_SDRAMC_A14                       _L_(2)       /**< SDRAMC signal line function value: A14 */
#define PIO_PA18C_SDRAMC_A14                       (_UL_(1) << 18)

#define PIN_PA19C_SDRAMC_A15                       _L_(19)      /**< SDRAMC signal: A15 on PA19 mux C*/
#define MUX_PA19C_SDRAMC_A15                       _L_(2)       /**< SDRAMC signal line function value: A15 */
#define PIO_PA19C_SDRAMC_A15                       (_UL_(1) << 19)

#define PIN_PA20C_SDRAMC_A16                       _L_(20)      /**< SDRAMC signal: A16 on PA20 mux C*/
#define MUX_PA20C_SDRAMC_A16                       _L_(2)       /**< SDRAMC signal line function value: A16 */
#define PIO_PA20C_SDRAMC_A16                       (_UL_(1) << 20)

#define PIN_PA20C_SDRAMC_BA0                       _L_(20)      /**< SDRAMC signal: BA0 on PA20 mux C*/
#define MUX_PA20C_SDRAMC_BA0                       _L_(2)       /**< SDRAMC signal line function value: BA0 */
#define PIO_PA20C_SDRAMC_BA0                       (_UL_(1) << 20)

#define PIN_PA0C_SDRAMC_A17                        _L_(0)       /**< SDRAMC signal: A17 on PA0 mux C*/
#define MUX_PA0C_SDRAMC_A17                        _L_(2)       /**< SDRAMC signal line function value: A17 */
#define PIO_PA0C_SDRAMC_A17                        (_UL_(1) << 0)

#define PIN_PA0C_SDRAMC_BA1                        _L_(0)       /**< SDRAMC signal: BA1 on PA0 mux C*/
#define MUX_PA0C_SDRAMC_BA1                        _L_(2)       /**< SDRAMC signal line function value: BA1 */
#define PIO_PA0C_SDRAMC_BA1                        (_UL_(1) << 0)

#define PIN_PA1C_SDRAMC_A18                        _L_(1)       /**< SDRAMC signal: A18 on PA1 mux C*/
#define MUX_PA1C_SDRAMC_A18                        _L_(2)       /**< SDRAMC signal line function value: A18 */
#define PIO_PA1C_SDRAMC_A18                        (_UL_(1) << 1)

#define PIN_PA23C_SDRAMC_A19                       _L_(23)      /**< SDRAMC signal: A19 on PA23 mux C*/
#define MUX_PA23C_SDRAMC_A19                       _L_(2)       /**< SDRAMC signal line function value: A19 */
#define PIO_PA23C_SDRAMC_A19                       (_UL_(1) << 23)

#define PIN_PA24C_SDRAMC_A20                       _L_(24)      /**< SDRAMC signal: A20 on PA24 mux C*/
#define MUX_PA24C_SDRAMC_A20                       _L_(2)       /**< SDRAMC signal line function value: A20 */
#define PIO_PA24C_SDRAMC_A20                       (_UL_(1) << 24)

#define PIN_PC16A_SDRAMC_A21                       _L_(80)      /**< SDRAMC signal: A21 on PC16 mux A*/
#define MUX_PC16A_SDRAMC_A21                       _L_(0)       /**< SDRAMC signal line function value: A21 */
#define PIO_PC16A_SDRAMC_A21                       (_UL_(1) << 16)

#define PIN_PC16A_SDRAMC_NANDALE                   _L_(80)      /**< SDRAMC signal: NANDALE on PC16 mux A*/
#define MUX_PC16A_SDRAMC_NANDALE                   _L_(0)       /**< SDRAMC signal line function value: NANDALE */
#define PIO_PC16A_SDRAMC_NANDALE                   (_UL_(1) << 16)

#define PIN_PC17A_SDRAMC_A22                       _L_(81)      /**< SDRAMC signal: A22 on PC17 mux A*/
#define MUX_PC17A_SDRAMC_A22                       _L_(0)       /**< SDRAMC signal line function value: A22 */
#define PIO_PC17A_SDRAMC_A22                       (_UL_(1) << 17)

#define PIN_PC17A_SDRAMC_NANDCLE                   _L_(81)      /**< SDRAMC signal: NANDCLE on PC17 mux A*/
#define MUX_PC17A_SDRAMC_NANDCLE                   _L_(0)       /**< SDRAMC signal line function value: NANDCLE */
#define PIO_PC17A_SDRAMC_NANDCLE                   (_UL_(1) << 17)

#define PIN_PA25C_SDRAMC_A23                       _L_(25)      /**< SDRAMC signal: A23 on PA25 mux C*/
#define MUX_PA25C_SDRAMC_A23                       _L_(2)       /**< SDRAMC signal line function value: A23 */
#define PIO_PA25C_SDRAMC_A23                       (_UL_(1) << 25)

#define PIN_PD17C_SDRAMC_CAS                       _L_(113)     /**< SDRAMC signal: CAS on PD17 mux C*/
#define MUX_PD17C_SDRAMC_CAS                       _L_(2)       /**< SDRAMC signal line function value: CAS */
#define PIO_PD17C_SDRAMC_CAS                       (_UL_(1) << 17)

#define PIN_PC0A_SDRAMC_D0                         _L_(64)      /**< SDRAMC signal: D0 on PC0 mux A*/
#define MUX_PC0A_SDRAMC_D0                         _L_(0)       /**< SDRAMC signal line function value: D0 */
#define PIO_PC0A_SDRAMC_D0                         (_UL_(1) << 0)

#define PIN_PC1A_SDRAMC_D1                         _L_(65)      /**< SDRAMC signal: D1 on PC1 mux A*/
#define MUX_PC1A_SDRAMC_D1                         _L_(0)       /**< SDRAMC signal line function value: D1 */
#define PIO_PC1A_SDRAMC_D1                         (_UL_(1) << 1)

#define PIN_PC2A_SDRAMC_D2                         _L_(66)      /**< SDRAMC signal: D2 on PC2 mux A*/
#define MUX_PC2A_SDRAMC_D2                         _L_(0)       /**< SDRAMC signal line function value: D2 */
#define PIO_PC2A_SDRAMC_D2                         (_UL_(1) << 2)

#define PIN_PC3A_SDRAMC_D3                         _L_(67)      /**< SDRAMC signal: D3 on PC3 mux A*/
#define MUX_PC3A_SDRAMC_D3                         _L_(0)       /**< SDRAMC signal line function value: D3 */
#define PIO_PC3A_SDRAMC_D3                         (_UL_(1) << 3)

#define PIN_PC4A_SDRAMC_D4                         _L_(68)      /**< SDRAMC signal: D4 on PC4 mux A*/
#define MUX_PC4A_SDRAMC_D4                         _L_(0)       /**< SDRAMC signal line function value: D4 */
#define PIO_PC4A_SDRAMC_D4                         (_UL_(1) << 4)

#define PIN_PC5A_SDRAMC_D5                         _L_(69)      /**< SDRAMC signal: D5 on PC5 mux A*/
#define MUX_PC5A_SDRAMC_D5                         _L_(0)       /**< SDRAMC signal line function value: D5 */
#define PIO_PC5A_SDRAMC_D5                         (_UL_(1) << 5)

#define PIN_PC6A_SDRAMC_D6                         _L_(70)      /**< SDRAMC signal: D6 on PC6 mux A*/
#define MUX_PC6A_SDRAMC_D6                         _L_(0)       /**< SDRAMC signal line function value: D6 */
#define PIO_PC6A_SDRAMC_D6                         (_UL_(1) << 6)

#define PIN_PC7A_SDRAMC_D7                         _L_(71)      /**< SDRAMC signal: D7 on PC7 mux A*/
#define MUX_PC7A_SDRAMC_D7                         _L_(0)       /**< SDRAMC signal line function value: D7 */
#define PIO_PC7A_SDRAMC_D7                         (_UL_(1) << 7)

#define PIN_PE0A_SDRAMC_D8                         _L_(128)     /**< SDRAMC signal: D8 on PE0 mux A*/
#define MUX_PE0A_SDRAMC_D8                         _L_(0)       /**< SDRAMC signal line function value: D8 */
#define PIO_PE0A_SDRAMC_D8                         (_UL_(1) << 0)

#define PIN_PE1A_SDRAMC_D9                         _L_(129)     /**< SDRAMC signal: D9 on PE1 mux A*/
#define MUX_PE1A_SDRAMC_D9                         _L_(0)       /**< SDRAMC signal line function value: D9 */
#define PIO_PE1A_SDRAMC_D9                         (_UL_(1) << 1)

#define PIN_PE2A_SDRAMC_D10                        _L_(130)     /**< SDRAMC signal: D10 on PE2 mux A*/
#define MUX_PE2A_SDRAMC_D10                        _L_(0)       /**< SDRAMC signal line function value: D10 */
#define PIO_PE2A_SDRAMC_D10                        (_UL_(1) << 2)

#define PIN_PE3A_SDRAMC_D11                        _L_(131)     /**< SDRAMC signal: D11 on PE3 mux A*/
#define MUX_PE3A_SDRAMC_D11                        _L_(0)       /**< SDRAMC signal line function value: D11 */
#define PIO_PE3A_SDRAMC_D11                        (_UL_(1) << 3)

#define PIN_PE4A_SDRAMC_D12                        _L_(132)     /**< SDRAMC signal: D12 on PE4 mux A*/
#define MUX_PE4A_SDRAMC_D12                        _L_(0)       /**< SDRAMC signal line function value: D12 */
#define PIO_PE4A_SDRAMC_D12                        (_UL_(1) << 4)

#define PIN_PE5A_SDRAMC_D13                        _L_(133)     /**< SDRAMC signal: D13 on PE5 mux A*/
#define MUX_PE5A_SDRAMC_D13                        _L_(0)       /**< SDRAMC signal line function value: D13 */
#define PIO_PE5A_SDRAMC_D13                        (_UL_(1) << 5)

#define PIN_PA15A_SDRAMC_D14                       _L_(15)      /**< SDRAMC signal: D14 on PA15 mux A*/
#define MUX_PA15A_SDRAMC_D14                       _L_(0)       /**< SDRAMC signal line function value: D14 */
#define PIO_PA15A_SDRAMC_D14                       (_UL_(1) << 15)

#define PIN_PA16A_SDRAMC_D15                       _L_(16)      /**< SDRAMC signal: D15 on PA16 mux A*/
#define MUX_PA16A_SDRAMC_D15                       _L_(0)       /**< SDRAMC signal line function value: D15 */
#define PIO_PA16A_SDRAMC_D15                       (_UL_(1) << 16)

#define PIN_PC9A_SDRAMC_NANDOE                     _L_(73)      /**< SDRAMC signal: NANDOE on PC9 mux A*/
#define MUX_PC9A_SDRAMC_NANDOE                     _L_(0)       /**< SDRAMC signal line function value: NANDOE */
#define PIO_PC9A_SDRAMC_NANDOE                     (_UL_(1) << 9)

#define PIN_PC10A_SDRAMC_NANDWE                    _L_(74)      /**< SDRAMC signal: NANDWE on PC10 mux A*/
#define MUX_PC10A_SDRAMC_NANDWE                    _L_(0)       /**< SDRAMC signal line function value: NANDWE */
#define PIO_PC10A_SDRAMC_NANDWE                    (_UL_(1) << 10)

#define PIN_PC14A_SDRAMC_NCS0                      _L_(78)      /**< SDRAMC signal: NCS0 on PC14 mux A*/
#define MUX_PC14A_SDRAMC_NCS0                      _L_(0)       /**< SDRAMC signal line function value: NCS0 */
#define PIO_PC14A_SDRAMC_NCS0                      (_UL_(1) << 14)

#define PIN_PC15A_SDRAMC_NCS1                      _L_(79)      /**< SDRAMC signal: NCS1 on PC15 mux A*/
#define MUX_PC15A_SDRAMC_NCS1                      _L_(0)       /**< SDRAMC signal line function value: NCS1 */
#define PIO_PC15A_SDRAMC_NCS1                      (_UL_(1) << 15)

#define PIN_PC15A_SDRAMC_SDCS                      _L_(79)      /**< SDRAMC signal: SDCS on PC15 mux A*/
#define MUX_PC15A_SDRAMC_SDCS                      _L_(0)       /**< SDRAMC signal line function value: SDCS */
#define PIO_PC15A_SDRAMC_SDCS                      (_UL_(1) << 15)

#define PIN_PD18A_SDRAMC_NCS1                      _L_(114)     /**< SDRAMC signal: NCS1 on PD18 mux A*/
#define MUX_PD18A_SDRAMC_NCS1                      _L_(0)       /**< SDRAMC signal line function value: NCS1 */
#define PIO_PD18A_SDRAMC_NCS1                      (_UL_(1) << 18)

#define PIN_PD18A_SDRAMC_SDCS                      _L_(114)     /**< SDRAMC signal: SDCS on PD18 mux A*/
#define MUX_PD18A_SDRAMC_SDCS                      _L_(0)       /**< SDRAMC signal line function value: SDCS */
#define PIO_PD18A_SDRAMC_SDCS                      (_UL_(1) << 18)

#define PIN_PA22C_SDRAMC_NCS2                      _L_(22)      /**< SDRAMC signal: NCS2 on PA22 mux C*/
#define MUX_PA22C_SDRAMC_NCS2                      _L_(2)       /**< SDRAMC signal line function value: NCS2 */
#define PIO_PA22C_SDRAMC_NCS2                      (_UL_(1) << 22)

#define PIN_PC12A_SDRAMC_NCS3                      _L_(76)      /**< SDRAMC signal: NCS3 on PC12 mux A*/
#define MUX_PC12A_SDRAMC_NCS3                      _L_(0)       /**< SDRAMC signal line function value: NCS3 */
#define PIO_PC12A_SDRAMC_NCS3                      (_UL_(1) << 12)

#define PIN_PD19A_SDRAMC_NCS3                      _L_(115)     /**< SDRAMC signal: NCS3 on PD19 mux A*/
#define MUX_PD19A_SDRAMC_NCS3                      _L_(0)       /**< SDRAMC signal line function value: NCS3 */
#define PIO_PD19A_SDRAMC_NCS3                      (_UL_(1) << 19)

#define PIN_PC11A_SDRAMC_NRD                       _L_(75)      /**< SDRAMC signal: NRD on PC11 mux A*/
#define MUX_PC11A_SDRAMC_NRD                       _L_(0)       /**< SDRAMC signal line function value: NRD */
#define PIO_PC11A_SDRAMC_NRD                       (_UL_(1) << 11)

#define PIN_PC13A_SDRAMC_NWAIT                     _L_(77)      /**< SDRAMC signal: NWAIT on PC13 mux A*/
#define MUX_PC13A_SDRAMC_NWAIT                     _L_(0)       /**< SDRAMC signal line function value: NWAIT */
#define PIO_PC13A_SDRAMC_NWAIT                     (_UL_(1) << 13)

#define PIN_PC8A_SDRAMC_NWR0                       _L_(72)      /**< SDRAMC signal: NWR0 on PC8 mux A*/
#define MUX_PC8A_SDRAMC_NWR0                       _L_(0)       /**< SDRAMC signal line function value: NWR0 */
#define PIO_PC8A_SDRAMC_NWR0                       (_UL_(1) << 8)

#define PIN_PC8A_SDRAMC_NWE                        _L_(72)      /**< SDRAMC signal: NWE on PC8 mux A*/
#define MUX_PC8A_SDRAMC_NWE                        _L_(0)       /**< SDRAMC signal line function value: NWE */
#define PIO_PC8A_SDRAMC_NWE                        (_UL_(1) << 8)

#define PIN_PD15C_SDRAMC_NWR1                      _L_(111)     /**< SDRAMC signal: NWR1 on PD15 mux C*/
#define MUX_PD15C_SDRAMC_NWR1                      _L_(2)       /**< SDRAMC signal line function value: NWR1 */
#define PIO_PD15C_SDRAMC_NWR1                      (_UL_(1) << 15)

#define PIN_PD15C_SDRAMC_NBS1                      _L_(111)     /**< SDRAMC signal: NBS1 on PD15 mux C*/
#define MUX_PD15C_SDRAMC_NBS1                      _L_(2)       /**< SDRAMC signal line function value: NBS1 */
#define PIO_PD15C_SDRAMC_NBS1                      (_UL_(1) << 15)

#define PIN_PD16C_SDRAMC_RAS                       _L_(112)     /**< SDRAMC signal: RAS on PD16 mux C*/
#define MUX_PD16C_SDRAMC_RAS                       _L_(2)       /**< SDRAMC signal line function value: RAS */
#define PIO_PD16C_SDRAMC_RAS                       (_UL_(1) << 16)

#define PIN_PC13C_SDRAMC_SDA10                     _L_(77)      /**< SDRAMC signal: SDA10 on PC13 mux C*/
#define MUX_PC13C_SDRAMC_SDA10                     _L_(2)       /**< SDRAMC signal line function value: SDA10 */
#define PIO_PC13C_SDRAMC_SDA10                     (_UL_(1) << 13)

#define PIN_PD13C_SDRAMC_SDA10                     _L_(109)     /**< SDRAMC signal: SDA10 on PD13 mux C*/
#define MUX_PD13C_SDRAMC_SDA10                     _L_(2)       /**< SDRAMC signal line function value: SDA10 */
#define PIO_PD13C_SDRAMC_SDA10                     (_UL_(1) << 13)

#define PIN_PD23C_SDRAMC_SDCK                      _L_(119)     /**< SDRAMC signal: SDCK on PD23 mux C*/
#define MUX_PD23C_SDRAMC_SDCK                      _L_(2)       /**< SDRAMC signal line function value: SDCK */
#define PIO_PD23C_SDRAMC_SDCK                      (_UL_(1) << 23)

#define PIN_PD14C_SDRAMC_SDCKE                     _L_(110)     /**< SDRAMC signal: SDCKE on PD14 mux C*/
#define MUX_PD14C_SDRAMC_SDCKE                     _L_(2)       /**< SDRAMC signal line function value: SDCKE */
#define PIO_PD14C_SDRAMC_SDCKE                     (_UL_(1) << 14)

#define PIN_PD29C_SDRAMC_SDWE                      _L_(125)     /**< SDRAMC signal: SDWE on PD29 mux C*/
#define MUX_PD29C_SDRAMC_SDWE                      _L_(2)       /**< SDRAMC signal line function value: SDWE */
#define PIO_PD29C_SDRAMC_SDWE                      (_UL_(1) << 29)

/* ========== PIO definition for SMC peripheral ========== */
#define PIN_PC18A_SMC_A0                           _L_(82)      /**< SMC signal: A0 on PC18 mux A*/
#define MUX_PC18A_SMC_A0                           _L_(0)       /**< SMC signal line function value: A0 */
#define PIO_PC18A_SMC_A0                           (_UL_(1) << 18)

#define PIN_PC18A_SMC_NBS0                         _L_(82)      /**< SMC signal: NBS0 on PC18 mux A*/
#define MUX_PC18A_SMC_NBS0                         _L_(0)       /**< SMC signal line function value: NBS0 */
#define PIO_PC18A_SMC_NBS0                         (_UL_(1) << 18)

#define PIN_PC19A_SMC_A1                           _L_(83)      /**< SMC signal: A1 on PC19 mux A*/
#define MUX_PC19A_SMC_A1                           _L_(0)       /**< SMC signal line function value: A1 */
#define PIO_PC19A_SMC_A1                           (_UL_(1) << 19)

#define PIN_PC20A_SMC_A2                           _L_(84)      /**< SMC signal: A2 on PC20 mux A*/
#define MUX_PC20A_SMC_A2                           _L_(0)       /**< SMC signal line function value: A2 */
#define PIO_PC20A_SMC_A2                           (_UL_(1) << 20)

#define PIN_PC21A_SMC_A3                           _L_(85)      /**< SMC signal: A3 on PC21 mux A*/
#define MUX_PC21A_SMC_A3                           _L_(0)       /**< SMC signal line function value: A3 */
#define PIO_PC21A_SMC_A3                           (_UL_(1) << 21)

#define PIN_PC22A_SMC_A4                           _L_(86)      /**< SMC signal: A4 on PC22 mux A*/
#define MUX_PC22A_SMC_A4                           _L_(0)       /**< SMC signal line function value: A4 */
#define PIO_PC22A_SMC_A4                           (_UL_(1) << 22)

#define PIN_PC23A_SMC_A5                           _L_(87)      /**< SMC signal: A5 on PC23 mux A*/
#define MUX_PC23A_SMC_A5                           _L_(0)       /**< SMC signal line function value: A5 */
#define PIO_PC23A_SMC_A5                           (_UL_(1) << 23)

#define PIN_PC24A_SMC_A6                           _L_(88)      /**< SMC signal: A6 on PC24 mux A*/
#define MUX_PC24A_SMC_A6                           _L_(0)       /**< SMC signal line function value: A6 */
#define PIO_PC24A_SMC_A6                           (_UL_(1) << 24)

#define PIN_PC25A_SMC_A7                           _L_(89)      /**< SMC signal: A7 on PC25 mux A*/
#define MUX_PC25A_SMC_A7                           _L_(0)       /**< SMC signal line function value: A7 */
#define PIO_PC25A_SMC_A7                           (_UL_(1) << 25)

#define PIN_PC26A_SMC_A8                           _L_(90)      /**< SMC signal: A8 on PC26 mux A*/
#define MUX_PC26A_SMC_A8                           _L_(0)       /**< SMC signal line function value: A8 */
#define PIO_PC26A_SMC_A8                           (_UL_(1) << 26)

#define PIN_PC27A_SMC_A9                           _L_(91)      /**< SMC signal: A9 on PC27 mux A*/
#define MUX_PC27A_SMC_A9                           _L_(0)       /**< SMC signal line function value: A9 */
#define PIO_PC27A_SMC_A9                           (_UL_(1) << 27)

#define PIN_PC28A_SMC_A10                          _L_(92)      /**< SMC signal: A10 on PC28 mux A*/
#define MUX_PC28A_SMC_A10                          _L_(0)       /**< SMC signal line function value: A10 */
#define PIO_PC28A_SMC_A10                          (_UL_(1) << 28)

#define PIN_PC29A_SMC_A11                          _L_(93)      /**< SMC signal: A11 on PC29 mux A*/
#define MUX_PC29A_SMC_A11                          _L_(0)       /**< SMC signal line function value: A11 */
#define PIO_PC29A_SMC_A11                          (_UL_(1) << 29)

#define PIN_PC30A_SMC_A12                          _L_(94)      /**< SMC signal: A12 on PC30 mux A*/
#define MUX_PC30A_SMC_A12                          _L_(0)       /**< SMC signal line function value: A12 */
#define PIO_PC30A_SMC_A12                          (_UL_(1) << 30)

#define PIN_PC31A_SMC_A13                          _L_(95)      /**< SMC signal: A13 on PC31 mux A*/
#define MUX_PC31A_SMC_A13                          _L_(0)       /**< SMC signal line function value: A13 */
#define PIO_PC31A_SMC_A13                          (_UL_(1) << 31)

#define PIN_PA18C_SMC_A14                          _L_(18)      /**< SMC signal: A14 on PA18 mux C*/
#define MUX_PA18C_SMC_A14                          _L_(2)       /**< SMC signal line function value: A14 */
#define PIO_PA18C_SMC_A14                          (_UL_(1) << 18)

#define PIN_PA19C_SMC_A15                          _L_(19)      /**< SMC signal: A15 on PA19 mux C*/
#define MUX_PA19C_SMC_A15                          _L_(2)       /**< SMC signal line function value: A15 */
#define PIO_PA19C_SMC_A15                          (_UL_(1) << 19)

#define PIN_PA20C_SMC_A16                          _L_(20)      /**< SMC signal: A16 on PA20 mux C*/
#define MUX_PA20C_SMC_A16                          _L_(2)       /**< SMC signal line function value: A16 */
#define PIO_PA20C_SMC_A16                          (_UL_(1) << 20)

#define PIN_PA20C_SMC_BA0                          _L_(20)      /**< SMC signal: BA0 on PA20 mux C*/
#define MUX_PA20C_SMC_BA0                          _L_(2)       /**< SMC signal line function value: BA0 */
#define PIO_PA20C_SMC_BA0                          (_UL_(1) << 20)

#define PIN_PA0C_SMC_A17                           _L_(0)       /**< SMC signal: A17 on PA0 mux C*/
#define MUX_PA0C_SMC_A17                           _L_(2)       /**< SMC signal line function value: A17 */
#define PIO_PA0C_SMC_A17                           (_UL_(1) << 0)

#define PIN_PA0C_SMC_BA1                           _L_(0)       /**< SMC signal: BA1 on PA0 mux C*/
#define MUX_PA0C_SMC_BA1                           _L_(2)       /**< SMC signal line function value: BA1 */
#define PIO_PA0C_SMC_BA1                           (_UL_(1) << 0)

#define PIN_PA1C_SMC_A18                           _L_(1)       /**< SMC signal: A18 on PA1 mux C*/
#define MUX_PA1C_SMC_A18                           _L_(2)       /**< SMC signal line function value: A18 */
#define PIO_PA1C_SMC_A18                           (_UL_(1) << 1)

#define PIN_PA23C_SMC_A19                          _L_(23)      /**< SMC signal: A19 on PA23 mux C*/
#define MUX_PA23C_SMC_A19                          _L_(2)       /**< SMC signal line function value: A19 */
#define PIO_PA23C_SMC_A19                          (_UL_(1) << 23)

#define PIN_PA24C_SMC_A20                          _L_(24)      /**< SMC signal: A20 on PA24 mux C*/
#define MUX_PA24C_SMC_A20                          _L_(2)       /**< SMC signal line function value: A20 */
#define PIO_PA24C_SMC_A20                          (_UL_(1) << 24)

#define PIN_PC16A_SMC_A21                          _L_(80)      /**< SMC signal: A21 on PC16 mux A*/
#define MUX_PC16A_SMC_A21                          _L_(0)       /**< SMC signal line function value: A21 */
#define PIO_PC16A_SMC_A21                          (_UL_(1) << 16)

#define PIN_PC16A_SMC_NANDALE                      _L_(80)      /**< SMC signal: NANDALE on PC16 mux A*/
#define MUX_PC16A_SMC_NANDALE                      _L_(0)       /**< SMC signal line function value: NANDALE */
#define PIO_PC16A_SMC_NANDALE                      (_UL_(1) << 16)

#define PIN_PC17A_SMC_A22                          _L_(81)      /**< SMC signal: A22 on PC17 mux A*/
#define MUX_PC17A_SMC_A22                          _L_(0)       /**< SMC signal line function value: A22 */
#define PIO_PC17A_SMC_A22                          (_UL_(1) << 17)

#define PIN_PC17A_SMC_NANDCLE                      _L_(81)      /**< SMC signal: NANDCLE on PC17 mux A*/
#define MUX_PC17A_SMC_NANDCLE                      _L_(0)       /**< SMC signal line function value: NANDCLE */
#define PIO_PC17A_SMC_NANDCLE                      (_UL_(1) << 17)

#define PIN_PA25C_SMC_A23                          _L_(25)      /**< SMC signal: A23 on PA25 mux C*/
#define MUX_PA25C_SMC_A23                          _L_(2)       /**< SMC signal line function value: A23 */
#define PIO_PA25C_SMC_A23                          (_UL_(1) << 25)

#define PIN_PD17C_SMC_CAS                          _L_(113)     /**< SMC signal: CAS on PD17 mux C*/
#define MUX_PD17C_SMC_CAS                          _L_(2)       /**< SMC signal line function value: CAS */
#define PIO_PD17C_SMC_CAS                          (_UL_(1) << 17)

#define PIN_PC0A_SMC_D0                            _L_(64)      /**< SMC signal: D0 on PC0 mux A*/
#define MUX_PC0A_SMC_D0                            _L_(0)       /**< SMC signal line function value: D0 */
#define PIO_PC0A_SMC_D0                            (_UL_(1) << 0)

#define PIN_PC1A_SMC_D1                            _L_(65)      /**< SMC signal: D1 on PC1 mux A*/
#define MUX_PC1A_SMC_D1                            _L_(0)       /**< SMC signal line function value: D1 */
#define PIO_PC1A_SMC_D1                            (_UL_(1) << 1)

#define PIN_PC2A_SMC_D2                            _L_(66)      /**< SMC signal: D2 on PC2 mux A*/
#define MUX_PC2A_SMC_D2                            _L_(0)       /**< SMC signal line function value: D2 */
#define PIO_PC2A_SMC_D2                            (_UL_(1) << 2)

#define PIN_PC3A_SMC_D3                            _L_(67)      /**< SMC signal: D3 on PC3 mux A*/
#define MUX_PC3A_SMC_D3                            _L_(0)       /**< SMC signal line function value: D3 */
#define PIO_PC3A_SMC_D3                            (_UL_(1) << 3)

#define PIN_PC4A_SMC_D4                            _L_(68)      /**< SMC signal: D4 on PC4 mux A*/
#define MUX_PC4A_SMC_D4                            _L_(0)       /**< SMC signal line function value: D4 */
#define PIO_PC4A_SMC_D4                            (_UL_(1) << 4)

#define PIN_PC5A_SMC_D5                            _L_(69)      /**< SMC signal: D5 on PC5 mux A*/
#define MUX_PC5A_SMC_D5                            _L_(0)       /**< SMC signal line function value: D5 */
#define PIO_PC5A_SMC_D5                            (_UL_(1) << 5)

#define PIN_PC6A_SMC_D6                            _L_(70)      /**< SMC signal: D6 on PC6 mux A*/
#define MUX_PC6A_SMC_D6                            _L_(0)       /**< SMC signal line function value: D6 */
#define PIO_PC6A_SMC_D6                            (_UL_(1) << 6)

#define PIN_PC7A_SMC_D7                            _L_(71)      /**< SMC signal: D7 on PC7 mux A*/
#define MUX_PC7A_SMC_D7                            _L_(0)       /**< SMC signal line function value: D7 */
#define PIO_PC7A_SMC_D7                            (_UL_(1) << 7)

#define PIN_PE0A_SMC_D8                            _L_(128)     /**< SMC signal: D8 on PE0 mux A*/
#define MUX_PE0A_SMC_D8                            _L_(0)       /**< SMC signal line function value: D8 */
#define PIO_PE0A_SMC_D8                            (_UL_(1) << 0)

#define PIN_PE1A_SMC_D9                            _L_(129)     /**< SMC signal: D9 on PE1 mux A*/
#define MUX_PE1A_SMC_D9                            _L_(0)       /**< SMC signal line function value: D9 */
#define PIO_PE1A_SMC_D9                            (_UL_(1) << 1)

#define PIN_PE2A_SMC_D10                           _L_(130)     /**< SMC signal: D10 on PE2 mux A*/
#define MUX_PE2A_SMC_D10                           _L_(0)       /**< SMC signal line function value: D10 */
#define PIO_PE2A_SMC_D10                           (_UL_(1) << 2)

#define PIN_PE3A_SMC_D11                           _L_(131)     /**< SMC signal: D11 on PE3 mux A*/
#define MUX_PE3A_SMC_D11                           _L_(0)       /**< SMC signal line function value: D11 */
#define PIO_PE3A_SMC_D11                           (_UL_(1) << 3)

#define PIN_PE4A_SMC_D12                           _L_(132)     /**< SMC signal: D12 on PE4 mux A*/
#define MUX_PE4A_SMC_D12                           _L_(0)       /**< SMC signal line function value: D12 */
#define PIO_PE4A_SMC_D12                           (_UL_(1) << 4)

#define PIN_PE5A_SMC_D13                           _L_(133)     /**< SMC signal: D13 on PE5 mux A*/
#define MUX_PE5A_SMC_D13                           _L_(0)       /**< SMC signal line function value: D13 */
#define PIO_PE5A_SMC_D13                           (_UL_(1) << 5)

#define PIN_PA15A_SMC_D14                          _L_(15)      /**< SMC signal: D14 on PA15 mux A*/
#define MUX_PA15A_SMC_D14                          _L_(0)       /**< SMC signal line function value: D14 */
#define PIO_PA15A_SMC_D14                          (_UL_(1) << 15)

#define PIN_PA16A_SMC_D15                          _L_(16)      /**< SMC signal: D15 on PA16 mux A*/
#define MUX_PA16A_SMC_D15                          _L_(0)       /**< SMC signal line function value: D15 */
#define PIO_PA16A_SMC_D15                          (_UL_(1) << 16)

#define PIN_PC9A_SMC_NANDOE                        _L_(73)      /**< SMC signal: NANDOE on PC9 mux A*/
#define MUX_PC9A_SMC_NANDOE                        _L_(0)       /**< SMC signal line function value: NANDOE */
#define PIO_PC9A_SMC_NANDOE                        (_UL_(1) << 9)

#define PIN_PC10A_SMC_NANDWE                       _L_(74)      /**< SMC signal: NANDWE on PC10 mux A*/
#define MUX_PC10A_SMC_NANDWE                       _L_(0)       /**< SMC signal line function value: NANDWE */
#define PIO_PC10A_SMC_NANDWE                       (_UL_(1) << 10)

#define PIN_PC14A_SMC_NCS0                         _L_(78)      /**< SMC signal: NCS0 on PC14 mux A*/
#define MUX_PC14A_SMC_NCS0                         _L_(0)       /**< SMC signal line function value: NCS0 */
#define PIO_PC14A_SMC_NCS0                         (_UL_(1) << 14)

#define PIN_PC15A_SMC_NCS1                         _L_(79)      /**< SMC signal: NCS1 on PC15 mux A*/
#define MUX_PC15A_SMC_NCS1                         _L_(0)       /**< SMC signal line function value: NCS1 */
#define PIO_PC15A_SMC_NCS1                         (_UL_(1) << 15)

#define PIN_PC15A_SMC_SDCS                         _L_(79)      /**< SMC signal: SDCS on PC15 mux A*/
#define MUX_PC15A_SMC_SDCS                         _L_(0)       /**< SMC signal line function value: SDCS */
#define PIO_PC15A_SMC_SDCS                         (_UL_(1) << 15)

#define PIN_PD18A_SMC_NCS1                         _L_(114)     /**< SMC signal: NCS1 on PD18 mux A*/
#define MUX_PD18A_SMC_NCS1                         _L_(0)       /**< SMC signal line function value: NCS1 */
#define PIO_PD18A_SMC_NCS1                         (_UL_(1) << 18)

#define PIN_PD18A_SMC_SDCS                         _L_(114)     /**< SMC signal: SDCS on PD18 mux A*/
#define MUX_PD18A_SMC_SDCS                         _L_(0)       /**< SMC signal line function value: SDCS */
#define PIO_PD18A_SMC_SDCS                         (_UL_(1) << 18)

#define PIN_PA22C_SMC_NCS2                         _L_(22)      /**< SMC signal: NCS2 on PA22 mux C*/
#define MUX_PA22C_SMC_NCS2                         _L_(2)       /**< SMC signal line function value: NCS2 */
#define PIO_PA22C_SMC_NCS2                         (_UL_(1) << 22)

#define PIN_PC12A_SMC_NCS3                         _L_(76)      /**< SMC signal: NCS3 on PC12 mux A*/
#define MUX_PC12A_SMC_NCS3                         _L_(0)       /**< SMC signal line function value: NCS3 */
#define PIO_PC12A_SMC_NCS3                         (_UL_(1) << 12)

#define PIN_PD19A_SMC_NCS3                         _L_(115)     /**< SMC signal: NCS3 on PD19 mux A*/
#define MUX_PD19A_SMC_NCS3                         _L_(0)       /**< SMC signal line function value: NCS3 */
#define PIO_PD19A_SMC_NCS3                         (_UL_(1) << 19)

#define PIN_PC11A_SMC_NRD                          _L_(75)      /**< SMC signal: NRD on PC11 mux A*/
#define MUX_PC11A_SMC_NRD                          _L_(0)       /**< SMC signal line function value: NRD */
#define PIO_PC11A_SMC_NRD                          (_UL_(1) << 11)

#define PIN_PC13A_SMC_NWAIT                        _L_(77)      /**< SMC signal: NWAIT on PC13 mux A*/
#define MUX_PC13A_SMC_NWAIT                        _L_(0)       /**< SMC signal line function value: NWAIT */
#define PIO_PC13A_SMC_NWAIT                        (_UL_(1) << 13)

#define PIN_PC8A_SMC_NWR0                          _L_(72)      /**< SMC signal: NWR0 on PC8 mux A*/
#define MUX_PC8A_SMC_NWR0                          _L_(0)       /**< SMC signal line function value: NWR0 */
#define PIO_PC8A_SMC_NWR0                          (_UL_(1) << 8)

#define PIN_PC8A_SMC_NWE                           _L_(72)      /**< SMC signal: NWE on PC8 mux A*/
#define MUX_PC8A_SMC_NWE                           _L_(0)       /**< SMC signal line function value: NWE */
#define PIO_PC8A_SMC_NWE                           (_UL_(1) << 8)

#define PIN_PD15C_SMC_NWR1                         _L_(111)     /**< SMC signal: NWR1 on PD15 mux C*/
#define MUX_PD15C_SMC_NWR1                         _L_(2)       /**< SMC signal line function value: NWR1 */
#define PIO_PD15C_SMC_NWR1                         (_UL_(1) << 15)

#define PIN_PD15C_SMC_NBS1                         _L_(111)     /**< SMC signal: NBS1 on PD15 mux C*/
#define MUX_PD15C_SMC_NBS1                         _L_(2)       /**< SMC signal line function value: NBS1 */
#define PIO_PD15C_SMC_NBS1                         (_UL_(1) << 15)

#define PIN_PD16C_SMC_RAS                          _L_(112)     /**< SMC signal: RAS on PD16 mux C*/
#define MUX_PD16C_SMC_RAS                          _L_(2)       /**< SMC signal line function value: RAS */
#define PIO_PD16C_SMC_RAS                          (_UL_(1) << 16)

#define PIN_PC13C_SMC_SDA10                        _L_(77)      /**< SMC signal: SDA10 on PC13 mux C*/
#define MUX_PC13C_SMC_SDA10                        _L_(2)       /**< SMC signal line function value: SDA10 */
#define PIO_PC13C_SMC_SDA10                        (_UL_(1) << 13)

#define PIN_PD13C_SMC_SDA10                        _L_(109)     /**< SMC signal: SDA10 on PD13 mux C*/
#define MUX_PD13C_SMC_SDA10                        _L_(2)       /**< SMC signal line function value: SDA10 */
#define PIO_PD13C_SMC_SDA10                        (_UL_(1) << 13)

#define PIN_PD23C_SMC_SDCK                         _L_(119)     /**< SMC signal: SDCK on PD23 mux C*/
#define MUX_PD23C_SMC_SDCK                         _L_(2)       /**< SMC signal line function value: SDCK */
#define PIO_PD23C_SMC_SDCK                         (_UL_(1) << 23)

#define PIN_PD14C_SMC_SDCKE                        _L_(110)     /**< SMC signal: SDCKE on PD14 mux C*/
#define MUX_PD14C_SMC_SDCKE                        _L_(2)       /**< SMC signal line function value: SDCKE */
#define PIO_PD14C_SMC_SDCKE                        (_UL_(1) << 14)

#define PIN_PD29C_SMC_SDWE                         _L_(125)     /**< SMC signal: SDWE on PD29 mux C*/
#define MUX_PD29C_SMC_SDWE                         _L_(2)       /**< SMC signal line function value: SDWE */
#define PIO_PD29C_SMC_SDWE                         (_UL_(1) << 29)

/* ========== PIO definition for SPI0 peripheral ========== */
#define PIN_PD20B_SPI0_MISO                        _L_(116)     /**< SPI0 signal: MISO on PD20 mux B*/
#define MUX_PD20B_SPI0_MISO                        _L_(1)       /**< SPI0 signal line function value: MISO */
#define PIO_PD20B_SPI0_MISO                        (_UL_(1) << 20)

#define PIN_PD21B_SPI0_MOSI                        _L_(117)     /**< SPI0 signal: MOSI on PD21 mux B*/
#define MUX_PD21B_SPI0_MOSI                        _L_(1)       /**< SPI0 signal line function value: MOSI */
#define PIO_PD21B_SPI0_MOSI                        (_UL_(1) << 21)

#define PIN_PB2D_SPI0_NPCS0                        _L_(34)      /**< SPI0 signal: NPCS0 on PB2 mux D*/
#define MUX_PB2D_SPI0_NPCS0                        _L_(3)       /**< SPI0 signal line function value: NPCS0 */
#define PIO_PB2D_SPI0_NPCS0                        (_UL_(1) << 2)

#define PIN_PA31A_SPI0_NPCS1                       _L_(31)      /**< SPI0 signal: NPCS1 on PA31 mux A*/
#define MUX_PA31A_SPI0_NPCS1                       _L_(0)       /**< SPI0 signal line function value: NPCS1 */
#define PIO_PA31A_SPI0_NPCS1                       (_UL_(1) << 31)

#define PIN_PD25B_SPI0_NPCS1                       _L_(121)     /**< SPI0 signal: NPCS1 on PD25 mux B*/
#define MUX_PD25B_SPI0_NPCS1                       _L_(1)       /**< SPI0 signal line function value: NPCS1 */
#define PIO_PD25B_SPI0_NPCS1                       (_UL_(1) << 25)

#define PIN_PD12C_SPI0_NPCS2                       _L_(108)     /**< SPI0 signal: NPCS2 on PD12 mux C*/
#define MUX_PD12C_SPI0_NPCS2                       _L_(2)       /**< SPI0 signal line function value: NPCS2 */
#define PIO_PD12C_SPI0_NPCS2                       (_UL_(1) << 12)

#define PIN_PD27B_SPI0_NPCS3                       _L_(123)     /**< SPI0 signal: NPCS3 on PD27 mux B*/
#define MUX_PD27B_SPI0_NPCS3                       _L_(1)       /**< SPI0 signal line function value: NPCS3 */
#define PIO_PD27B_SPI0_NPCS3                       (_UL_(1) << 27)

#define PIN_PD22B_SPI0_SPCK                        _L_(118)     /**< SPI0 signal: SPCK on PD22 mux B*/
#define MUX_PD22B_SPI0_SPCK                        _L_(1)       /**< SPI0 signal line function value: SPCK */
#define PIO_PD22B_SPI0_SPCK                        (_UL_(1) << 22)

/* ========== PIO definition for SPI1 peripheral ========== */
#define PIN_PC26C_SPI1_MISO                        _L_(90)      /**< SPI1 signal: MISO on PC26 mux C*/
#define MUX_PC26C_SPI1_MISO                        _L_(2)       /**< SPI1 signal line function value: MISO */
#define PIO_PC26C_SPI1_MISO                        (_UL_(1) << 26)

#define PIN_PC27C_SPI1_MOSI                        _L_(91)      /**< SPI1 signal: MOSI on PC27 mux C*/
#define MUX_PC27C_SPI1_MOSI                        _L_(2)       /**< SPI1 signal line function value: MOSI */
#define PIO_PC27C_SPI1_MOSI                        (_UL_(1) << 27)

#define PIN_PC25C_SPI1_NPCS0                       _L_(89)      /**< SPI1 signal: NPCS0 on PC25 mux C*/
#define MUX_PC25C_SPI1_NPCS0                       _L_(2)       /**< SPI1 signal line function value: NPCS0 */
#define PIO_PC25C_SPI1_NPCS0                       (_UL_(1) << 25)

#define PIN_PC28C_SPI1_NPCS1                       _L_(92)      /**< SPI1 signal: NPCS1 on PC28 mux C*/
#define MUX_PC28C_SPI1_NPCS1                       _L_(2)       /**< SPI1 signal line function value: NPCS1 */
#define PIO_PC28C_SPI1_NPCS1                       (_UL_(1) << 28)

#define PIN_PD0C_SPI1_NPCS1                        _L_(96)      /**< SPI1 signal: NPCS1 on PD0 mux C*/
#define MUX_PD0C_SPI1_NPCS1                        _L_(2)       /**< SPI1 signal line function value: NPCS1 */
#define PIO_PD0C_SPI1_NPCS1                        (_UL_(1) << 0)

#define PIN_PC29C_SPI1_NPCS2                       _L_(93)      /**< SPI1 signal: NPCS2 on PC29 mux C*/
#define MUX_PC29C_SPI1_NPCS2                       _L_(2)       /**< SPI1 signal line function value: NPCS2 */
#define PIO_PC29C_SPI1_NPCS2                       (_UL_(1) << 29)

#define PIN_PD1C_SPI1_NPCS2                        _L_(97)      /**< SPI1 signal: NPCS2 on PD1 mux C*/
#define MUX_PD1C_SPI1_NPCS2                        _L_(2)       /**< SPI1 signal line function value: NPCS2 */
#define PIO_PD1C_SPI1_NPCS2                        (_UL_(1) << 1)

#define PIN_PC30C_SPI1_NPCS3                       _L_(94)      /**< SPI1 signal: NPCS3 on PC30 mux C*/
#define MUX_PC30C_SPI1_NPCS3                       _L_(2)       /**< SPI1 signal line function value: NPCS3 */
#define PIO_PC30C_SPI1_NPCS3                       (_UL_(1) << 30)

#define PIN_PD2C_SPI1_NPCS3                        _L_(98)      /**< SPI1 signal: NPCS3 on PD2 mux C*/
#define MUX_PD2C_SPI1_NPCS3                        _L_(2)       /**< SPI1 signal line function value: NPCS3 */
#define PIO_PD2C_SPI1_NPCS3                        (_UL_(1) << 2)

#define PIN_PC24C_SPI1_SPCK                        _L_(88)      /**< SPI1 signal: SPCK on PC24 mux C*/
#define MUX_PC24C_SPI1_SPCK                        _L_(2)       /**< SPI1 signal line function value: SPCK */
#define PIO_PC24C_SPI1_SPCK                        (_UL_(1) << 24)

/* ========== PIO definition for SSC peripheral ========== */
#define PIN_PA10C_SSC_RD                           _L_(10)      /**< SSC signal: RD on PA10 mux C*/
#define MUX_PA10C_SSC_RD                           _L_(2)       /**< SSC signal line function value: RD */
#define PIO_PA10C_SSC_RD                           (_UL_(1) << 10)

#define PIN_PD24B_SSC_RF                           _L_(120)     /**< SSC signal: RF on PD24 mux B*/
#define MUX_PD24B_SSC_RF                           _L_(1)       /**< SSC signal line function value: RF */
#define PIO_PD24B_SSC_RF                           (_UL_(1) << 24)

#define PIN_PA22A_SSC_RK                           _L_(22)      /**< SSC signal: RK on PA22 mux A*/
#define MUX_PA22A_SSC_RK                           _L_(0)       /**< SSC signal line function value: RK */
#define PIO_PA22A_SSC_RK                           (_UL_(1) << 22)

#define PIN_PB5D_SSC_TD                            _L_(37)      /**< SSC signal: TD on PB5 mux D*/
#define MUX_PB5D_SSC_TD                            _L_(3)       /**< SSC signal line function value: TD */
#define PIO_PB5D_SSC_TD                            (_UL_(1) << 5)

#define PIN_PD10C_SSC_TD                           _L_(106)     /**< SSC signal: TD on PD10 mux C*/
#define MUX_PD10C_SSC_TD                           _L_(2)       /**< SSC signal line function value: TD */
#define PIO_PD10C_SSC_TD                           (_UL_(1) << 10)

#define PIN_PD26B_SSC_TD                           _L_(122)     /**< SSC signal: TD on PD26 mux B*/
#define MUX_PD26B_SSC_TD                           _L_(1)       /**< SSC signal line function value: TD */
#define PIO_PD26B_SSC_TD                           (_UL_(1) << 26)

#define PIN_PB0D_SSC_TF                            _L_(32)      /**< SSC signal: TF on PB0 mux D*/
#define MUX_PB0D_SSC_TF                            _L_(3)       /**< SSC signal line function value: TF */
#define PIO_PB0D_SSC_TF                            (_UL_(1) << 0)

#define PIN_PB1D_SSC_TK                            _L_(33)      /**< SSC signal: TK on PB1 mux D*/
#define MUX_PB1D_SSC_TK                            _L_(3)       /**< SSC signal line function value: TK */
#define PIO_PB1D_SSC_TK                            (_UL_(1) << 1)

/* ========== PIO definition for TC0 peripheral ========== */
#define PIN_PA4B_TC0_TCLK0                         _L_(4)       /**< TC0 signal: TCLK0 on PA4 mux B*/
#define MUX_PA4B_TC0_TCLK0                         _L_(1)       /**< TC0 signal line function value: TCLK0 */
#define PIO_PA4B_TC0_TCLK0                         (_UL_(1) << 4)

#define PIN_PA28B_TC0_TCLK1                        _L_(28)      /**< TC0 signal: TCLK1 on PA28 mux B*/
#define MUX_PA28B_TC0_TCLK1                        _L_(1)       /**< TC0 signal line function value: TCLK1 */
#define PIO_PA28B_TC0_TCLK1                        (_UL_(1) << 28)

#define PIN_PA29B_TC0_TCLK2                        _L_(29)      /**< TC0 signal: TCLK2 on PA29 mux B*/
#define MUX_PA29B_TC0_TCLK2                        _L_(1)       /**< TC0 signal line function value: TCLK2 */
#define PIO_PA29B_TC0_TCLK2                        (_UL_(1) << 29)

#define PIN_PA0B_TC0_TIOA0                         _L_(0)       /**< TC0 signal: TIOA0 on PA0 mux B*/
#define MUX_PA0B_TC0_TIOA0                         _L_(1)       /**< TC0 signal line function value: TIOA0 */
#define PIO_PA0B_TC0_TIOA0                         (_UL_(1) << 0)

#define PIN_PA15B_TC0_TIOA1                        _L_(15)      /**< TC0 signal: TIOA1 on PA15 mux B*/
#define MUX_PA15B_TC0_TIOA1                        _L_(1)       /**< TC0 signal line function value: TIOA1 */
#define PIO_PA15B_TC0_TIOA1                        (_UL_(1) << 15)

#define PIN_PA26B_TC0_TIOA2                        _L_(26)      /**< TC0 signal: TIOA2 on PA26 mux B*/
#define MUX_PA26B_TC0_TIOA2                        _L_(1)       /**< TC0 signal line function value: TIOA2 */
#define PIO_PA26B_TC0_TIOA2                        (_UL_(1) << 26)

#define PIN_PA1B_TC0_TIOB0                         _L_(1)       /**< TC0 signal: TIOB0 on PA1 mux B*/
#define MUX_PA1B_TC0_TIOB0                         _L_(1)       /**< TC0 signal line function value: TIOB0 */
#define PIO_PA1B_TC0_TIOB0                         (_UL_(1) << 1)

#define PIN_PA16B_TC0_TIOB1                        _L_(16)      /**< TC0 signal: TIOB1 on PA16 mux B*/
#define MUX_PA16B_TC0_TIOB1                        _L_(1)       /**< TC0 signal line function value: TIOB1 */
#define PIO_PA16B_TC0_TIOB1                        (_UL_(1) << 16)

#define PIN_PA27B_TC0_TIOB2                        _L_(27)      /**< TC0 signal: TIOB2 on PA27 mux B*/
#define MUX_PA27B_TC0_TIOB2                        _L_(1)       /**< TC0 signal line function value: TIOB2 */
#define PIO_PA27B_TC0_TIOB2                        (_UL_(1) << 27)

/* ========== PIO definition for TC1 peripheral ========== */
#define PIN_PC25B_TC1_TCLK3                        _L_(89)      /**< TC1 signal: TCLK3 on PC25 mux B*/
#define MUX_PC25B_TC1_TCLK3                        _L_(1)       /**< TC1 signal line function value: TCLK3 */
#define PIO_PC25B_TC1_TCLK3                        (_UL_(1) << 25)

#define PIN_PC28B_TC1_TCLK4                        _L_(92)      /**< TC1 signal: TCLK4 on PC28 mux B*/
#define MUX_PC28B_TC1_TCLK4                        _L_(1)       /**< TC1 signal line function value: TCLK4 */
#define PIO_PC28B_TC1_TCLK4                        (_UL_(1) << 28)

#define PIN_PC31B_TC1_TCLK5                        _L_(95)      /**< TC1 signal: TCLK5 on PC31 mux B*/
#define MUX_PC31B_TC1_TCLK5                        _L_(1)       /**< TC1 signal line function value: TCLK5 */
#define PIO_PC31B_TC1_TCLK5                        (_UL_(1) << 31)

#define PIN_PC23B_TC1_TIOA3                        _L_(87)      /**< TC1 signal: TIOA3 on PC23 mux B*/
#define MUX_PC23B_TC1_TIOA3                        _L_(1)       /**< TC1 signal line function value: TIOA3 */
#define PIO_PC23B_TC1_TIOA3                        (_UL_(1) << 23)

#define PIN_PC26B_TC1_TIOA4                        _L_(90)      /**< TC1 signal: TIOA4 on PC26 mux B*/
#define MUX_PC26B_TC1_TIOA4                        _L_(1)       /**< TC1 signal line function value: TIOA4 */
#define PIO_PC26B_TC1_TIOA4                        (_UL_(1) << 26)

#define PIN_PC29B_TC1_TIOA5                        _L_(93)      /**< TC1 signal: TIOA5 on PC29 mux B*/
#define MUX_PC29B_TC1_TIOA5                        _L_(1)       /**< TC1 signal line function value: TIOA5 */
#define PIO_PC29B_TC1_TIOA5                        (_UL_(1) << 29)

#define PIN_PC24B_TC1_TIOB3                        _L_(88)      /**< TC1 signal: TIOB3 on PC24 mux B*/
#define MUX_PC24B_TC1_TIOB3                        _L_(1)       /**< TC1 signal line function value: TIOB3 */
#define PIO_PC24B_TC1_TIOB3                        (_UL_(1) << 24)

#define PIN_PC27B_TC1_TIOB4                        _L_(91)      /**< TC1 signal: TIOB4 on PC27 mux B*/
#define MUX_PC27B_TC1_TIOB4                        _L_(1)       /**< TC1 signal line function value: TIOB4 */
#define PIO_PC27B_TC1_TIOB4                        (_UL_(1) << 27)

#define PIN_PC30B_TC1_TIOB5                        _L_(94)      /**< TC1 signal: TIOB5 on PC30 mux B*/
#define MUX_PC30B_TC1_TIOB5                        _L_(1)       /**< TC1 signal line function value: TIOB5 */
#define PIO_PC30B_TC1_TIOB5                        (_UL_(1) << 30)

/* ========== PIO definition for TC2 peripheral ========== */
#define PIN_PC7B_TC2_TCLK6                         _L_(71)      /**< TC2 signal: TCLK6 on PC7 mux B*/
#define MUX_PC7B_TC2_TCLK6                         _L_(1)       /**< TC2 signal line function value: TCLK6 */
#define PIO_PC7B_TC2_TCLK6                         (_UL_(1) << 7)

#define PIN_PC10B_TC2_TCLK7                        _L_(74)      /**< TC2 signal: TCLK7 on PC10 mux B*/
#define MUX_PC10B_TC2_TCLK7                        _L_(1)       /**< TC2 signal line function value: TCLK7 */
#define PIO_PC10B_TC2_TCLK7                        (_UL_(1) << 10)

#define PIN_PC14B_TC2_TCLK8                        _L_(78)      /**< TC2 signal: TCLK8 on PC14 mux B*/
#define MUX_PC14B_TC2_TCLK8                        _L_(1)       /**< TC2 signal line function value: TCLK8 */
#define PIO_PC14B_TC2_TCLK8                        (_UL_(1) << 14)

#define PIN_PC5B_TC2_TIOA6                         _L_(69)      /**< TC2 signal: TIOA6 on PC5 mux B*/
#define MUX_PC5B_TC2_TIOA6                         _L_(1)       /**< TC2 signal line function value: TIOA6 */
#define PIO_PC5B_TC2_TIOA6                         (_UL_(1) << 5)

#define PIN_PC8B_TC2_TIOA7                         _L_(72)      /**< TC2 signal: TIOA7 on PC8 mux B*/
#define MUX_PC8B_TC2_TIOA7                         _L_(1)       /**< TC2 signal line function value: TIOA7 */
#define PIO_PC8B_TC2_TIOA7                         (_UL_(1) << 8)

#define PIN_PC11B_TC2_TIOA8                        _L_(75)      /**< TC2 signal: TIOA8 on PC11 mux B*/
#define MUX_PC11B_TC2_TIOA8                        _L_(1)       /**< TC2 signal line function value: TIOA8 */
#define PIO_PC11B_TC2_TIOA8                        (_UL_(1) << 11)

#define PIN_PC6B_TC2_TIOB6                         _L_(70)      /**< TC2 signal: TIOB6 on PC6 mux B*/
#define MUX_PC6B_TC2_TIOB6                         _L_(1)       /**< TC2 signal line function value: TIOB6 */
#define PIO_PC6B_TC2_TIOB6                         (_UL_(1) << 6)

#define PIN_PC9B_TC2_TIOB7                         _L_(73)      /**< TC2 signal: TIOB7 on PC9 mux B*/
#define MUX_PC9B_TC2_TIOB7                         _L_(1)       /**< TC2 signal line function value: TIOB7 */
#define PIO_PC9B_TC2_TIOB7                         (_UL_(1) << 9)

#define PIN_PC12B_TC2_TIOB8                        _L_(76)      /**< TC2 signal: TIOB8 on PC12 mux B*/
#define MUX_PC12B_TC2_TIOB8                        _L_(1)       /**< TC2 signal line function value: TIOB8 */
#define PIO_PC12B_TC2_TIOB8                        (_UL_(1) << 12)

/* ========== PIO definition for TC3 peripheral ========== */
#define PIN_PE2B_TC3_TCLK9                         _L_(130)     /**< TC3 signal: TCLK9 on PE2 mux B*/
#define MUX_PE2B_TC3_TCLK9                         _L_(1)       /**< TC3 signal line function value: TCLK9 */
#define PIO_PE2B_TC3_TCLK9                         (_UL_(1) << 2)

#define PIN_PE5B_TC3_TCLK10                        _L_(133)     /**< TC3 signal: TCLK10 on PE5 mux B*/
#define MUX_PE5B_TC3_TCLK10                        _L_(1)       /**< TC3 signal line function value: TCLK10 */
#define PIO_PE5B_TC3_TCLK10                        (_UL_(1) << 5)

#define PIN_PD24C_TC3_TCLK11                       _L_(120)     /**< TC3 signal: TCLK11 on PD24 mux C*/
#define MUX_PD24C_TC3_TCLK11                       _L_(2)       /**< TC3 signal line function value: TCLK11 */
#define PIO_PD24C_TC3_TCLK11                       (_UL_(1) << 24)

#define PIN_PE0B_TC3_TIOA9                         _L_(128)     /**< TC3 signal: TIOA9 on PE0 mux B*/
#define MUX_PE0B_TC3_TIOA9                         _L_(1)       /**< TC3 signal line function value: TIOA9 */
#define PIO_PE0B_TC3_TIOA9                         (_UL_(1) << 0)

#define PIN_PE3B_TC3_TIOA10                        _L_(131)     /**< TC3 signal: TIOA10 on PE3 mux B*/
#define MUX_PE3B_TC3_TIOA10                        _L_(1)       /**< TC3 signal line function value: TIOA10 */
#define PIO_PE3B_TC3_TIOA10                        (_UL_(1) << 3)

#define PIN_PD21C_TC3_TIOA11                       _L_(117)     /**< TC3 signal: TIOA11 on PD21 mux C*/
#define MUX_PD21C_TC3_TIOA11                       _L_(2)       /**< TC3 signal line function value: TIOA11 */
#define PIO_PD21C_TC3_TIOA11                       (_UL_(1) << 21)

#define PIN_PE1B_TC3_TIOB9                         _L_(129)     /**< TC3 signal: TIOB9 on PE1 mux B*/
#define MUX_PE1B_TC3_TIOB9                         _L_(1)       /**< TC3 signal line function value: TIOB9 */
#define PIO_PE1B_TC3_TIOB9                         (_UL_(1) << 1)

#define PIN_PE4B_TC3_TIOB10                        _L_(132)     /**< TC3 signal: TIOB10 on PE4 mux B*/
#define MUX_PE4B_TC3_TIOB10                        _L_(1)       /**< TC3 signal line function value: TIOB10 */
#define PIO_PE4B_TC3_TIOB10                        (_UL_(1) << 4)

#define PIN_PD22C_TC3_TIOB11                       _L_(118)     /**< TC3 signal: TIOB11 on PD22 mux C*/
#define MUX_PD22C_TC3_TIOB11                       _L_(2)       /**< TC3 signal line function value: TIOB11 */
#define PIO_PD22C_TC3_TIOB11                       (_UL_(1) << 22)

/* ========== PIO definition for TWIHS0 peripheral ========== */
#define PIN_PA4A_TWIHS0_TWCK0                      _L_(4)       /**< TWIHS0 signal: TWCK0 on PA4 mux A*/
#define MUX_PA4A_TWIHS0_TWCK0                      _L_(0)       /**< TWIHS0 signal line function value: TWCK0 */
#define PIO_PA4A_TWIHS0_TWCK0                      (_UL_(1) << 4)

#define PIN_PA3A_TWIHS0_TWD0                       _L_(3)       /**< TWIHS0 signal: TWD0 on PA3 mux A*/
#define MUX_PA3A_TWIHS0_TWD0                       _L_(0)       /**< TWIHS0 signal line function value: TWD0 */
#define PIO_PA3A_TWIHS0_TWD0                       (_UL_(1) << 3)

/* ========== PIO definition for TWIHS1 peripheral ========== */
#define PIN_PB5A_TWIHS1_TWCK1                      _L_(37)      /**< TWIHS1 signal: TWCK1 on PB5 mux A*/
#define MUX_PB5A_TWIHS1_TWCK1                      _L_(0)       /**< TWIHS1 signal line function value: TWCK1 */
#define PIO_PB5A_TWIHS1_TWCK1                      (_UL_(1) << 5)

#define PIN_PB4A_TWIHS1_TWD1                       _L_(36)      /**< TWIHS1 signal: TWD1 on PB4 mux A*/
#define MUX_PB4A_TWIHS1_TWD1                       _L_(0)       /**< TWIHS1 signal line function value: TWD1 */
#define PIO_PB4A_TWIHS1_TWD1                       (_UL_(1) << 4)

/* ========== PIO definition for TWIHS2 peripheral ========== */
#define PIN_PD28C_TWIHS2_TWCK2                     _L_(124)     /**< TWIHS2 signal: TWCK2 on PD28 mux C*/
#define MUX_PD28C_TWIHS2_TWCK2                     _L_(2)       /**< TWIHS2 signal line function value: TWCK2 */
#define PIO_PD28C_TWIHS2_TWCK2                     (_UL_(1) << 28)

#define PIN_PD27C_TWIHS2_TWD2                      _L_(123)     /**< TWIHS2 signal: TWD2 on PD27 mux C*/
#define MUX_PD27C_TWIHS2_TWD2                      _L_(2)       /**< TWIHS2 signal line function value: TWD2 */
#define PIO_PD27C_TWIHS2_TWD2                      (_UL_(1) << 27)

/* ========== PIO definition for UART0 peripheral ========== */
#define PIN_PA9A_UART0_URXD0                       _L_(9)       /**< UART0 signal: URXD0 on PA9 mux A*/
#define MUX_PA9A_UART0_URXD0                       _L_(0)       /**< UART0 signal line function value: URXD0 */
#define PIO_PA9A_UART0_URXD0                       (_UL_(1) << 9)

#define PIN_PA10A_UART0_UTXD0                      _L_(10)      /**< UART0 signal: UTXD0 on PA10 mux A*/
#define MUX_PA10A_UART0_UTXD0                      _L_(0)       /**< UART0 signal line function value: UTXD0 */
#define PIO_PA10A_UART0_UTXD0                      (_UL_(1) << 10)

/* ========== PIO definition for UART1 peripheral ========== */
#define PIN_PA5C_UART1_URXD1                       _L_(5)       /**< UART1 signal: URXD1 on PA5 mux C*/
#define MUX_PA5C_UART1_URXD1                       _L_(2)       /**< UART1 signal line function value: URXD1 */
#define PIO_PA5C_UART1_URXD1                       (_UL_(1) << 5)

#define PIN_PA4C_UART1_UTXD1                       _L_(4)       /**< UART1 signal: UTXD1 on PA4 mux C*/
#define MUX_PA4C_UART1_UTXD1                       _L_(2)       /**< UART1 signal line function value: UTXD1 */
#define PIO_PA4C_UART1_UTXD1                       (_UL_(1) << 4)

#define PIN_PA6C_UART1_UTXD1                       _L_(6)       /**< UART1 signal: UTXD1 on PA6 mux C*/
#define MUX_PA6C_UART1_UTXD1                       _L_(2)       /**< UART1 signal line function value: UTXD1 */
#define PIO_PA6C_UART1_UTXD1                       (_UL_(1) << 6)

#define PIN_PD26D_UART1_UTXD1                      _L_(122)     /**< UART1 signal: UTXD1 on PD26 mux D*/
#define MUX_PD26D_UART1_UTXD1                      _L_(3)       /**< UART1 signal line function value: UTXD1 */
#define PIO_PD26D_UART1_UTXD1                      (_UL_(1) << 26)

/* ========== PIO definition for UART2 peripheral ========== */
#define PIN_PD25C_UART2_URXD2                      _L_(121)     /**< UART2 signal: URXD2 on PD25 mux C*/
#define MUX_PD25C_UART2_URXD2                      _L_(2)       /**< UART2 signal line function value: URXD2 */
#define PIO_PD25C_UART2_URXD2                      (_UL_(1) << 25)

#define PIN_PD26C_UART2_UTXD2                      _L_(122)     /**< UART2 signal: UTXD2 on PD26 mux C*/
#define MUX_PD26C_UART2_UTXD2                      _L_(2)       /**< UART2 signal line function value: UTXD2 */
#define PIO_PD26C_UART2_UTXD2                      (_UL_(1) << 26)

/* ========== PIO definition for UART3 peripheral ========== */
#define PIN_PD28A_UART3_URXD3                      _L_(124)     /**< UART3 signal: URXD3 on PD28 mux A*/
#define MUX_PD28A_UART3_URXD3                      _L_(0)       /**< UART3 signal line function value: URXD3 */
#define PIO_PD28A_UART3_URXD3                      (_UL_(1) << 28)

#define PIN_PD30A_UART3_UTXD3                      _L_(126)     /**< UART3 signal: UTXD3 on PD30 mux A*/
#define MUX_PD30A_UART3_UTXD3                      _L_(0)       /**< UART3 signal line function value: UTXD3 */
#define PIO_PD30A_UART3_UTXD3                      (_UL_(1) << 30)

#define PIN_PD31B_UART3_UTXD3                      _L_(127)     /**< UART3 signal: UTXD3 on PD31 mux B*/
#define MUX_PD31B_UART3_UTXD3                      _L_(1)       /**< UART3 signal line function value: UTXD3 */
#define PIO_PD31B_UART3_UTXD3                      (_UL_(1) << 31)

/* ========== PIO definition for UART4 peripheral ========== */
#define PIN_PD18C_UART4_URXD4                      _L_(114)     /**< UART4 signal: URXD4 on PD18 mux C*/
#define MUX_PD18C_UART4_URXD4                      _L_(2)       /**< UART4 signal line function value: URXD4 */
#define PIO_PD18C_UART4_URXD4                      (_UL_(1) << 18)

#define PIN_PD3C_UART4_UTXD4                       _L_(99)      /**< UART4 signal: UTXD4 on PD3 mux C*/
#define MUX_PD3C_UART4_UTXD4                       _L_(2)       /**< UART4 signal line function value: UTXD4 */
#define PIO_PD3C_UART4_UTXD4                       (_UL_(1) << 3)

#define PIN_PD19C_UART4_UTXD4                      _L_(115)     /**< UART4 signal: UTXD4 on PD19 mux C*/
#define MUX_PD19C_UART4_UTXD4                      _L_(2)       /**< UART4 signal line function value: UTXD4 */
#define PIO_PD19C_UART4_UTXD4                      (_UL_(1) << 19)

/* ========== PIO definition for USART0 peripheral ========== */
#define PIN_PB2C_USART0_CTS0                       _L_(34)      /**< USART0 signal: CTS0 on PB2 mux C*/
#define MUX_PB2C_USART0_CTS0                       _L_(2)       /**< USART0 signal line function value: CTS0 */
#define PIO_PB2C_USART0_CTS0                       (_UL_(1) << 2)

#define PIN_PD0D_USART0_DCD0                       _L_(96)      /**< USART0 signal: DCD0 on PD0 mux D*/
#define MUX_PD0D_USART0_DCD0                       _L_(3)       /**< USART0 signal line function value: DCD0 */
#define PIO_PD0D_USART0_DCD0                       (_UL_(1) << 0)

#define PIN_PD2D_USART0_DSR0                       _L_(98)      /**< USART0 signal: DSR0 on PD2 mux D*/
#define MUX_PD2D_USART0_DSR0                       _L_(3)       /**< USART0 signal line function value: DSR0 */
#define PIO_PD2D_USART0_DSR0                       (_UL_(1) << 2)

#define PIN_PD1D_USART0_DTR0                       _L_(97)      /**< USART0 signal: DTR0 on PD1 mux D*/
#define MUX_PD1D_USART0_DTR0                       _L_(3)       /**< USART0 signal line function value: DTR0 */
#define PIO_PD1D_USART0_DTR0                       (_UL_(1) << 1)

#define PIN_PD3D_USART0_RI0                        _L_(99)      /**< USART0 signal: RI0 on PD3 mux D*/
#define MUX_PD3D_USART0_RI0                        _L_(3)       /**< USART0 signal line function value: RI0 */
#define PIO_PD3D_USART0_RI0                        (_UL_(1) << 3)

#define PIN_PB3C_USART0_RTS0                       _L_(35)      /**< USART0 signal: RTS0 on PB3 mux C*/
#define MUX_PB3C_USART0_RTS0                       _L_(2)       /**< USART0 signal line function value: RTS0 */
#define PIO_PB3C_USART0_RTS0                       (_UL_(1) << 3)

#define PIN_PB0C_USART0_RXD0                       _L_(32)      /**< USART0 signal: RXD0 on PB0 mux C*/
#define MUX_PB0C_USART0_RXD0                       _L_(2)       /**< USART0 signal line function value: RXD0 */
#define PIO_PB0C_USART0_RXD0                       (_UL_(1) << 0)

#define PIN_PB13C_USART0_SCK0                      _L_(45)      /**< USART0 signal: SCK0 on PB13 mux C*/
#define MUX_PB13C_USART0_SCK0                      _L_(2)       /**< USART0 signal line function value: SCK0 */
#define PIO_PB13C_USART0_SCK0                      (_UL_(1) << 13)

#define PIN_PB1C_USART0_TXD0                       _L_(33)      /**< USART0 signal: TXD0 on PB1 mux C*/
#define MUX_PB1C_USART0_TXD0                       _L_(2)       /**< USART0 signal line function value: TXD0 */
#define PIO_PB1C_USART0_TXD0                       (_UL_(1) << 1)

/* ========== PIO definition for USART1 peripheral ========== */
#define PIN_PA25A_USART1_CTS1                      _L_(25)      /**< USART1 signal: CTS1 on PA25 mux A*/
#define MUX_PA25A_USART1_CTS1                      _L_(0)       /**< USART1 signal line function value: CTS1 */
#define PIO_PA25A_USART1_CTS1                      (_UL_(1) << 25)

#define PIN_PA26A_USART1_DCD1                      _L_(26)      /**< USART1 signal: DCD1 on PA26 mux A*/
#define MUX_PA26A_USART1_DCD1                      _L_(0)       /**< USART1 signal line function value: DCD1 */
#define PIO_PA26A_USART1_DCD1                      (_UL_(1) << 26)

#define PIN_PA28A_USART1_DSR1                      _L_(28)      /**< USART1 signal: DSR1 on PA28 mux A*/
#define MUX_PA28A_USART1_DSR1                      _L_(0)       /**< USART1 signal line function value: DSR1 */
#define PIO_PA28A_USART1_DSR1                      (_UL_(1) << 28)

#define PIN_PA27A_USART1_DTR1                      _L_(27)      /**< USART1 signal: DTR1 on PA27 mux A*/
#define MUX_PA27A_USART1_DTR1                      _L_(0)       /**< USART1 signal line function value: DTR1 */
#define PIO_PA27A_USART1_DTR1                      (_UL_(1) << 27)

#define PIN_PA3B_USART1_LONCOL1                    _L_(3)       /**< USART1 signal: LONCOL1 on PA3 mux B*/
#define MUX_PA3B_USART1_LONCOL1                    _L_(1)       /**< USART1 signal line function value: LONCOL1 */
#define PIO_PA3B_USART1_LONCOL1                    (_UL_(1) << 3)

#define PIN_PA29A_USART1_RI1                       _L_(29)      /**< USART1 signal: RI1 on PA29 mux A*/
#define MUX_PA29A_USART1_RI1                       _L_(0)       /**< USART1 signal line function value: RI1 */
#define PIO_PA29A_USART1_RI1                       (_UL_(1) << 29)

#define PIN_PA24A_USART1_RTS1                      _L_(24)      /**< USART1 signal: RTS1 on PA24 mux A*/
#define MUX_PA24A_USART1_RTS1                      _L_(0)       /**< USART1 signal line function value: RTS1 */
#define PIO_PA24A_USART1_RTS1                      (_UL_(1) << 24)

#define PIN_PA21A_USART1_RXD1                      _L_(21)      /**< USART1 signal: RXD1 on PA21 mux A*/
#define MUX_PA21A_USART1_RXD1                      _L_(0)       /**< USART1 signal line function value: RXD1 */
#define PIO_PA21A_USART1_RXD1                      (_UL_(1) << 21)

#define PIN_PA23A_USART1_SCK1                      _L_(23)      /**< USART1 signal: SCK1 on PA23 mux A*/
#define MUX_PA23A_USART1_SCK1                      _L_(0)       /**< USART1 signal line function value: SCK1 */
#define PIO_PA23A_USART1_SCK1                      (_UL_(1) << 23)

#define PIN_PB4D_USART1_TXD1                       _L_(36)      /**< USART1 signal: TXD1 on PB4 mux D*/
#define MUX_PB4D_USART1_TXD1                       _L_(3)       /**< USART1 signal line function value: TXD1 */
#define PIO_PB4D_USART1_TXD1                       (_UL_(1) << 4)

/* ========== PIO definition for USART2 peripheral ========== */
#define PIN_PD19B_USART2_CTS2                      _L_(115)     /**< USART2 signal: CTS2 on PD19 mux B*/
#define MUX_PD19B_USART2_CTS2                      _L_(1)       /**< USART2 signal line function value: CTS2 */
#define PIO_PD19B_USART2_CTS2                      (_UL_(1) << 19)

#define PIN_PD4D_USART2_DCD2                       _L_(100)     /**< USART2 signal: DCD2 on PD4 mux D*/
#define MUX_PD4D_USART2_DCD2                       _L_(3)       /**< USART2 signal line function value: DCD2 */
#define PIO_PD4D_USART2_DCD2                       (_UL_(1) << 4)

#define PIN_PD6D_USART2_DSR2                       _L_(102)     /**< USART2 signal: DSR2 on PD6 mux D*/
#define MUX_PD6D_USART2_DSR2                       _L_(3)       /**< USART2 signal line function value: DSR2 */
#define PIO_PD6D_USART2_DSR2                       (_UL_(1) << 6)

#define PIN_PD5D_USART2_DTR2                       _L_(101)     /**< USART2 signal: DTR2 on PD5 mux D*/
#define MUX_PD5D_USART2_DTR2                       _L_(3)       /**< USART2 signal line function value: DTR2 */
#define PIO_PD5D_USART2_DTR2                       (_UL_(1) << 5)

#define PIN_PD7D_USART2_RI2                        _L_(103)     /**< USART2 signal: RI2 on PD7 mux D*/
#define MUX_PD7D_USART2_RI2                        _L_(3)       /**< USART2 signal line function value: RI2 */
#define PIO_PD7D_USART2_RI2                        (_UL_(1) << 7)

#define PIN_PD18B_USART2_RTS2                      _L_(114)     /**< USART2 signal: RTS2 on PD18 mux B*/
#define MUX_PD18B_USART2_RTS2                      _L_(1)       /**< USART2 signal line function value: RTS2 */
#define PIO_PD18B_USART2_RTS2                      (_UL_(1) << 18)

#define PIN_PD15B_USART2_RXD2                      _L_(111)     /**< USART2 signal: RXD2 on PD15 mux B*/
#define MUX_PD15B_USART2_RXD2                      _L_(1)       /**< USART2 signal line function value: RXD2 */
#define PIO_PD15B_USART2_RXD2                      (_UL_(1) << 15)

#define PIN_PD17B_USART2_SCK2                      _L_(113)     /**< USART2 signal: SCK2 on PD17 mux B*/
#define MUX_PD17B_USART2_SCK2                      _L_(1)       /**< USART2 signal line function value: SCK2 */
#define PIO_PD17B_USART2_SCK2                      (_UL_(1) << 17)

#define PIN_PD16B_USART2_TXD2                      _L_(112)     /**< USART2 signal: TXD2 on PD16 mux B*/
#define MUX_PD16B_USART2_TXD2                      _L_(1)       /**< USART2 signal line function value: TXD2 */
#define PIO_PD16B_USART2_TXD2                      (_UL_(1) << 16)

/* ========== PIO definition for EBI peripheral ========== */
#define PIN_PC18A_EBI_A0                           _L_(82)      /**< EBI signal: A0 on PC18 mux A*/
#define MUX_PC18A_EBI_A0                           _L_(0)       /**< EBI signal line function value: A0 */
#define PIO_PC18A_EBI_A0                           (_UL_(1) << 18)

#define PIN_PC18A_EBI_NBS0                         _L_(82)      /**< EBI signal: NBS0 on PC18 mux A*/
#define MUX_PC18A_EBI_NBS0                         _L_(0)       /**< EBI signal line function value: NBS0 */
#define PIO_PC18A_EBI_NBS0                         (_UL_(1) << 18)

#define PIN_PC19A_EBI_A1                           _L_(83)      /**< EBI signal: A1 on PC19 mux A*/
#define MUX_PC19A_EBI_A1                           _L_(0)       /**< EBI signal line function value: A1 */
#define PIO_PC19A_EBI_A1                           (_UL_(1) << 19)

#define PIN_PC20A_EBI_A2                           _L_(84)      /**< EBI signal: A2 on PC20 mux A*/
#define MUX_PC20A_EBI_A2                           _L_(0)       /**< EBI signal line function value: A2 */
#define PIO_PC20A_EBI_A2                           (_UL_(1) << 20)

#define PIN_PC21A_EBI_A3                           _L_(85)      /**< EBI signal: A3 on PC21 mux A*/
#define MUX_PC21A_EBI_A3                           _L_(0)       /**< EBI signal line function value: A3 */
#define PIO_PC21A_EBI_A3                           (_UL_(1) << 21)

#define PIN_PC22A_EBI_A4                           _L_(86)      /**< EBI signal: A4 on PC22 mux A*/
#define MUX_PC22A_EBI_A4                           _L_(0)       /**< EBI signal line function value: A4 */
#define PIO_PC22A_EBI_A4                           (_UL_(1) << 22)

#define PIN_PC23A_EBI_A5                           _L_(87)      /**< EBI signal: A5 on PC23 mux A*/
#define MUX_PC23A_EBI_A5                           _L_(0)       /**< EBI signal line function value: A5 */
#define PIO_PC23A_EBI_A5                           (_UL_(1) << 23)

#define PIN_PC24A_EBI_A6                           _L_(88)      /**< EBI signal: A6 on PC24 mux A*/
#define MUX_PC24A_EBI_A6                           _L_(0)       /**< EBI signal line function value: A6 */
#define PIO_PC24A_EBI_A6                           (_UL_(1) << 24)

#define PIN_PC25A_EBI_A7                           _L_(89)      /**< EBI signal: A7 on PC25 mux A*/
#define MUX_PC25A_EBI_A7                           _L_(0)       /**< EBI signal line function value: A7 */
#define PIO_PC25A_EBI_A7                           (_UL_(1) << 25)

#define PIN_PC26A_EBI_A8                           _L_(90)      /**< EBI signal: A8 on PC26 mux A*/
#define MUX_PC26A_EBI_A8                           _L_(0)       /**< EBI signal line function value: A8 */
#define PIO_PC26A_EBI_A8                           (_UL_(1) << 26)

#define PIN_PC27A_EBI_A9                           _L_(91)      /**< EBI signal: A9 on PC27 mux A*/
#define MUX_PC27A_EBI_A9                           _L_(0)       /**< EBI signal line function value: A9 */
#define PIO_PC27A_EBI_A9                           (_UL_(1) << 27)

#define PIN_PC28A_EBI_A10                          _L_(92)      /**< EBI signal: A10 on PC28 mux A*/
#define MUX_PC28A_EBI_A10                          _L_(0)       /**< EBI signal line function value: A10 */
#define PIO_PC28A_EBI_A10                          (_UL_(1) << 28)

#define PIN_PC29A_EBI_A11                          _L_(93)      /**< EBI signal: A11 on PC29 mux A*/
#define MUX_PC29A_EBI_A11                          _L_(0)       /**< EBI signal line function value: A11 */
#define PIO_PC29A_EBI_A11                          (_UL_(1) << 29)

#define PIN_PC30A_EBI_A12                          _L_(94)      /**< EBI signal: A12 on PC30 mux A*/
#define MUX_PC30A_EBI_A12                          _L_(0)       /**< EBI signal line function value: A12 */
#define PIO_PC30A_EBI_A12                          (_UL_(1) << 30)

#define PIN_PC31A_EBI_A13                          _L_(95)      /**< EBI signal: A13 on PC31 mux A*/
#define MUX_PC31A_EBI_A13                          _L_(0)       /**< EBI signal line function value: A13 */
#define PIO_PC31A_EBI_A13                          (_UL_(1) << 31)

#define PIN_PA18C_EBI_A14                          _L_(18)      /**< EBI signal: A14 on PA18 mux C*/
#define MUX_PA18C_EBI_A14                          _L_(2)       /**< EBI signal line function value: A14 */
#define PIO_PA18C_EBI_A14                          (_UL_(1) << 18)

#define PIN_PA19C_EBI_A15                          _L_(19)      /**< EBI signal: A15 on PA19 mux C*/
#define MUX_PA19C_EBI_A15                          _L_(2)       /**< EBI signal line function value: A15 */
#define PIO_PA19C_EBI_A15                          (_UL_(1) << 19)

#define PIN_PA20C_EBI_A16                          _L_(20)      /**< EBI signal: A16 on PA20 mux C*/
#define MUX_PA20C_EBI_A16                          _L_(2)       /**< EBI signal line function value: A16 */
#define PIO_PA20C_EBI_A16                          (_UL_(1) << 20)

#define PIN_PA20C_EBI_BA0                          _L_(20)      /**< EBI signal: BA0 on PA20 mux C*/
#define MUX_PA20C_EBI_BA0                          _L_(2)       /**< EBI signal line function value: BA0 */
#define PIO_PA20C_EBI_BA0                          (_UL_(1) << 20)

#define PIN_PA0C_EBI_A17                           _L_(0)       /**< EBI signal: A17 on PA0 mux C*/
#define MUX_PA0C_EBI_A17                           _L_(2)       /**< EBI signal line function value: A17 */
#define PIO_PA0C_EBI_A17                           (_UL_(1) << 0)

#define PIN_PA0C_EBI_BA1                           _L_(0)       /**< EBI signal: BA1 on PA0 mux C*/
#define MUX_PA0C_EBI_BA1                           _L_(2)       /**< EBI signal line function value: BA1 */
#define PIO_PA0C_EBI_BA1                           (_UL_(1) << 0)

#define PIN_PA1C_EBI_A18                           _L_(1)       /**< EBI signal: A18 on PA1 mux C*/
#define MUX_PA1C_EBI_A18                           _L_(2)       /**< EBI signal line function value: A18 */
#define PIO_PA1C_EBI_A18                           (_UL_(1) << 1)

#define PIN_PA23C_EBI_A19                          _L_(23)      /**< EBI signal: A19 on PA23 mux C*/
#define MUX_PA23C_EBI_A19                          _L_(2)       /**< EBI signal line function value: A19 */
#define PIO_PA23C_EBI_A19                          (_UL_(1) << 23)

#define PIN_PA24C_EBI_A20                          _L_(24)      /**< EBI signal: A20 on PA24 mux C*/
#define MUX_PA24C_EBI_A20                          _L_(2)       /**< EBI signal line function value: A20 */
#define PIO_PA24C_EBI_A20                          (_UL_(1) << 24)

#define PIN_PC16A_EBI_A21                          _L_(80)      /**< EBI signal: A21 on PC16 mux A*/
#define MUX_PC16A_EBI_A21                          _L_(0)       /**< EBI signal line function value: A21 */
#define PIO_PC16A_EBI_A21                          (_UL_(1) << 16)

#define PIN_PC16A_EBI_NANDALE                      _L_(80)      /**< EBI signal: NANDALE on PC16 mux A*/
#define MUX_PC16A_EBI_NANDALE                      _L_(0)       /**< EBI signal line function value: NANDALE */
#define PIO_PC16A_EBI_NANDALE                      (_UL_(1) << 16)

#define PIN_PC17A_EBI_A22                          _L_(81)      /**< EBI signal: A22 on PC17 mux A*/
#define MUX_PC17A_EBI_A22                          _L_(0)       /**< EBI signal line function value: A22 */
#define PIO_PC17A_EBI_A22                          (_UL_(1) << 17)

#define PIN_PC17A_EBI_NANDCLE                      _L_(81)      /**< EBI signal: NANDCLE on PC17 mux A*/
#define MUX_PC17A_EBI_NANDCLE                      _L_(0)       /**< EBI signal line function value: NANDCLE */
#define PIO_PC17A_EBI_NANDCLE                      (_UL_(1) << 17)

#define PIN_PA25C_EBI_A23                          _L_(25)      /**< EBI signal: A23 on PA25 mux C*/
#define MUX_PA25C_EBI_A23                          _L_(2)       /**< EBI signal line function value: A23 */
#define PIO_PA25C_EBI_A23                          (_UL_(1) << 25)

#define PIN_PD17C_EBI_CAS                          _L_(113)     /**< EBI signal: CAS on PD17 mux C*/
#define MUX_PD17C_EBI_CAS                          _L_(2)       /**< EBI signal line function value: CAS */
#define PIO_PD17C_EBI_CAS                          (_UL_(1) << 17)

#define PIN_PC0A_EBI_D0                            _L_(64)      /**< EBI signal: D0 on PC0 mux A*/
#define MUX_PC0A_EBI_D0                            _L_(0)       /**< EBI signal line function value: D0 */
#define PIO_PC0A_EBI_D0                            (_UL_(1) << 0)

#define PIN_PC1A_EBI_D1                            _L_(65)      /**< EBI signal: D1 on PC1 mux A*/
#define MUX_PC1A_EBI_D1                            _L_(0)       /**< EBI signal line function value: D1 */
#define PIO_PC1A_EBI_D1                            (_UL_(1) << 1)

#define PIN_PC2A_EBI_D2                            _L_(66)      /**< EBI signal: D2 on PC2 mux A*/
#define MUX_PC2A_EBI_D2                            _L_(0)       /**< EBI signal line function value: D2 */
#define PIO_PC2A_EBI_D2                            (_UL_(1) << 2)

#define PIN_PC3A_EBI_D3                            _L_(67)      /**< EBI signal: D3 on PC3 mux A*/
#define MUX_PC3A_EBI_D3                            _L_(0)       /**< EBI signal line function value: D3 */
#define PIO_PC3A_EBI_D3                            (_UL_(1) << 3)

#define PIN_PC4A_EBI_D4                            _L_(68)      /**< EBI signal: D4 on PC4 mux A*/
#define MUX_PC4A_EBI_D4                            _L_(0)       /**< EBI signal line function value: D4 */
#define PIO_PC4A_EBI_D4                            (_UL_(1) << 4)

#define PIN_PC5A_EBI_D5                            _L_(69)      /**< EBI signal: D5 on PC5 mux A*/
#define MUX_PC5A_EBI_D5                            _L_(0)       /**< EBI signal line function value: D5 */
#define PIO_PC5A_EBI_D5                            (_UL_(1) << 5)

#define PIN_PC6A_EBI_D6                            _L_(70)      /**< EBI signal: D6 on PC6 mux A*/
#define MUX_PC6A_EBI_D6                            _L_(0)       /**< EBI signal line function value: D6 */
#define PIO_PC6A_EBI_D6                            (_UL_(1) << 6)

#define PIN_PC7A_EBI_D7                            _L_(71)      /**< EBI signal: D7 on PC7 mux A*/
#define MUX_PC7A_EBI_D7                            _L_(0)       /**< EBI signal line function value: D7 */
#define PIO_PC7A_EBI_D7                            (_UL_(1) << 7)

#define PIN_PE0A_EBI_D8                            _L_(128)     /**< EBI signal: D8 on PE0 mux A*/
#define MUX_PE0A_EBI_D8                            _L_(0)       /**< EBI signal line function value: D8 */
#define PIO_PE0A_EBI_D8                            (_UL_(1) << 0)

#define PIN_PE1A_EBI_D9                            _L_(129)     /**< EBI signal: D9 on PE1 mux A*/
#define MUX_PE1A_EBI_D9                            _L_(0)       /**< EBI signal line function value: D9 */
#define PIO_PE1A_EBI_D9                            (_UL_(1) << 1)

#define PIN_PE2A_EBI_D10                           _L_(130)     /**< EBI signal: D10 on PE2 mux A*/
#define MUX_PE2A_EBI_D10                           _L_(0)       /**< EBI signal line function value: D10 */
#define PIO_PE2A_EBI_D10                           (_UL_(1) << 2)

#define PIN_PE3A_EBI_D11                           _L_(131)     /**< EBI signal: D11 on PE3 mux A*/
#define MUX_PE3A_EBI_D11                           _L_(0)       /**< EBI signal line function value: D11 */
#define PIO_PE3A_EBI_D11                           (_UL_(1) << 3)

#define PIN_PE4A_EBI_D12                           _L_(132)     /**< EBI signal: D12 on PE4 mux A*/
#define MUX_PE4A_EBI_D12                           _L_(0)       /**< EBI signal line function value: D12 */
#define PIO_PE4A_EBI_D12                           (_UL_(1) << 4)

#define PIN_PE5A_EBI_D13                           _L_(133)     /**< EBI signal: D13 on PE5 mux A*/
#define MUX_PE5A_EBI_D13                           _L_(0)       /**< EBI signal line function value: D13 */
#define PIO_PE5A_EBI_D13                           (_UL_(1) << 5)

#define PIN_PA15A_EBI_D14                          _L_(15)      /**< EBI signal: D14 on PA15 mux A*/
#define MUX_PA15A_EBI_D14                          _L_(0)       /**< EBI signal line function value: D14 */
#define PIO_PA15A_EBI_D14                          (_UL_(1) << 15)

#define PIN_PA16A_EBI_D15                          _L_(16)      /**< EBI signal: D15 on PA16 mux A*/
#define MUX_PA16A_EBI_D15                          _L_(0)       /**< EBI signal line function value: D15 */
#define PIO_PA16A_EBI_D15                          (_UL_(1) << 16)

#define PIN_PC9A_EBI_NANDOE                        _L_(73)      /**< EBI signal: NANDOE on PC9 mux A*/
#define MUX_PC9A_EBI_NANDOE                        _L_(0)       /**< EBI signal line function value: NANDOE */
#define PIO_PC9A_EBI_NANDOE                        (_UL_(1) << 9)

#define PIN_PC10A_EBI_NANDWE                       _L_(74)      /**< EBI signal: NANDWE on PC10 mux A*/
#define MUX_PC10A_EBI_NANDWE                       _L_(0)       /**< EBI signal line function value: NANDWE */
#define PIO_PC10A_EBI_NANDWE                       (_UL_(1) << 10)

#define PIN_PC14A_EBI_NCS0                         _L_(78)      /**< EBI signal: NCS0 on PC14 mux A*/
#define MUX_PC14A_EBI_NCS0                         _L_(0)       /**< EBI signal line function value: NCS0 */
#define PIO_PC14A_EBI_NCS0                         (_UL_(1) << 14)

#define PIN_PC15A_EBI_NCS1                         _L_(79)      /**< EBI signal: NCS1 on PC15 mux A*/
#define MUX_PC15A_EBI_NCS1                         _L_(0)       /**< EBI signal line function value: NCS1 */
#define PIO_PC15A_EBI_NCS1                         (_UL_(1) << 15)

#define PIN_PC15A_EBI_SDCS                         _L_(79)      /**< EBI signal: SDCS on PC15 mux A*/
#define MUX_PC15A_EBI_SDCS                         _L_(0)       /**< EBI signal line function value: SDCS */
#define PIO_PC15A_EBI_SDCS                         (_UL_(1) << 15)

#define PIN_PD18A_EBI_NCS1                         _L_(114)     /**< EBI signal: NCS1 on PD18 mux A*/
#define MUX_PD18A_EBI_NCS1                         _L_(0)       /**< EBI signal line function value: NCS1 */
#define PIO_PD18A_EBI_NCS1                         (_UL_(1) << 18)

#define PIN_PD18A_EBI_SDCS                         _L_(114)     /**< EBI signal: SDCS on PD18 mux A*/
#define MUX_PD18A_EBI_SDCS                         _L_(0)       /**< EBI signal line function value: SDCS */
#define PIO_PD18A_EBI_SDCS                         (_UL_(1) << 18)

#define PIN_PA22C_EBI_NCS2                         _L_(22)      /**< EBI signal: NCS2 on PA22 mux C*/
#define MUX_PA22C_EBI_NCS2                         _L_(2)       /**< EBI signal line function value: NCS2 */
#define PIO_PA22C_EBI_NCS2                         (_UL_(1) << 22)

#define PIN_PC12A_EBI_NCS3                         _L_(76)      /**< EBI signal: NCS3 on PC12 mux A*/
#define MUX_PC12A_EBI_NCS3                         _L_(0)       /**< EBI signal line function value: NCS3 */
#define PIO_PC12A_EBI_NCS3                         (_UL_(1) << 12)

#define PIN_PD19A_EBI_NCS3                         _L_(115)     /**< EBI signal: NCS3 on PD19 mux A*/
#define MUX_PD19A_EBI_NCS3                         _L_(0)       /**< EBI signal line function value: NCS3 */
#define PIO_PD19A_EBI_NCS3                         (_UL_(1) << 19)

#define PIN_PC11A_EBI_NRD                          _L_(75)      /**< EBI signal: NRD on PC11 mux A*/
#define MUX_PC11A_EBI_NRD                          _L_(0)       /**< EBI signal line function value: NRD */
#define PIO_PC11A_EBI_NRD                          (_UL_(1) << 11)

#define PIN_PC13A_EBI_NWAIT                        _L_(77)      /**< EBI signal: NWAIT on PC13 mux A*/
#define MUX_PC13A_EBI_NWAIT                        _L_(0)       /**< EBI signal line function value: NWAIT */
#define PIO_PC13A_EBI_NWAIT                        (_UL_(1) << 13)

#define PIN_PC8A_EBI_NWR0                          _L_(72)      /**< EBI signal: NWR0 on PC8 mux A*/
#define MUX_PC8A_EBI_NWR0                          _L_(0)       /**< EBI signal line function value: NWR0 */
#define PIO_PC8A_EBI_NWR0                          (_UL_(1) << 8)

#define PIN_PC8A_EBI_NWE                           _L_(72)      /**< EBI signal: NWE on PC8 mux A*/
#define MUX_PC8A_EBI_NWE                           _L_(0)       /**< EBI signal line function value: NWE */
#define PIO_PC8A_EBI_NWE                           (_UL_(1) << 8)

#define PIN_PD15C_EBI_NWR1                         _L_(111)     /**< EBI signal: NWR1 on PD15 mux C*/
#define MUX_PD15C_EBI_NWR1                         _L_(2)       /**< EBI signal line function value: NWR1 */
#define PIO_PD15C_EBI_NWR1                         (_UL_(1) << 15)

#define PIN_PD15C_EBI_NBS1                         _L_(111)     /**< EBI signal: NBS1 on PD15 mux C*/
#define MUX_PD15C_EBI_NBS1                         _L_(2)       /**< EBI signal line function value: NBS1 */
#define PIO_PD15C_EBI_NBS1                         (_UL_(1) << 15)

#define PIN_PD16C_EBI_RAS                          _L_(112)     /**< EBI signal: RAS on PD16 mux C*/
#define MUX_PD16C_EBI_RAS                          _L_(2)       /**< EBI signal line function value: RAS */
#define PIO_PD16C_EBI_RAS                          (_UL_(1) << 16)

#define PIN_PC13C_EBI_SDA10                        _L_(77)      /**< EBI signal: SDA10 on PC13 mux C*/
#define MUX_PC13C_EBI_SDA10                        _L_(2)       /**< EBI signal line function value: SDA10 */
#define PIO_PC13C_EBI_SDA10                        (_UL_(1) << 13)

#define PIN_PD13C_EBI_SDA10                        _L_(109)     /**< EBI signal: SDA10 on PD13 mux C*/
#define MUX_PD13C_EBI_SDA10                        _L_(2)       /**< EBI signal line function value: SDA10 */
#define PIO_PD13C_EBI_SDA10                        (_UL_(1) << 13)

#define PIN_PD23C_EBI_SDCK                         _L_(119)     /**< EBI signal: SDCK on PD23 mux C*/
#define MUX_PD23C_EBI_SDCK                         _L_(2)       /**< EBI signal line function value: SDCK */
#define PIO_PD23C_EBI_SDCK                         (_UL_(1) << 23)

#define PIN_PD14C_EBI_SDCKE                        _L_(110)     /**< EBI signal: SDCKE on PD14 mux C*/
#define MUX_PD14C_EBI_SDCKE                        _L_(2)       /**< EBI signal line function value: SDCKE */
#define PIO_PD14C_EBI_SDCKE                        (_UL_(1) << 14)

#define PIN_PD29C_EBI_SDWE                         _L_(125)     /**< EBI signal: SDWE on PD29 mux C*/
#define MUX_PD29C_EBI_SDWE                         _L_(2)       /**< EBI signal line function value: SDWE */
#define PIO_PD29C_EBI_SDWE                         (_UL_(1) << 29)
/** @}  end of Peripheral Pio Definitions */

/* ************************************************************************** */
/*   MEMORY MAPPING DEFINITIONS FOR SAME70Q21*/
/* ************************************************************************** */

#define PERIPHERALS_SIZE         _U_(0x20000000)       /* 524288kB Memory segment type: io */
#define SYSTEM_SIZE              _U_(0x10000000)       /* 262144kB Memory segment type: io */
#define QSPIMEM_SIZE             _U_(0x20000000)       /* 524288kB Memory segment type: other */
#define AXIMX_SIZE               _U_(0x00100000)       /* 1024kB Memory segment type: other */
#define ITCM_SIZE                _U_(0x00200000)       /* 2048kB Memory segment type: other */
#define IFLASH_SIZE              _U_(0x00200000)       /* 2048kB Memory segment type: flash */
#define IFLASH_PAGE_SIZE         _U_(       512)
#define IFLASH_NB_OF_PAGES       _U_(      4096)

#define IROM_SIZE                _U_(0x00004000)       /*   16kB Memory segment type: rom */
#define DTCM_SIZE                _U_(0x00020000)       /*  128kB Memory segment type: other */
#define IRAM_SIZE                _U_(0x00060000)       /*  384kB Memory segment type: ram */
#define EBI_CS0_SIZE             _U_(0x01000000)       /* 16384kB Memory segment type: other */
#define EBI_CS1_SIZE             _U_(0x01000000)       /* 16384kB Memory segment type: other */
#define EBI_CS2_SIZE             _U_(0x01000000)       /* 16384kB Memory segment type: other */
#define EBI_CS3_SIZE             _U_(0x01000000)       /* 16384kB Memory segment type: other */
#define SDRAM_CS_SIZE            _U_(0x10000000)       /* 262144kB Memory segment type: other */

#define PERIPHERALS_ADDR         _U_(0x40000000)       /**< PERIPHERALS base address (type: io)*/
#define SYSTEM_ADDR              _U_(0xe0000000)       /**< SYSTEM base address (type: io)*/
#define QSPIMEM_ADDR             _U_(0x80000000)       /**< QSPIMEM base address (type: other)*/
#define AXIMX_ADDR               _U_(0xa0000000)       /**< AXIMX base address (type: other)*/
#define ITCM_ADDR                _U_(0x00000000)       /**< ITCM base address (type: other)*/
#define IFLASH_ADDR              _U_(0x00400000)       /**< IFLASH base address (type: flash)*/
#define IROM_ADDR                _U_(0x00800000)       /**< IROM base address (type: rom)*/
#define DTCM_ADDR                _U_(0x20000000)       /**< DTCM base address (type: other)*/
#define IRAM_ADDR                _U_(0x20400000)       /**< IRAM base address (type: ram)*/
#define EBI_CS0_ADDR             _U_(0x60000000)       /**< EBI_CS0 base address (type: other)*/
#define EBI_CS1_ADDR             _U_(0x61000000)       /**< EBI_CS1 base address (type: other)*/
#define EBI_CS2_ADDR             _U_(0x62000000)       /**< EBI_CS2 base address (type: other)*/
#define EBI_CS3_ADDR             _U_(0x63000000)       /**< EBI_CS3 base address (type: other)*/
#define SDRAM_CS_ADDR            _U_(0x70000000)       /**< SDRAM_CS base address (type: other)*/

/* ************************************************************************** */
/**  DEVICE SIGNATURES FOR SAME70Q21 */
/* ************************************************************************** */
#define JTAGID                   _UL_(0X05B3D03F)
#define CHIP_JTAGID              _UL_(0X05B3D03F)
#define CHIP_CIDR                _UL_(0XA1020E00)
#define CHIP_EXID                _UL_(0X00000002)

/* ************************************************************************** */
/**  ELECTRICAL DEFINITIONS FOR SAME70Q21 */
/* ************************************************************************** */
#define CHIP_FREQ_SLCK_RC_MIN          _UL_(20000)     
#define CHIP_FREQ_SLCK_RC              _UL_(32000)     /**< \brief Typical Slow Clock Internal RC frequency*/
#define CHIP_FREQ_SLCK_RC_MAX          _UL_(44000)     
#define CHIP_FREQ_MAINCK_RC_4MHZ       _UL_(4000000)   
#define CHIP_FREQ_MAINCK_RC_8MHZ       _UL_(8000000)   
#define CHIP_FREQ_MAINCK_RC_12MHZ      _UL_(12000000)  
#define CHIP_FREQ_CPU_MAX              _UL_(300000000) 
#define CHIP_FREQ_XTAL_32K             _UL_(32768)     
#define CHIP_FREQ_XTAL_12M             _UL_(12000000)  
#define CHIP_FREQ_FWS_0                _UL_(23000000)  /**< \brief Maximum operating frequency when FWS is 0*/
#define CHIP_FREQ_FWS_1                _UL_(46000000)  /**< \brief Maximum operating frequency when FWS is 1*/
#define CHIP_FREQ_FWS_2                _UL_(69000000)  /**< \brief Maximum operating frequency when FWS is 2*/
#define CHIP_FREQ_FWS_3                _UL_(92000000)  /**< \brief Maximum operating frequency when FWS is 3*/
#define CHIP_FREQ_FWS_4                _UL_(115000000) /**< \brief Maximum operating frequency when FWS is 4*/
#define CHIP_FREQ_FWS_5                _UL_(138000000) /**< \brief Maximum operating frequency when FWS is 5*/
#define CHIP_FREQ_FWS_6                _UL_(150000000) /**< \brief Maximum operating frequency when FWS is 6*/
#define CHIP_FREQ_FWS_NUMBER           _UL_(7)         /**< \brief Number of FWS ranges*/

#ifdef __cplusplus
}
#endif

/** @}  end of SAME70Q21 definitions */


#endif /* _SAME70Q21_H_ */
