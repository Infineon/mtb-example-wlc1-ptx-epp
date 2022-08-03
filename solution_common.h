/***************************************************************************//**
* \file solution_common.h
* \version 1.00
*
* Header file of solution layer common types.
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _SOLUTION_COMMON_H_
#define _SOLUTION_COMMON_H_

#include <stdint.h>
#include <stdbool.h>
#include "cy_pdstack_dpm.h"
#include "cy_qistack_pm.h"

/*******************************************************************************
*                              Type Definitions
*******************************************************************************/

/** Buck-boost USBPD driver index */
#define CY_SOLN_BB_INDEX			            (0u)
#define CY_SOLN_INV_INDEX			            (1u)

/** Inverter USBPD driver index */
#define CY_SOLN_INV_INDEX			            (1u)

#define CY_QI_TIMER_STATUS_LED_TIME_ID          (CY_SOLN_TIMER_0_TIME_ID)
/** LED Status timeout in mS */
#define CY_QI_TIMEOUT_LED_POWER_ON              (200u)
#define CY_QI_TIMEOUT_LED_OBJ_NO_ASK            (400u)
#define CY_QI_TIMEOUT_LED_RX_EPT                (250u)

/** Fault polling task timer */
#define CY_SOLN_TIMER_FAULT_POLL                (CY_SOLN_TIMER_1_TIME_ID)
#define CY_SOLN_TIMER_FAULT_POLL_TIME_MS        (500u)

/** Fault retry timer */
#define CY_SOLN_TIMER_FAULT_RETRY               (CY_SOLN_TIMER_2_TIME_ID)
#define CY_SOLN_TIMER_FAULT_RETRY_TIME_MS       (50u)

/** UVP force recovery timer */
#define CY_SOLN_TIMER_FAULT_UVP_FORCE_RECOVER          (CY_SOLN_TIMER_3_TIME_ID)
#define CY_SOLN_TIMER_FAULT_UVP_FORCE_RECOVER_TIME_MS  (1000u)

/** 500ms timer */
#define CY_SOLN_TIMER_AUTOMATION_LOGS           (CY_SOLN_TIMER_4_TIME_ID)
#define CY_SOLN_TIMER_AUTO_LOGS_TIME_MS         (500u)

/** VBRG transition timers */
#define CY_SOLN_TIMER_VBRG_TIMEOUT_ID           (CY_SOLN_TIMER_9_TIME_ID)
#define CY_SOLN_TIMER_VBRG_TIMEOUT              (250u)

/** VIN Hysteresis thresholds */
#define CY_SOLN_VIN_LVL_VSAFE_9V_ENTRY          (6000u)
#define CY_SOLN_VIN_LVL_VSAFE_9V_EXIT           (7000u)
#define CY_SOLN_VIN_LVL_VSAFE_15V_ENTRY         (12000u)
#define CY_SOLN_VIN_LVL_VSAFE_15V_EXIT          (13000u)


/**
 * @brief Enumeration to hold the VIN transition states.
 */
typedef enum 
{
    CY_SOLN_VIN_NO_TRANSITION = 0xFF,
    CY_SOLN_VIN_DOWN_TRANSITION = 0,
    CY_SOLN_VIN_UP_TRANSITION = 1u
} cy_en_soln_vin_trans_t;

/**
 * @brief Enumeration to hold the ADFT list.
 */
typedef enum 
{
    ADFT_NONE = 0,
    ADFT_P0_VOUT_MON,
    ADFT_P1_VOUT_MON,
    ADFT_P0_VOUT_CBL,
    ADFT_P1_VOUT_CBL,
    ADFT_P0_VOUT_CC,
    ADFT_P1_VOUT_CC,
} cy_en_soln_adft_t;

/**
 * @brief Enumeration to hold the faults list.
 */
typedef enum 
{
    CY_SOLN_FAULT_VIN_OVP = 0,
    CY_SOLN_FAULT_VIN_UVP,
    CY_SOLN_FAULT_VBRG_UVP,
    CY_SOLN_FAULT_VBRG_OVP,
    CY_SOLN_FAULT_VBRG_OCP,
    CY_SOLN_FAULT_VBRG_SCP,
    CY_SOLN_FAULT_BB_ILIM,
    CY_SOLN_FAULT_VREG_INRUSH,
    CY_SOLN_FAULT_VDDD_BOD,
    CY_SOLN_FAULT_OTP,
    CY_SOLN_FAULT_CC_OVP,
    CY_SOLN_FAULT_MAX
} cy_en_soln_fault_t;

/**
 * @brief Enumeration to hold the fault state.
 */
typedef enum 
{
    CY_SOLN_FAULT_NONE,
    CY_SOLN_FAULT_QI_STOP_RETRY,
    CY_SOLN_FAULT_QI_STOP_NO_RETRY,
    CY_SOLN_FAULT_QI_RECOVER,
    CY_SOLN_FAULT_PD_RECOVER,
    CY_SOLN_FAULT_DEVICE_RESET
} cy_en_soln_fault_state_t;

/**
 * @brief Enumeration to hold the PDO request
 */
typedef enum 
{
    CY_SOLN_PDO_VSAFE5V = CY_PD_VSAFE_5V,
    CY_SOLN_PDO_VSAFE9V = CY_PD_VSAFE_9V,
    CY_SOLN_PDO_VSAFE15V = CY_PD_VSAFE_15V,
    CY_SOLN_PDO_VSAFE20V = CY_PD_VSAFE_20V
} cy_en_soln_pdo_t;

/**
 * @brief Enumeration to hold the fault status parameters.
 */
typedef struct
{
    bool enable;
    cy_en_soln_fault_state_t state;
    bool otpBjt;
    bool otpNtc;
    bool faultPollPending;
    bool UVPForceRecExpired;
}cy_en_soln_fault_context_t;

/**
 * @brief Structure to hold the Solution context.
 */
typedef struct {
    /** Type-C and PD contract status */
    bool contract;

    /** Type-C and PD contract change status */
    bool contractUpdate;

    /** PDO request */
    cy_en_soln_pdo_t pdoRequest;

    /** DPM command pending */
    bool dpmPending;

    /* PDO Variables. */
    uint32_t pd_max_cur;
    uint32_t pd_pwr;
    uint32_t pd_volt;
    uint32_t pd_op_cur;

    /* Qi object Detect status. */
    bool objDetected;

    /* Qi object Remove status. */
    bool objRemoved;

    /* Qi CEP status. */
    bool cepRcvd;

    /* Qi Reset Status. */
    bool qiReset;

    /** ADFT backup done */
    bool adftBackup;

    /** ADFT function */
    cy_en_soln_adft_t adft;

    /** ADFT GPIO port */
    GPIO_PRT_Type * adft_port;

    /** ADFT GPIO port pin */
    uint32_t adft_pin;

    /** VIN Voltage */
    uint16_t vinVolt;

    /** VIN Voltage with successful contract*/
    uint16_t vinVoltContract;

    /** UART Instance used for Console block*/
    CySCB_Type * uartSCBInstance;

    /* VIN voltage transition direction */ 
    cy_en_soln_vin_trans_t vinDir;

    /* Qi context */
    cy_stc_qi_context_t * qistack_ctx;

    /* PD context */
    cy_stc_pdstack_context_t * pdstack_ctx;

    /* Fault context */
    cy_en_soln_fault_context_t faultCtx;
    
    /** Die temperature */
    uint8_t dieTemp;

    /** NTC temperature */
    uint8_t ntcTemp; 
    
    /** flag to run automation log task or not */
    bool runAutmoationTask;

    /** VBTR multislope operation */
    uint8_t vbtrSlopeIndex;
    int16_t vbtrDacTarget;

    /** Coil Source Discharge status */
    bool coilSrcDishStatus;

} cy_stc_soln_policy_ctx_t;

/*******************************************************************************
* Function Name: get_soln_context
****************************************************************************//**
*
* This function returns the solution context.
*
* \param portIdx
* Specific Port Index.
*
* \return
* Solution Context pointer
*
*******************************************************************************/
cy_stc_soln_policy_ctx_t * get_soln_context(void);

/*******************************************************************************
* Function Name: get_timer_context
****************************************************************************//**
*
* This function returns the timer context of the solution.
*
* \param portIdx
* Specific Port Index.
*
* \return
* Solution Timer Context pointer
*
*******************************************************************************/
cy_stc_sw_timer_t * get_timer_context(void);

/*******************************************************************************
* Function Name: soln_get_pdstack_context
****************************************************************************//**
*
* This function returns the PD stack context for specified port index.
*
* \param portIdx
* Specific Port Index.
*
* \return
* PdStack Libray Context pointer
*
*******************************************************************************/
cy_stc_pdstack_context_t * get_pdstack_context(uint8_t portIdx);

/*******************************************************************************
* Function Name: soln_get_usbpd_context
****************************************************************************//**
*
* This function returns the USBPD hardware module context for specified 
* port index.
*
* \param portIdx
* Specific Port Index.
*
* \return
* USBPD module Libray Context pointer
*
*******************************************************************************/
cy_stc_usbpd_context_t * get_usbpd_context(uint8_t portIdx);

/*******************************************************************************
* Function Name: soln_get_qistack_context
****************************************************************************//**
*
* This function returns the Qi stack context.
*
* \param portIdx
* None.
*
* \return
* QiStack Libray Context pointer
*
*******************************************************************************/
cy_stc_qi_context_t * get_qistack_context(void);

/*
 * This function keeps backup of previous ADFT signal and pin.
 */
void soln_adft_backup(void);

/*
 * This function restores previous ADFT signal and pin.
 */
void soln_adft_restore(void);

/* 
 * This function enables ADFT on AMUXA for a selected signal 
 * from the list.
 */
void soln_adft_set(cy_en_soln_adft_t adft, GPIO_PRT_Type * port, uint32_t pin);

#endif /* _SOLUTION_COMMON_H_ */
