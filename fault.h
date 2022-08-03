/***************************************************************************//**
* \file fault.h
* \version 1.00
*
* Header file of solution layer fault protection.
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _FAULT_H_
#define _FAULT_H_

#include <stdint.h>
#include <stdbool.h>
#include "solution_common.h"

/*******************************************************************************
*                              Type Definitions
*******************************************************************************/


/*******************************************************************************
* Function Name: soln_fault_init
****************************************************************************//**
*
* This function initializes faults in the system.
*
* \param portIdx
* None.
*
* \return
* None.
*
*******************************************************************************/
void cy_soln_fault_vbrg_ovp_en(void);
void cy_soln_fault_vbrg_ovp_dis(void);
void cy_soln_fault_vbrg_ocp_en(uint16_t faultOcpThr);
void cy_soln_fault_vbrg_ocp_dis(void);
void cy_soln_fault_vbrg_scp_en(void);
void cy_soln_fault_vbrg_scp_dis(void);
void cy_soln_fault_vin_ovp_en(void);
void cy_soln_fault_vin_ovp_dis(void);
void cy_soln_fault_vin_uvp_en(void);
void cy_soln_fault_vin_uvp_dis(void);
void cy_soln_fault_handler(cy_en_soln_fault_t evt);
void cy_soln_fault_state_request(cy_en_soln_fault_state_t state);

#endif /* _FAULT_H_ */
