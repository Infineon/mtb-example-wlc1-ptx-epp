/***************************************************************************//**
* \file app_version.h
* \version 1.0
*
* @brief @{Version definition for the CCG firmware application. This version
* information follows a Cypress defined format that identifies the type
* of application along with the version information.@}
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#ifndef _APP_VERSION_H_
#define _APP_VERSION_H_
    

/******************************************************************************
 * Constant definitions.
 *****************************************************************************/

#define APP_TYPE_ENTERTAINMENT          (0x6365u)        /* "ce" */
#define APP_TYPE_CHARGER                (0x6367u)        /* "cg" */
#define APP_TYPE_HEADUNIT               (0x6875u)        /* "hu" */
#define APP_TYPE_WIRELESS               (0x7769u)        /* "wi" */

#define APP_TYPE_STRING                 (APP_TYPE_WIRELESS)
#define APP_EXT_CIR_NUM                 (0x00u)
#define APP_MAJOR_VERSION               (0x00u)
#define APP_MINOR_VERSION               (0x00u)

/*
   The version format is as follows:
   Bytes 1:0 identify the type of CCG application. 0x6e62 specifies that this is a notebook port controller.
   Byte    2 identifies the hardware design version.
   Byte    3 identifies the major and minor version of the firmware.
 */
#define APP_VERSION                                             \
    (((uint32_t)APP_TYPE_STRING) | ((uint32_t)APP_EXT_CIR_NUM << 16) |              \
     ((uint32_t)APP_MINOR_VERSION << 24) | ((uint32_t)APP_MAJOR_VERSION << 28))

#endif /* _APP_VERSION_H_ */

/* [] END OF FILE */

