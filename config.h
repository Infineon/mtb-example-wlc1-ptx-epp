/***************************************************************************//**
* \file config.h
* \version 1.0
*
* @brief @{Header file that enables/disables various CCG firmware features.
*
* This file also provides mapping to the implementation for hardware dependent
* functions like FET control, voltage selection etc.
*
* This current implementation matches the CY4531 EVK from Cypress. This can be
* updated by users to match their hardware implementation.@}
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <cybsp.h>
#include "cy_qistack_common.h"

/*******************************************************************************
 * Header files including
 ******************************************************************************/
#define CY_CABLE_COMP_ENABLE                       (0u)

#ifndef CY_PD_SOURCE_ONLY
/* Restrict PD stack to sink operation. */
#define CY_PD_SOURCE_ONLY                           (0u)
#endif /* CY_PD_SOURCE_ONLY */

#ifndef CY_PD_SINK_ONLY
/* Restrict PD stack to sink operation. */
#define CY_PD_SINK_ONLY                             (1u)
#endif /* CY_PD_SOURCE_ONLY */

#ifndef NO_OF_TYPEC_PORTS
#define NO_OF_TYPEC_PORTS                         (CY_IP_MXUSBPD_INSTANCES)
#endif /* NO_OF_TYPEC_PORTS */

/*******************************************************************************
 * Enable PD spec Rev 3 support
 ******************************************************************************/
#ifndef CY_PD_REV3_ENABLE
#define CY_PD_REV3_ENABLE                        (1u)
#endif /* CY_PD_REV3_ENABLE */

#if CY_PD_REV3_ENABLE
    #define CY_PD_FRS_RX_ENABLE                   (0u)
    #define CY_PD_FRS_TX_ENABLE                   (0u)
#ifndef CY_PD_PPS_SRC_ENABLE
    #define CY_PD_PPS_SRC_ENABLE                  (0u)
#endif /* CY_PD_PPS_SRC_ENABLE */
#endif /* CY_PD_REV3_ENABLE */

#if CY_PD_PPS_SRC_ENABLE
#ifndef CY_PD_VBUS_CF_EN
    #define CY_PD_VBUS_CF_EN                      (0u)
#endif /* CY_PD_VBUS_CF_EN */

    /*
     * Enable/disable workaround for supporting bad sink devices
     * which misbehaves on seeing a PPS PDO.
     */
    #define CCG_REV3_HANDLE_BAD_SINK                (0u)
#endif /* CY_PD_PPS_SRC_ENABLE */

#define CCG_PROG_SOURCE_ENABLE                    (1u)


/* Enable hardware based DRP toggle for additional power saving. */
#define CY_PD_HW_DRP_TOGGLE_ENABLE                (0u)

/*
 * Macro defines additional delay in milliseconds before the PD stack starts sending
 * SRC_CAP message. This may be required to work with some non-compliant sink devices
 * which require more start up time for PD.
 */
#define DELAY_SRC_CAP_START_MS                  (100u)

#define PD_PDO_SEL_ALGO                         (0u)

/* This firmware supports only CCG controlled DP/USB switch operation. */
#define MUX_TYPE                                (3u)

#define FLASHING_MODE_PD_ENABLE                 (1u)


/*******************************************************************************
*                              Solution Configuration
*******************************************************************************/

/**
 * Enabling this macro disables USBPD as VIN type and 
 * uses direct VIN supply.
 */
#define CY_SOLN_QI_OVERRIDE_PD		            (0u)

/**
 * Enabling this macro enables dynamic USBPD contract based on the
 * object presense and PID requirements.
 * This feature keeps the VIN at lower contract if there is no requirement 
 * to deliver higher power from PTx. 
 * This is a power saving feature.
 */
#define CY_SOLN_QI_DYNAMIC_PD_EN                (1u)

/* 
 * OTP entry/exit configuration.
 * Qi shall stop in fault condition and PD continues to run.
 * Entry temperature must be greater than exit temperature.
 */
#define CY_SOLN_OTP_ENABLE                      (1u)

/* VBRG OVP */
#define CY_SOLN_VBRG_OVP_EN                     (1u)

/* VBRG OCP */
#define CY_SOLN_VBRG_OCP_EN                     (1u)

/* VBRG SCP */
#define CY_SOLN_VBRG_SCP_EN                     (1u)
#define CY_SOLN_VBRG_SCP_CUR_10MA               (600u) /* SCP can be either 6A or 10A only */

#if !CY_SOLN_QI_OVERRIDE_PD

/* VIN OVP */
#define CY_SOLN_VIN_OVP_EN                      (1u)

/* VIN UVP */
#define CY_SOLN_VIN_UVP_EN                      (1u)
#define CY_SOLN_VIN_UVP_PER                     (-30) /* Keep negative, X percentage less than contract voltage */

#endif /* CY_SOLN_QI_OVERRIDE_PD */

/* Buck-boost Inductor Current Limit Detection */
#define CY_SOLN_BB_ILIM_EN                      (1u)

/* VREG INRUSH Detection */
#define CY_SOLN_VREG_INRUSH_EN                  (1u)

/* VDDD BOD Detection */
#define CY_SOLN_VDDD_BOD_EN                     (1u)

/*******************************************************************************
 * PSOURCE controls for PD port 1.
 ******************************************************************************/

/* VBUS PGDO FET Control selection based on the FET control pin used in the system hardware */
#define VBUS_FET_CTRL_0                             (1u)
#define VBUS_FET_CTRL_1                             (0u)

#define VBUS_FET_CTRL                               (VBUS_FET_CTRL_0)

#define APP_VBUS_SET_VOLT_P1(context, volt_mV)               vbus_ctrl_fb_set_volt(context, volt_mV)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P1()                   pd_internal_vbus_discharge_on(0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 OFF. */
#define APP_DISCHARGE_FET_OFF_P1()                  pd_internal_vbus_discharge_off(0)

/* 
 * VBUS_IN voltage is monitored during FET ON.
 * Provider FET is not turned ON until VBUS_IN is in safe range.
 * Safe range is 5% above and below VSAFE_5V.
 */
#define APP_PSOURCE_SAFE_FET_ON_ENABLE              (1u)

/*******************************************************************************
 * PSOURCE controls for Port 2 (TYPE A port).
 ******************************************************************************/

#define APP_VBUS_SRC_FET_ON_P2()                    vbus_ctrl_pwm_turn_on(TYPEC_PORT_1_IDX)

#define APP_VBUS_SET_VOLT_P2(volt_mV)               vbus_ctrl_pwm_set_volt(TYPEC_PORT_1_IDX, volt_mV)

#define APP_VBUS_SRC_FET_OFF_P2()                   //vbus_ctrl_pwm_turn_off(TYPEC_PORT_1_IDX)

/* Function/Macro to turn VBUS_DISCHARGE FET for P1 ON. */
#define APP_DISCHARGE_FET_ON_P2()                   ((void)0)

/* Function/Macro to turn VBUS_DISCHARGE FET for P2 OFF. */
#define APP_DISCHARGE_FET_OFF_P2()                  ((void)0)
#define APP_GPIO_POLL_ADC_ID                    (CY_USBPD_ADC_ID_1)
#define APP_VBUS_POLL_ADC_ID                    (CY_USBPD_ADC_ID_0)
#define APP_GPIO_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_A)
#define APP_VBUS_POLL_ADC_INPUT                 (CY_USBPD_ADC_INPUT_AMUX_B)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * upward voltage transition above 5V.
 */
#define VBUS_CTRL_ABOVE_5V_UP_MAX_STEP          (100u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * downward voltage transition above 5V.
 */
#define VBUS_CTRL_ABOVE_5V_DOWN_MAX_STEP        (100u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * upward voltage transition below 5V.
 */
#define VBUS_CTRL_BELOW_5V_UP_MAX_STEP          (100u)

/*
 * Maximum voltage step in mV transition that can happen in one change when
 * downward voltage transition below 5V.
 */
#define VBUS_CTRL_BELOW_5V_DOWN_MAX_STEP        (100u)

/* 
 * Enable this macro if the interval between steps need to be done in micro-
 * seconds. This may be required if the regulator is very sensitive to feedback
 * node change. This will allow for smaller feedback voltage step size. When
 * enabling this feature, a TCPWM timer should be added with TC interrupt 
 * configured for one-shot timer interrupting with the required interval as
 * period. The minimum interval allowed is 200us. Use this only if you need step
 * interval between 200us and 1ms. If this is disabled, the step size is assumed
 * to be 1ms.
 *
 * Most direct feedback based regulators are fast and does not require this. So
 * this is left disabled by default.
 */
#define VBUS_CTRL_STEP_US_ENABLE                (0u)

/* 
 * Period (in ms) to debounce the VBUS stable detection. VBUS change is detected
 * using ADC over the specified duration.
 */
#define VBUS_CTRL_SLOPE_DEBOUNCE_PERIOD         (50u)

/* 
 * Minimum period (in ms) for vbus to settle after reaching stable slope. This
 * is imposed to ensure that we provide a minimum time before indicating ready.
 */
#define VBUS_CTRL_SETTLE_TIME_PERIOD            (15u)

/*
 * Enable / disable VBUS Slow Discharge Feature. When this feature is enabled,
 * the discharge drive strength shall be increased by steps every ms until the
 * selected top drive strength is achieved. Similarly, the drive strength is
 * decreased in steps while stopping the discharge.
 */
#ifndef VBUS_SLOW_DISCHARGE_EN
#define VBUS_SLOW_DISCHARGE_EN                  (0u)
#endif /* VBUS_SLOW_DISCHARGE_EN */

#ifndef VBUS_IN_DISCHARGE_EN
/* VBUS in discharge enable */
#define VBUS_IN_DISCHARGE_EN                    (1u)
#endif /* VBUS_IN_DISCHARGE_EN */

/* 
 * Allow VBUS_IN discharge below 5V.
 * When VBUS_IN_DISCHARGE_EN macro is enabled, VBUS_IN discharge is enabled for all
 * VBUS downward transitions above 5V, but is disabled for transitions below 5V.
 * Because, for VBUS_IN powered solutions, VBUS_IN should not be accidently
 * brought to the low voltage where system behavior is undefined. 
 * VBUS_IN discharge below 5V may be required for solutions where regulator
 * requires higher discharge strength to set voltage below 5V.
 * It is recommended to enable this feature only for solution which are not
 * VBUS_IN powered.
 */
#define VBUS_IN_DISCH_BELOW_5V_EN               (1u)

/* VBUS discharge drive strength settings. */
#if VBUS_IN_DISCHARGE_EN
#define VBUS_C_DISCHG_DS                        (1u)
#else /* !VBUS_IN_DISCHARGE_EN */
#define VBUS_C_DISCHG_DS                        (4u)
#endif
#define VBUS_IN_DISCHG_DS                       (4u)

/* 
 * Enable / disable charge pump.
 * Charge pump is not needed for solutions with Vddd of 5V.
 */
#define PUMP_DISABLE                            (1u)

/*******************************************************************************
 * Power Source (PSOURCE) Configuration.
 ******************************************************************************/

/* Time (in ms) allowed for source voltage to become valid. */
#define APP_PSOURCE_EN_TIMER_PERIOD             (250u)

/* Period (in ms) of VBus validity checks after enabling the power source. */
#define APP_PSOURCE_EN_MONITOR_TIMER_PERIOD     (1u)

/* Time (in ms) between VBus valid and triggering of PS_RDY. */
#define APP_PSOURCE_EN_HYS_TIMER_PERIOD         (5u)

/* Time (in ms) for which the VBus_Discharge path will be enabled when turning power source OFF. */
#define APP_PSOURCE_DIS_TIMER_PERIOD            (600u)

/* Period (in ms) of VBus drop to VSAFE0 checks after power source is turned OFF. */
#define APP_PSOURCE_DIS_MONITOR_TIMER_PERIOD    (1u)

/* VBus Monitoring is done using internal resistor divider. */
#define VBUS_MON_INTERNAL                       (1u)

/* Time (in ms) allowed to wait for VBUS_IN discharge for safe FET ON */
#define APP_PSOURCE_SAFE_FET_ON_TIMER_PERIOD    (50u)

/*******************************************************************************
 * VBus monitor configuration.
 ******************************************************************************/

/* Allowed VBus valid margin as percentage of expected voltage. */
#define VBUS_TURN_ON_MARGIN                     (-20)

/* Allowed VBus valid margin (as percentage of expected voltage) before detach detection is triggered. */
#define VBUS_TURN_OFF_MARGIN                    (-20)

/* Allowed margin over expected voltage (as percentage) for negative VBus voltage transitions. */
#define VBUS_DISCHARGE_MARGIN                   (20)

/* Allowed margin over 5V before the provider FET is turned OFF when discharging to VSAFE0. */
#define VBUS_DISCHARGE_TO_5V_MARGIN             (10)

/*******************************************************************************
 * VBus Monitor connection configuration for Port 1.
 ******************************************************************************/

/* CCG IO port to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PORT_NO_P1                 (VBUS_MON_P1__PORT)

/* CCG IO pin to which the VBUS_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VBUS_MON_PIN_NO_P1                  (VBUS_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VBUS_MON_P1 pin. */
#define APP_VBUS_MON_PORT_PIN_P1                ((VBUS_MON_P1__PORT << 4) | VBUS_MON_P1__SHIFT)

/*
 * IO setting to connect VBUS_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VBUS_MON_AMUX_INPUT_P1              (6)

/*******************************************************************************
 * VBus Monitoring Controls for detach detection.
 ******************************************************************************/

/* Division factor applied between VBus and the voltage on VBUS_MON input. */
#define VBUS_MON_DIVIDER                            (11u)

/*******************************************************************************
 * VBus OCP fault GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PORT_NO_P1               (OCP_FAULT_P1__PORT)

/* CCG pin to which the OCP_FAULT_P1 pin corresponds. This is derived from the PSoC Creator Schematic. */
#define APP_VBUS_OCP_FAULT_PIN_NO_P1                (OCP_FAULT_P1__SHIFT)

/* Combined Port+Pin representation for the OCP_FAULT_P1 pin. */
#define APP_VBUS_OCP_PORT_PIN_P1                    ((OCP_FAULT_P1__PORT << 4) | OCP_FAULT_P1__SHIFT)

/*******************************************************************************
 * VBUS_OVP_TRIP GPIO connection configuration.
 ******************************************************************************/

/* CCG port to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PORT_NO_P1                (VBUS_OVP_TRIP_P1__PORT)

/* CCG pin to which the VBUS_OVP_TRIP_P1 pin corresponds. */
#define APP_VBUS_OVP_TRIP_PIN_NO_P1                 (VBUS_OVP_TRIP_P1__SHIFT)

/* Combined Port+Pin representation of the VBUS_OVP_TRIP_P1 pin. */
#define APP_VBUS_OVP_TRIP_PORT_PIN_P1               ((VBUS_OVP_TRIP_P1__PORT << 4) | VBUS_OVP_TRIP_P1__SHIFT)

/* CCG IO mode corresponding to the VBUS_OVP_TRIP functionality. This should be set to 12. */
#define APP_VBUS_OVP_TRIP_HSIOM_P1                  (12)

/*******************************************************************************
 * Input under voltage and over voltage protection
 ******************************************************************************/

/* VIN UVP mode: AUTO FET Control. */
#define VIN_UVP_MODE                                (2u)

/* VIN UVP Comparator Hysteresis value in mV units */
#define VIN_UV_COMP_HYSTERESIS                      (250u)

/* VIN OVP mode: AUTO FET Control. */
#define VIN_OVP_MODE                                (2u)

/* VIN OVP Comparator Hysteresis value in mV units */
#define VIN_OV_COMP_HYSTERESIS                      (750u)

/*******************************************************************************
 * VConn Monitor connection configuration for Port 1.
 * This section is optional as VConn monitoring is not enabled in the stack.
 ******************************************************************************/

/* CCG IO port to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PORT_NO_P1                    (VCONN_MON_P1__PORT)

/* CCG IO pin to which the VCONN_MON_P1 pin corresponds. This is taken from the PSoC Creator Schematic. */
#define APP_VCONN_MON_PIN_NO_P1                     (VCONN_MON_P1__SHIFT)

/* Combined Port+Pin representation for the VCONN_MON_P1 pin. */
#define APP_VCONN_MON_PORT_PIN_P1                   ((VCONN_MON_P1__PORT << 4) | VCONN_MON_P1__SHIFT)

/*
 * IO setting to connect VCONN_MON_P1 to an internal comparator. This should be selected from:
 * a) HSIOM_MODE_AMUXA = 6, AMUXBUS A connection
 * b) HSIOM_MODE_AMUXB = 7, AMUXBUS B connection
 */
#define APP_VCONN_MON_AMUX_INPUT_P1                 (7)

/*******************************************************************************
 * VBUS offset voltage configuration.
 *
 * Offset voltage value is a configuration table parameter.
 ******************************************************************************/

/* VBUS offset voltage enable setting. */

/*******************************************************************************
 * Battery Charging Support Configuration
 ******************************************************************************/
#define QC_CF_EN                                   (0u)

#define EXTENDED_SRC_CAP_LENGTH                    (24u)

#ifndef BATTERY_CHARGING_ENABLE
#define BATTERY_CHARGING_ENABLE                    (0u)
#endif /* BATTERY_CHARGING_ENABLE */

/* Disable BC 1.2 state machine after PD contract. */
#define CCG_BC_12_IN_PD_ENABLE                     (0u)

#if BATTERY_CHARGING_ENABLE

/*
 * Enable / disable dynamic configuration for supported legacy / proprietary
 * charging protocols. This allows the firmware to override the configuration
 * from the configuration table during run-time.
 */
#define LEGACY_DYN_CFG_ENABLE                       (0u)
#endif /* BATTERY_CHARGING_ENABLE */

#if (QC_CF_EN || LEGACY_DYN_CFG_ENABLE)
/* Defines the max PDP value when PD3.0 is not supported */
#define CCG_MAX_PDP_VALUE                          (36u)
/* Defines the max cable rating when connected to a legacy device. Units of 10mA */
#define LEGACY_MAX_CABLE_RATING                    (300u)
#endif /* (QC_CF_EN || LEGACY_DYN_CFG_ENABLE) */

#define  BC_PORT_0_IDX                             (0u)
#define  BC_PORT_1_IDX                             (1u)

/*******************************************************************************
 * VBus Over-Voltage Protection Configuration.
 *
 * The VBus OVP feature uses an internal ADC in the CCG to measure the voltage
 * on the VBUS_MON input and uses the ADC output to detect over-voltage
 * conditions.
 *
 * The default implementation of OVP uses firmware ISRs to turn off the FETs
 * when OVP is detected. If quicker response is desired, there is the option of
 * using a direct OVP_TRIP output derived from a hardware comparator associated
 * with the ADC.
 ******************************************************************************/

/*******************************************************************************
 * System fault configuration features.
 ******************************************************************************/

/* 
 * Enable/Disable VCONN feature
 */
#define CCG_VCONN_DISABLE                           (0u)

/* 
 * Enable/Disable delay between fault retries for Type-C/PD faults.
 */
#define FAULT_RETRY_DELAY_EN                        (0u)

#if FAULT_RETRY_DELAY_EN

/* 
 * Delay between fault retries in mS.
 */
#define FAULT_RETRY_DELAY_MS                        (500u)

#endif /* FAULT_RETRY_DELAY_EN */

/* 
 * Enable/Disable delayed infinite fault recovery for Type-C/PD faults.
 * Fault recovery shall be tried with a fixed delay after configured 
 * fault retry count is elapsed. 
 */
#define FAULT_INFINITE_RECOVERY_EN                  (0u)

#if FAULT_INFINITE_RECOVERY_EN

/* 
 * Delayed fault recovery period in mS.
 */
#define FAULT_INFINITE_RECOVERY_DELAY_MS            (5000u)

#endif /* FAULT_INFINITE_RECOVERY_EN */

/* Enable watchdog hardware reset for CPU lock-up recovery */
#define WATCHDOG_HARDWARE_RESET_ENABLE              (1u)

/* Disable CCG device reset on error (watchdog expiry or hard fault). */
#define RESET_ON_ERROR_ENABLE                       (1u)

/* Enable reset reporting through HPI. */
#define HPI_WATCHDOG_RESET_ENABLE                   (0u)

/* Watchdog reset timer id. */
#define WATCHDOG_TIMER_ID                           (0xC2u)

/*
 * Watchdog reset period in ms. This should be set to a value greater than
 * 500 ms to avoid significant increase in power consumption.
 */
#define WATCHDOG_RESET_PERIOD_MS                    (750u)

/* Enable tracking of maximum stack usage. */
#define STACK_USAGE_CHECK_ENABLE                    (0u)

/*
 * Macro to enable VBAT-GND SCP fault retry/recovery.
 * When battery voltage is not in safe range (engine ignition OFF voltage),
 * do a long retries for only configured recovery time limit and give up.
 * Otherwise, if battery voltage is good (engine ignition ON voltage),
 * do initial short retries for configured recovery time limit and thereafter
 * do infinite long retries.
 */
#define VBAT_GND_SCP_RECOVERY_ENABLE                (0u)

#if VBAT_GND_SCP_RECOVERY_ENABLE

/*
 * Macro defines lower battery threshold in mV (engine ignition OFF voltage)
 * below which few retries are done to save battery power.
 */
#define VBAT_GND_SCP_BAT_THRESHOLD_MV               (13000u)

/*
 * Macro defines VBAT-GND SCP short retry/recovery period after
 * the fault (in seconds).
 * Maximum allowed period is 60 seconds.
 */
#define VBAT_GND_SCP_SHORT_RETRY_PERIOD_S           (10u)

/*
 * Macro defines VBAT-GND SCP long retry/recovery period after
 * the fault (in seconds).
 * Maximum allowed period is 60 seconds.
 */
#define VBAT_GND_SCP_LONG_RETRY_PERIOD_S            (60u)

/*
 * VBAT_GND SCP recovery timeout limit (in seconds).
 * Maximum allowed period is 2^16 seconds.
 */
#define VBAT_GND_SCP_RETRY_MAX_PERIOD_S             (600u)

#endif /* VBAT_GND_SCP_RECOVERY_ENABLE */

/*******************************************************************************
 * PSOURCE and PSINK controls for PD.
 ******************************************************************************/
/* Regulator controls */
#define REGULATOR_IS_ENABLED(context)               Cy_USBPD_BB_IsEnabled(context)

/*******************************************************************************
 * Firmware feature configuration.
 ******************************************************************************/

/* Set to 1 if building a debug enabled binary with no boot-loader dependency. */
#define CCG_FIRMWARE_APP_ONLY                       (0u)

#ifndef SYS_DEEPSLEEP_ENABLE
/* Enable CCG deep sleep to save power. */
#define SYS_DEEPSLEEP_ENABLE                        (0u)
#endif /* SYS_DEEPSLEEP_ENABLE */

/* Enable Alternate Mode support when CCG is DFP. */
#define DFP_ALT_MODE_SUPP                           (0u)

/* Enable DisplayPort Source support as DFP. */
#define DP_DFP_SUPP                                 (0u)

/* Enable Alt mode as UFP */
#define UFP_ALT_MODE_SUPP                           (0u)

/* Enable saving only SVIDs which are supported by CCG. */
#define SAVE_SUPP_SVID_ONLY                         (1u)

/*
 * The following macro defines whether we will handle extended   
 * message in solution space. 
 */    
#define CCG_SLN_EXTN_MSG_HANDLER_ENABLE             (0u)

/*
 * Enable/Disable firmware active LED operation.
 *
 * The blinking LED is enabled by default but it is recommended to disable it
 * for production designs to save power.
 */
#define APP_FW_LED_ENABLE                           (0u)

/*
 * Timer ID allocation for various solution soft timers.
 */

/*
 * Activity indicator LED timer. The timer is used to indicate that the firmware
 * is functional. The feature is controlled by APP_FW_LED_ENABLE.
 */
#define LED_TIMER_ID                                (0xC0)
/*
 * The LED toggle period.
 */
#define LED_TIMER_PERIOD                            (1000)

/* Timer used to ensure I2C transfers to the MUX complete on time. */
#define MUX_I2C_TIMER                               (0xC1)
/* The MUX transfer timeout is set to 10 ms timeout period. */
#define MUX_I2C_TIMER_PERIOD                        (10u)

/*******************************************************************************
 * Get Battery status and Get Battery configuration response configuration
 ******************************************************************************/
    
/* Valid battery_status response when source */
    
#define CCG_EXT_MSG_VALID_BAT_STAT_SRC              (0xffff0600)

/* Valid battery_status response when sink */
    
#define CCG_EXT_MSG_VALID_BAT_STAT_SNK              (0xffff0200)
    
/* Invalid battery_status response */
    
#define CCG_EXT_MSG_INVALID_BAT_REF                 (0xffff0100)

/* This macro defines the number of batteries */   
#define CCG_PB_NO_OF_BATT                           (1u)   

/* This macro defines the VID-PID for Power Bank */   
#define CCG_PB_VID_PID                              (0xf6b104b4)

/* This macro defines the battery design capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Design capacity unknown
 */     
#define CCG_PB_BAT_DES_CAP                          (0xFFFF)
    
/* This macro defines the battery last full charge capacity (in 0.1WH)
 * 0x0000 : Battery not present
 * 0xFFFF : Last full charge capacity unknown
 */
#define CCG_PB_BAT_FUL_CHG_CAP                      (0xFFFF)

/*
 * Macro enables the black box feature to store system faults and
 * debug information in flash.
 * User can read black box flash row to see the counters against each element
 * of this black box.
 */
#define SYS_BLACK_BOX_ENABLE                        (0u)

/*
 * Macro defines debounce time period in milliseconds from blackbox event to
 * start of blackbox data write, if VDDD is stable.
 */
#define SYS_BLACK_BOX_WRITE_DEFER_TIME              (100u)

/*
 * Macro when enabled facilitates VDDD stability check before start of blackbox
 * data write. VDDD will be monitored for 20ms at every 5ms sampling interval.
 * This feature, if enabled, executes after SYS_BLACK_BOX_WRITE_DEFER_TIME
 * milliseconds from blackbox specific event.
 */
#define SYS_BLACK_BOX_DEBOUNCE_ENABLE               (0u)

/*
 * Macro to enable power cycle counter in black box.
 * When enabled, firmware will be blocked for 20ms every power cycle to write to flash.
 */
#define SYS_BLACK_BOX_POWER_CYCLE_COUNT_ENABLE      (0u)

/*
 * Macro to enable type-c detach counter in black box.
 * When enabled, firmware will be blocked for 20ms every type-c detach to write to flash.
 */
#define SYS_BLACK_BOX_DETACH_COUNT_ENABLE           (0u)

/* Single firmware Image */
#define CCG_DUALAPP_DISABLE                         (1u)

/* Enable HPI support. */
#define CCG_HPI_ENABLE                              (1u)

/* Enable HPI with SROM */
#define HPI_AUTO_SROM_ENABLE                        (0u)

/*
 * Defines the I2C master scb index. I2C Index shall be used for temperature
 * reading with attached I2C sensors. It will also be used for performing
 * load sharing for Master CCG3PA.
 */
#define CCG_I2C_MASTER_SCB_INDEX                    (0u)

/*
 * Index of SCB used for Load sharing functionality for Slave CCG3PA partner.
 */
#define CCG_I2C_SLAVE_SCB_INDEX                     (3u)

/* HPI SCB Index */
#define HPI_SCB_INDEX                               (CCG_I2C_SLAVE_SCB_INDEX)

#if CCG_HPI_ENABLE

/* Should be enabled if HPI Auto is used */
#define CCG_HPI_AUTO_ENABLE                         (0u)

/* Enable external billboard */
#define CCG_BB_ENABLE                               (0u)

/* Should be enabled if HPI BillBoard is used */
#define CCG_HPI_BB_ENABLE                           (0u)

/* Billboard operational model - No EC + HPI event based */
#define HPI_BB_OPER_MODEL                           (0x01)

/*
 * Enables HPI Auto specific command handling support.
 * Should not be enabled unless supported by underlying HPI AUTO library.
 *
 */
#define CCG_HPI_AUTO_CMD_ENABLE                     (0u)
#endif /* CCG_HPI_ENABLE */


/* LIN enable setting. */
#define CCG_LIN_ENABLE                              (0u)

#if CCG_LIN_ENABLE
/* LIN SCB Index */
#define CCG_LIN_SCB_INDEX                           (1u)
/* LIN TX Pin */
#define CCG_LIN_SCB_PIN_TX                          ((gpio_port_pin_t)((UART_LIN_TX_PORT_NUM << 4u) | (UART_LIN_TX_PIN)))
/* LIN RX Pin */
#define CCG_LIN_SCB_PIN_RX                          ((gpio_port_pin_t)((UART_LIN_RX_PORT_NUM << 4u) | (UART_LIN_RX_PIN)))
/* LIN EN Pin */
#define CCG_LIN_SCB_PIN_EN                          ((gpio_port_pin_t)((UART_LIN_TXR_EN_PORT_NUM << 4u) | (UART_LIN_TXR_EN_PIN)))
#endif /* CCG_LIN_ENABLE */
/***********************************************************************************/

#if (CCG_HPI_ENABLE && CCG_LIN_ENABLE)
/* LIN HPI enable setting. */
#define CCG_HPI_OVER_LIN_ENABLE                     (1u)
#else
/* LIN HPI enable setting. */
#define CCG_HPI_OVER_LIN_ENABLE                     (0u)
#endif /* (CCG_HPI_ENABLE && CCG_LIN_ENABLE) */

#if (!CCG_LIN_ENABLE)
#define CCG_LPM_GPIO_ENABLE                      	(0u)
#endif /* !CCG_LIN_ENABLE */

/*******************************************************************************
 * Load Sharing specific Configuration.
 ******************************************************************************/
/*
 * Enables load sharing protocol between controllers.
 */
#ifndef CCG_LOAD_SHARING_ENABLE
#define CCG_LOAD_SHARING_ENABLE                     (0u)
#endif /* CCG_LOAD_SHARING_ENABLE */

/* Defines total number of ports sharing the load */
#define CCG_LOAD_SHARING_TOTAL_PORTS_COUNT          (2u)

/* Defines the load sharing slave address */
#define CCG_LOAD_SHARING_SLAVE_ADDRESS              (0x08u)

/* Enable BIST shared test mode handling */
#if CCG_LOAD_SHARING_ENABLE
#define CCG_BIST_STM_ENABLE                         (1u)
#endif /* CCG_LOAD_SHARING_ENABLE */

/*******************************************************************************
 * Power throttling specific Configuration.
 ******************************************************************************/
/*
 * Enables voltage throttling based on sensor temperature.
 */
#ifndef CCG_TEMP_BASED_VOLTAGE_THROTTLING
#define CCG_TEMP_BASED_VOLTAGE_THROTTLING           (0u)
#endif /* CCG_TEMP_BASED_VOLTAGE_THROTTLING */
/*
 * Set this macro to 1 if the temperature based power throttling is
 * done with thermistor.
 */

#define TEMPERATURE_SENSOR_IS_THERMISTOR            (1u)

/*
 * Set this macro to 1 if the temperature based power throttling is
 * done with I2C sensor
 */

#define TEMPERATURE_SENSOR_IS_I2C                   (0u)

#if TEMPERATURE_SENSOR_IS_I2C
    /*
     * Defines the slave address of the temperature sensor.
     */
    #define THROTTLE_SLAVE_ADDRESS                  (0x1A)
    /**
    @brief Temperature sensor count
    */
    #define TEMPERATURE_SENSOR_COUNT                (4u)

#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */

#if TEMPERATURE_SENSOR_IS_THERMISTOR

/**
@brief Temperature sensor count
*/
#define TEMPERATURE_SENSOR_COUNT                    (1u)

/*
 * Defines the temperature starting from which mapping is done
 */
#define BASE_MAP_TEMP                               (20u)

/*
 * Defines the resolution of the map function in deg(C)
 */
#define TEMP_MAP_RESOLUTION                         (5u)

/*
 * Defines the number of map data available
 */
#define VOLT_TO_TEMP_MAP_TABLE_COUNT                (20u)

/*
 * Defines the safe temperature value when the thermistor voltage is below
 * the first entry of the table
 */
#define SAFE_DEFAULT_TEMPERATURE                    (25u)

/*
 * Macro when set defines the Thermistor type as NTC.
 */
#define THERMISTOR_IS_NTC                           (1u)

/*
 * Macro defines the pin to which port 0, thermistor 0 is connected
 */
#define PORT_0_THERMISTOR_0_PIN_CONNECTION          (GPIO_PORT_3_PIN_2)

/*
 * Macro defines the pin to which port 0, thermistor 0 is connected
 */
#define PORT_1_THERMISTOR_0_PIN_CONNECTION          (GPIO_PORT_3_PIN_2)

#endif /* TEMPERATURE_SENSOR_IS_THERMISTOR */

/*
 * Defines the fault value returned by I2C/Thermistor.
 */
#define THROTTLE_SENSOR_FAULT                       (0xFF)

/*
 * Defines the value for increment in case of sensor fault/read failure
 */
#define THROTTLE_SENSOR_FAULT_INCR                  (1u)

/*
 * Enables voltage throttling based on battery voltage(VIN).
 */
#ifndef CCG_VIN_BASED_VOLTAGE_THROTTLING
#define CCG_VIN_BASED_VOLTAGE_THROTTLING            (0u)
#endif /* CCG_VIN_BASED_VOLTAGE_THROTTLING */

#define THROTTLE_VDDD_REFERENCE                     (5000u)

#define APP_GPIO_POLL_ADC_HSIOM                 (HSIOM_MODE_AMUXA)

#define CCG_LPM_GPIO_ENABLE                      (0u)

#define CCG_PSEUDO_METADATA_DISABLE             (1u)

/* Disable the bootwait window feature - only change this macro if I2C interface is enabled */
#define BOOT_WAIT_WINDOW_DISABLE                    (1u)

#define CCG_BOOT                                    (0u)

/* Single firmware Image */
#define CCG_DUALAPP_DISABLE                         (1u)


/***********************************************************************************/

#define CCG_SROM_CODE_ENABLE                        (0u)

/* Manufacturing id */
#define CCG_MFG_ID                                   (0xF6B10000)
#endif /* _CONFIG_H_ */

/* End of file */
