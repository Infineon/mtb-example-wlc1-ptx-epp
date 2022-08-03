/***************************************************************************//**
* \file solution.h
* \version 1.00
*
* Header file of solution layer.
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _SOLUTION_H_
#define _SOLUTION_H_

#include <stdint.h>
#include <stdbool.h>
#include "app.h"
#include "psink.h"
#include "cy_qistack_pm.h"
#include "solution_common.h"

/*******************************************************************************
*                              Type Definitions
*******************************************************************************/

/** Minimum VIN voltage for Qi policy to start */
#define CY_SOLN_MIN_VIN_FOR_QI			(5000u)

/** ASK path gain options */
#define CY_ADFT_VOLT_H_GAIN              	(get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->askDemodVoltHighGain)
#define CY_ADFT_VOLT_L_GAIN              	(get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->askDemodVoltLowGain)
#define CY_ADFT_CUR_H_GAIN               	(7u) /* Gain = 110, but not to be used as CC is enabled. */
#define CY_ADFT_CUR_L_GAIN               	(get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->askDemodCurrGain)

/* Sample the VBus current using ADC. */
#define CY_AMUX_ADC_CCG7D_VBUS_MON_SEL          (4u)

/* Sample the VBus current using ADC. */
#define CY_CSA_GAIN_MAX_VALUE                   (1850u)

/** SPI OVS value to be used. Use oversampling of 16. */
#define CY_QI_BMC_RX_SPI_OVS_VALUE              (15u)

/** SCB interrupt vector number */
#define CY_QI_BMC_RX_SPI_INTR_NUM(port)         (8u + (port))

/** Analog ping pulse count */
#define CY_QI_ANALOG_PING_PULSE_COUNT           (4u)

/** System frequency */
#define CY_QI_SYS_CLK_FREQ_KHZ                  (48000u)

/** Tolerance for the Maximum supported coil source voltage */
#define CY_QI_VIN_BUCK_TOLERANCE                (500u)

/** Coil source VBTR multi-slope configuration */
#define CY_QI_VBTR_MULTISLOPE_COUNT             (3) /** Do not modify. Implementation specific. */
#define CY_QI_VBTR_MULTISLOPE_1_TARGET_DIV      (2) /** Keep signed */
#define CY_QI_VBTR_MULTISLOPE_1_WIDTH_MUL       (1) /** Keep signed */
#define CY_QI_VBTR_MULTISLOPE_2_TARGET_DIV      (2) /** Keep signed */
#define CY_QI_VBTR_MULTISLOPE_2_WIDTH_MUL       (4) /** Keep signed */
#define CY_QI_VBTR_MULTISLOPE_3_TARGET_DIV      (1) /** Keep signed */
#define CY_QI_VBTR_MULTISLOPE_3_WIDTH_MUL       (10) /** Keep signed */

/**
 * Current calibration data. The data at various current values are stored in
 * SFLASH to generate extrapolationc cure. The TRIM uses two bytes to store 
 * data in 10X form. It contains Vsense * Gain voltage value in 100uV units.
 */
#define CBL_GAIN150_TRIM_P250A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff399u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff39au) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN150_TRIM_P500A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff39bu) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff39cu) + ((uint32_t)(port) * 0x26))))

#define CBL_GAIN70_TRIM_P250A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff37fu) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff380u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN70_TRIM_P500A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff381u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff382u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN70_TRIM_1A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff383u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff384u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN70_TRIM_1P5A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff385u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff386u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN70_TRIM_2A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff387u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff388u) + ((uint32_t)(port) * 0x26))))

#define CBL_GAIN40_TRIM_1A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff38du) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff38eu) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN40_TRIM_1P5A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff38fu) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff390u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN40_TRIM_2A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff391u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff392u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN40_TRIM_2P5A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff393u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff394u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN40_TRIM_3A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff395u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff396u) + ((uint32_t)(port) * 0x26))))
#define CBL_GAIN40_TRIM_4A_ROOM(port) MAKE_WORD( \
        (*(volatile uint8_t *)((0x0ffff397u) + ((uint32_t)(port) * 0x26))), \
        (*(volatile uint8_t *)((0x0ffff398u) + ((uint32_t)(port) * 0x26))))


extern uint32_t gl_contract_voltage[];

/*******************************************************************************
* Function Name: soln_policy_init
****************************************************************************//**
*
* This function runs the solution policy to handle PD stack, Qi stack and
* any other tasks in the system.
*
* \param portIdx
* None.
*
* \return
* None.
*
*******************************************************************************/
void soln_policy_init(cy_stc_soln_policy_ctx_t * soln_ctx,
		cy_stc_qi_context_t * qistack_ctx,
		cy_stc_pdstack_context_t * pdstack_ctx);

/*******************************************************************************
* Function Name: soln_policy_task
****************************************************************************//**
*
* This function runs the solution policy to handle PD stack and Qi stack.
*
* \param portIdx
* None.
*
* \return
* None.
*
*******************************************************************************/
void soln_policy_task(cy_stc_soln_policy_ctx_t * soln_ctx);

/*******************************************************************************
* Function Name: soln_policy_sleep
****************************************************************************//**
*
* This function checks solution policy and enters deep sleep if allowed 
* to save power.
*
* \param portIdx
* None.
*
* \return
* None.
*
*******************************************************************************/
void soln_policy_sleep(cy_stc_soln_policy_ctx_t * soln_ctx);

/*******************************************************************************
* Function Name: soln_vconn_enable
                 soln_vconn_disable
                 soln_vconn_is_present
****************************************************************************//**
*
* PD stack solution level functions for VCONN.
*
* WICG is a PD sink solution which does not support VCONN.
* So we need to override the VCONN function to not apply
* VCONN. VCONN is internally used by firmware to remove Rd.
*
*******************************************************************************/
bool soln_vconn_enable(cy_stc_pdstack_context_t * port_ctx, uint8_t channel);
void soln_vconn_disable(cy_stc_pdstack_context_t * port_ctx, uint8_t channel);
bool soln_vconn_is_present(cy_stc_pdstack_context_t * port_ctx);

/*******************************************************************************
* Function Name: soln_vbus_is_present
                 soln_vbus_get_value
****************************************************************************//**
*
* PD stack solution level functions for VBUS.
*
* WICG uses Port 1 instance of UDBPD module for VBUS measurement of Port 0 
* PD stack. It's a override because of hardware configuration.
*******************************************************************************/
bool soln_vbus_is_present(cy_stc_pdstack_context_t * port_ctx, uint16_t volt, int8_t per);
uint16_t soln_vbus_get_value(cy_stc_pdstack_context_t * port_ctx);

/*******************************************************************************
* Function Name: soln_psnk_enable
                 soln_psnk_disable
                 soln_psnk_set_current
                 soln_psnk_set_voltage
****************************************************************************//**
*
* PD stack solution level functions for SINK voltage/current.
*
* WICG uses Sink functions to enable/disable faults and other specific actions.
*******************************************************************************/
void soln_psnk_enable (cy_stc_pdstack_context_t * port_ctx);
void soln_psnk_disable (cy_stc_pdstack_context_t * port_ctx, cy_pdstack_sink_discharge_off_cbk_t snk_discharge_off_handler);
void soln_psnk_set_current (cy_stc_pdstack_context_t * port_ctx, uint16_t cur_10mA);
void soln_psnk_set_voltage (cy_stc_pdstack_context_t * port_ctx, uint16_t volt_mV);
void soln_eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap, cy_pdstack_app_resp_cbk_t app_resp_handler);

/*******************************************************************************
* Function Name: soln_pd_event_handler
****************************************************************************//**
*
* PD stack solution level function for Type-C and PD event handling.
*
* Handles WICG specific actions for required Type-C and PD event.
*******************************************************************************/
void soln_pd_event_handler(cy_stc_pdstack_context_t * port_ctx, cy_en_pdstack_app_evt_t evt, const void *data);

/*******************************************************************************
* Function Name: soln_qi_event_handler
****************************************************************************//**
*
* Qi stack solution level function for Type-C and PD event handling.
*
* Handles WICG specific actions for required Type-C and PD event.
*******************************************************************************/
void soln_qi_event_handler(
            struct cy_stc_qi_context *qiCtx,
            cy_en_qi_app_evt_t evt
            );

/*******************************************************************************
* Function Name: soln_qi_hardware_init
****************************************************************************//**
*
* Qi stack solution level function for device specific hardware initialization.
*
*******************************************************************************/
void soln_qi_hardware_init(
        struct cy_stc_qi_context *qiCtx
        );

/*******************************************************************************
* Function Name: soln_qi_ask_bmc_init
****************************************************************************//**
*
* Qi stack solution level function for ASK BMC decoder hardware initialization.
*
*******************************************************************************/
void soln_qi_ask_bmc_init(
        struct cy_stc_qi_context *qiCtx
        );

/*******************************************************************************
* Function Name: soln_qi_set_ask_path
****************************************************************************//**
*
* Qi stack solution level function for device specific ASK demodulation
* path selection.
*
*******************************************************************************/
void soln_qi_set_ask_path(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_ask_path_t askPath
        );

/*******************************************************************************
* Function Name: soln_qi_fsk_oper_init
****************************************************************************//**
*
* Qi stack solution level function for FSK hardware initialization.
*
*******************************************************************************/
void soln_qi_fsk_oper_init(
        struct cy_stc_qi_context *qiCtx
        );

/*******************************************************************************
* Function Name: soln_qi_ask_cc_up_cmp_enable
                 soln_qi_cc_up_cmp_enable
                 soln_qi_cc_up_cmp_disable
                 soln_qi_cc_dn_cmp_enable
                 soln_qi_cc_dn_cmp_disable
                 soln_qi_pds_scp_cmp_enable
                 soln_qi_pds_scp_cmp_disable
****************************************************************************//**
*
* Qi stack solution level functions for comparator control
*
*******************************************************************************/
void soln_qi_ask_cc_up_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        );

void soln_qi_cc_up_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        );

void soln_qi_cc_up_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_cc_dn_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        );

void soln_qi_cc_dn_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_pds_scp_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        void * cbk
        );

void soln_qi_pds_scp_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        );

/*******************************************************************************
* Function Name: soln_qi_inv_fb_enable
                 soln_qi_inv_fb_disable
                 soln_qi_inv_send_analog_ping
                 soln_qi_inv_start_digital_ping
                 soln_qi_inv_stop_digital_ping
****************************************************************************//**
*
* Qi stack solution level functions for Inverter full birdge control.
*
*******************************************************************************/
void Cy_USBPD_INV_Init(
        cy_stc_usbpd_context_t *context
        );

void soln_qi_inv_fb_enable(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_inv_fb_disable(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_inv_send_analog_ping(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_inv_start_digital_ping(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_inv_stop_digital_ping(
        struct cy_stc_qi_context *qiCtx
        );
/*******************************************************************************
* Function Name: soln_qi_coil_src_enable
                 soln_qi_coil_src_disable
                 soln_qi_coil_src_set_voltage
                 soln_qi_coil_src_ready_status
                 soln_qi_coil_src_get_voltage
                 soln_qi_coil_src_get_current
                 soln_qi_coil_src_discharge_en
                 soln_qi_coil_src_discharge_dis
****************************************************************************//**
*
* Qi stack solution level functions for Coil Vbridge voltage/current.
*
*******************************************************************************/
void soln_qi_coil_src_enable(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_coil_src_disable(
        struct cy_stc_qi_context *qiCtx
        );

bool soln_qi_coil_src_enable_status(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_coil_src_set_voltage(
        struct cy_stc_qi_context *qiCtx,
        uint16_t volt_mV,
        bool multiSlope
        );

bool soln_qi_coil_src_ready_status(
        struct cy_stc_qi_context *qiCtx
        );

uint16_t soln_qi_coil_src_get_voltage(
        struct cy_stc_qi_context *qiCtx
        );

uint16_t soln_qi_coil_src_get_current(
        struct cy_stc_qi_context *qiCtx,
        uint8_t avgSamples
        );

void soln_qi_coil_src_discharge_en(
        struct cy_stc_qi_context *qiCtx
        );

void soln_qi_coil_src_discharge_dis(
        struct cy_stc_qi_context *qiCtx
        );

/**
 * @brief Solution level function for LED Handling
 * 
 * @param pin 
 * @param setValue 
 */
void soln_qi_led_set_pin_value(
        uint8_t pin,
         bool setValue);

/**
 * @brief Solution level handler for Write String 
 * 
 * @param string 
 */
void soln_qi_uart_write_string(
        char_t const string[]
        );

/**
 * @brief Auth Init PWM
 *
 */
void soln_auth_init_pwm(void);

/*******************************************************************************
* Function Name: cy_soln_fault_init
                 cy_soln_fault_task
****************************************************************************//**
*
* Solution level fault handling
*
*******************************************************************************/
void cy_soln_fault_init(void);
void cy_soln_fault_task(void);


uint16_t soln_qi_coil_src_max_supported_volt(
        struct cy_stc_qi_context *qiCtx
        );
/**
 * This task reads the UART queue buffer until it gets empty and
 * print it on Console
 */
bool cy_soln_uart_task(
       cy_stc_soln_policy_ctx_t *soln_ctx);

/**
 * This function enables the hardware block to use external clock.
 * Expected to be called only once during initialization.
 */
void soln_hfclk_ext_clk_init(void);

/**
 * This function disabled the hardware block to use external clock.
 * Expected to be called only once during shutdown or power save.
 */
void sol_hfclk_ext_clk_deinit(void);

/**
 * This function selects the HF clock source. Default source is IMO.
 * Selection enable will cause the HF clock source to be switched to
 * external clock. Calling this function shall disconnect IMO. The
 * hardware must have external clock supplied else, will result in
 * CPU locking up.
 */
void soln_hfclk_ext_clk_ctrl(
        bool enable
        );

/**
 * @brief Used for Set and Clear for the DEBUG_PIN , Can be used from Qi Stack for Debug.
 * 
 * @param qiCtx 
 * @param setorclear 
 */
void soln_debug_gpio_setvalue(struct cy_stc_qi_context *qiCtx, bool setorclear);

#endif /* _SOLUTION_H_ */
