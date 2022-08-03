/***************************************************************************//**
* \file fault.c
* \version 1.00
*
* Source file of solution layer fault protection.
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "fault.h"
#include "solution.h"
#include "app.h"
#include "thermistor.h"
#include "cy_usbpd_config_table.h"
#include "cy_qistack_console.h"
#include "cy_qistack_utils.h"


/*******************************************************************************
*                        Static Function Definitions
*******************************************************************************/

/* Array to map the thermal voltage read by thermistor to corresponding temperature */
const uint16_t gl_thermistor_volt_map_table[VOLT_TO_TEMP_MAP_TABLE_COUNT] =
{
    /* This voltage-temperature mapping is based on NTCG164KF104FTDS NTC thermistor 
     * If a different thermistor is to be used, the table has to be updated.
     */
    3060, /* Corresponds to 20 degree C */
    2769, /* Corresponds to 25 degree C */
    2478, /* Corresponds to 30 degree C */
    2197, /* Corresponds to 35 degree C */
    1930, /* Corresponds to 40 degree C */
    1683, /* Corresponds to 45 degree C */
    1459, /* Corresponds to 50 degree C */
    1258, /* Corresponds to 55 degree C */
    1081, /* Corresponds to 60 degree C */
    926, /* Corresponds to 65 degree C */
    793, /* Corresponds to 70 degree C */
    678, /* Corresponds to 75 degree C */
    579, /* Corresponds to 80 degree C */
    496, /* Corresponds to 85 degree C */
    425, /* Corresponds to 90 degree C */
    364, /* Corresponds to 95 degree C */
    313, /* Corresponds to 100 degree C */
    270, /* Corresponds to 105 degree C */
    233, /* Corresponds to 110 degree C */
    201  /* Corresponds to 115 degree C */
};

static void cy_cb_soln_fault_poll_time(cy_timer_id_t id, void * context)
{
    CY_UNUSED_PARAMETER(id);
    get_soln_context()->faultCtx.faultPollPending = true;
}

static void cy_cb_soln_fault_retry_time(cy_timer_id_t id, void * context)
{
    CY_UNUSED_PARAMETER(id);
    cy_soln_fault_state_request(CY_SOLN_FAULT_QI_RECOVER);
}

#if CY_SOLN_VBRG_OVP_EN
static bool cy_cb_soln_fault_vbrg_ovp(void *context, bool compOut)
{
    cy_soln_fault_vbrg_ovp_dis();
    cy_soln_fault_handler(CY_SOLN_FAULT_VBRG_OVP);
    return true;
}
#endif /* CY_SOLN_VBRG_OVP_EN */

#if CY_SOLN_VBRG_OCP_EN
static bool cy_cb_soln_fault_vbrg_ocp(void *context, bool compOut)
{
    cy_soln_fault_vbrg_ocp_dis();
    cy_soln_fault_handler(CY_SOLN_FAULT_VBRG_OCP);
    return true;
}
#endif /* CY_SOLN_VBRG_OCP_EN */

#if CY_SOLN_VBRG_SCP_EN
static bool cy_cb_soln_fault_vbrg_scp(void *context, bool compOut)
{
    cy_soln_fault_vbrg_scp_dis();
    cy_soln_fault_handler(CY_SOLN_FAULT_VBRG_SCP);
    return true;
}
#endif /* CY_SOLN_VBRG_SCP_EN */

#if CY_SOLN_VIN_OVP_EN
static void cy_cb_soln_fault_vin_ovp(void *context, bool compOut)
{
    cy_soln_fault_vin_ovp_dis();
    cy_soln_fault_handler(CY_SOLN_FAULT_VIN_OVP);
}

#endif /* CY_SOLN_VIN_OVP_EN */

#if CY_SOLN_VIN_UVP_EN
static void cy_cb_soln_fault_vin_uvp(void *context, bool compOut)
{
    cy_soln_fault_vin_uvp_dis();
    cy_soln_fault_handler(CY_SOLN_FAULT_VIN_UVP);
}

static void cy_cb_soln_fault_vin_uvp_force_time(cy_timer_id_t id, void *callbackContext)
{
    get_soln_context()->faultCtx.UVPForceRecExpired = true;
}
#endif /* CY_SOLN_VIN_OVP_EN */

#if CY_SOLN_BB_ILIM_EN
static void cy_cb_soln_fault_bb_ilim(void *context, bool compOut)
{
    cy_soln_fault_handler(CY_SOLN_FAULT_BB_ILIM);
}
#endif /* CY_SOLN_BB_ILIM_EN */

#if CY_SOLN_VREG_INRUSH_EN
static void cy_cb_soln_fault_vreg_inrush(void *context, bool compOut)
{
    cy_soln_fault_handler(CY_SOLN_FAULT_VREG_INRUSH);
}
#endif /* CY_SOLN_VREG_INRUSH_EN */

#if CY_SOLN_VDDD_BOD_EN
static void cy_cb_soln_fault_vreg_bod(void *context, bool compOut)
{
    cy_soln_fault_handler(CY_SOLN_FAULT_VDDD_BOD);
}
#endif /* CY_SOLN_VDDD_BOD_EN */

void cy_soln_fault_init(void)
{
    cy_en_soln_fault_context_t * faultCtx = &get_soln_context()->faultCtx;
    cy_stc_usbpd_context_t * usbpdCtx = get_usbpd_context(CY_SOLN_BB_INDEX);

    memset(faultCtx, 0, sizeof(cy_en_soln_fault_context_t));

    /* Register NTC voltage and temperature mapping table */
    register_thermistor_mapping_table(get_soln_context()->pdstack_ctx,
        gl_thermistor_volt_map_table);

    /* Register System fault handlers */
#if CY_SOLN_BB_ILIM_EN
    usbpdCtx->bbIlimCbk = cy_cb_soln_fault_bb_ilim;
#endif /* CY_SOLN_BB_ILIM_EN */
#if CY_SOLN_VREG_INRUSH_EN
    usbpdCtx->vregInrushCbk = cy_cb_soln_fault_vreg_inrush;
#endif /* CY_SOLN_VREG_INRUSH_EN */
#if CY_SOLN_VDDD_BOD_EN
    usbpdCtx->bodCbk = cy_cb_soln_fault_vreg_bod;
#endif /* CY_SOLN_VDDD_BOD_EN */
}

#if CY_SOLN_VBRG_OVP_EN
void cy_soln_fault_vbrg_ovp_en(void)
{
    const cy_stc_fault_protect_cfg_t * faultCfg =
        get_wireless_fault_config(get_qistack_context()->ptrCfg);

    if(0 != faultCfg->faultOVPThr)
    {
        Cy_USBPD_Fault_Vbus_OvpEnable(get_usbpd_context(CY_SOLN_BB_INDEX),
            faultCfg->faultOVPThr,
            cy_cb_soln_fault_vbrg_ovp,
            false);
    }
}

void cy_soln_fault_vbrg_ovp_dis(void)
{
    Cy_USBPD_Fault_Vbus_OvpDisable(get_usbpd_context(CY_SOLN_BB_INDEX),
        false);
}
#endif /* CY_SOLN_VBRG_OVP_EN */

#if CY_SOLN_VBRG_OCP_EN
void cy_soln_fault_vbrg_ocp_en(uint16_t faultOcpThr)
{
    if (0 != faultOcpThr)
    {
        Cy_USBPD_Fault_Vbus_OcpEnable(get_usbpd_context(CY_SOLN_BB_INDEX),
            ((faultOcpThr)/10),
            cy_cb_soln_fault_vbrg_ocp);
    }
}

void cy_soln_fault_vbrg_ocp_dis(void)
{
    Cy_USBPD_Fault_Vbus_OcpDisable(get_usbpd_context(CY_SOLN_BB_INDEX), false);
}
#endif /* CY_SOLN_VBRG_OCP_EN */

#if CY_SOLN_VBRG_SCP_EN
void cy_soln_fault_vbrg_scp_en(void)
{
    Cy_USBPD_Fault_Vbus_ScpEnable(get_usbpd_context(CY_SOLN_BB_INDEX),
        CY_SOLN_VBRG_SCP_CUR_10MA,
        cy_cb_soln_fault_vbrg_scp);
}

void cy_soln_fault_vbrg_scp_dis(void)
{
    Cy_USBPD_Fault_Vbus_ScpDisable(get_usbpd_context(CY_SOLN_BB_INDEX));
}
#endif /* CY_SOLN_VBRG_SCP_EN */

#if CY_SOLN_VIN_OVP_EN
void cy_soln_fault_vin_ovp_en(void)
{
    const ovp_settings_t* ovpCfg = GET_VBUS_OVP_TABLE(get_usbpd_context(CY_SOLN_BB_INDEX));
    uint16_t thres = 0;

    if(true == ovpCfg->enable)
    {
        thres = cy_apply_threshold(get_soln_context()->vinVolt, 
            ovpCfg->threshold);

        Cy_USBPD_Fault_VinOvpEn(get_usbpd_context(CY_SOLN_BB_INDEX), 
            thres,
            cy_cb_soln_fault_vin_ovp, 
            false, 
            VBUS_UVP_MODE_INT_COMP);
    }
}

void cy_soln_fault_vin_ovp_dis(void)
{
    Cy_USBPD_Fault_VinOvpDis(get_usbpd_context(CY_SOLN_BB_INDEX), false);
}
#endif /* CY_SOLN_VIN_OVP_EN */

#if CY_SOLN_VIN_UVP_EN
void cy_soln_fault_vin_uvp_en(void)
{
    uint16_t thres = 0;

    thres = cy_apply_threshold(get_soln_context()->vinVolt, 
        CY_SOLN_VIN_UVP_PER);

    Cy_USBPD_Fault_VinUvpEn(get_usbpd_context(CY_SOLN_BB_INDEX), 
        thres,
        cy_cb_soln_fault_vin_uvp, 
        false, 
        VBUS_UVP_MODE_INT_COMP);

}

void cy_soln_fault_vin_uvp_dis(void)
{
    Cy_USBPD_Fault_VinUvpDis(get_usbpd_context(CY_SOLN_BB_INDEX), false);
}
#endif /* CY_SOLN_VIN_UVP_EN */

void cy_soln_fault_handler(cy_en_soln_fault_t evt)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    cy_en_qi_ptx_ept_reason_t eptReason = CY_QI_PTX_EPT_CAUSE_MAX;

    switch(evt)
    {
#if CY_SOLN_VBRG_OVP_EN
        case CY_SOLN_FAULT_VBRG_OVP:
        {
        	Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT: VBRG_OVP\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VBRG_OVP;
        } 
        break;
#endif /* CY_SOLN_VBRG_OVP_EN */
#if CY_SOLN_VBRG_OCP_EN
        case CY_SOLN_FAULT_VBRG_OCP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT: VBRG_OCP\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VBRG_OCP;
        } 
        break;
#endif /* CY_SOLN_VBRG_OCP_EN */
#if CY_SOLN_VBRG_SCP_EN
        case CY_SOLN_FAULT_VBRG_SCP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT: VBRG_SCP\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VBRG_SCP;
        } 
        break;
#endif /* CY_SOLN_VBRG_SCP_EN */
#if CY_SOLN_OTP_ENABLE
        case CY_SOLN_FAULT_OTP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT: OTP\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_NO_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_OTP;
        }
        break;
#endif /* CY_SOLN_OTP_ENABLE */
#if CY_SOLN_VIN_OVP_EN
        case CY_SOLN_FAULT_VIN_OVP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: VIN_OVP\r\n");
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT HANDLE: PD RECOVERY\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            /* Enqueue VBUS OVP fault event to PD event handling. */
            app_event_handler(get_pdstack_context(CY_SOLN_BB_INDEX), APP_EVT_VBUS_OVP_FAULT, NULL);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VIN_OVP;
        }
        break;
#endif /* CY_SOLN_VIN_OVP_EN */
#if CY_SOLN_VIN_UVP_EN
        case CY_SOLN_FAULT_VIN_UVP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: VIN_UVP\r\n");
            /* Do not treat VIN UVP as PD fault */
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_NO_RETRY);

            cy_sw_timer_start(get_timer_context(), get_soln_context(),
                CY_SOLN_TIMER_FAULT_UVP_FORCE_RECOVER,
                CY_SOLN_TIMER_FAULT_UVP_FORCE_RECOVER_TIME_MS,
                cy_cb_soln_fault_vin_uvp_force_time);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VIN_UVP;
        }
        break;
#endif /* CY_SOLN_VIN_UVP_EN */
#if CY_SOLN_VREG_INRUSH_EN
        case CY_SOLN_FAULT_VREG_INRUSH:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: VREG_INRUSH\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_DEVICE_RESET);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VREG_INRUSH;
        }
        break;
#endif /* CY_SOLN_VREG_INRUSH_EN */
#if CY_SOLN_VDDD_BOD_EN
        case CY_SOLN_FAULT_VDDD_BOD:
        {
            uint16_t vin, vddd;

            /* 
             * Since we are getting power outage, measure and log the
             * VIN and VDDD voltages for any further debug.
             */

            /* VIN is same as VBUS. */
            vin = soln_vbus_get_value(soln_ctx->pdstack_ctx);
            /* We can only use the ADC1 of port 0 to measure VDDD. */
            vddd = Cy_USBPD_Adc_Calibrate(get_usbpd_context(CY_SOLN_BB_INDEX), CY_USBPD_ADC_ID_1);
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL,
                    "\r\nFAULT: VDDD_BOD: VIN = %dmV, VDDD = %dmV ----------------\r\n",
                    vin, vddd);
            cy_soln_fault_state_request(CY_SOLN_FAULT_DEVICE_RESET);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_VDDD_BOD;
        }
        break;
#endif /* CY_SOLN_VDDD_BOD_EN */
#if CY_SOLN_BB_ILIM_EN
        case CY_SOLN_FAULT_BB_ILIM:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: BUCK_BOOST_ILIM\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_BB_ILIM;
        }
        break;
#endif /* CY_SOLN_BB_ILIM_EN */

        case CY_SOLN_FAULT_CC_OVP:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: CC_OVP\r\n");
            cy_soln_fault_state_request(CY_SOLN_FAULT_QI_STOP_RETRY);
            eptReason = CY_QI_PTX_EPT_CAUSE_FAULT_CC_OVP;
        }
        break;

        default:
        break;
    }
    
    if (CY_QI_PTX_EPT_CAUSE_MAX != eptReason)
    {
        soln_ctx->qistack_ctx->qiStat.ptxEptReason = eptReason;
        soln_ctx->qistack_ctx->ptrAppCbk->app_event_handler(soln_ctx->qistack_ctx, CY_QI_APP_EVT_PTX_EPT_REASON);
    }
}

void cy_soln_fault_state_request(cy_en_soln_fault_state_t state)
{
    get_soln_context()->faultCtx.state = state;
}

static uint8_t cy_soln_measure_ntc(void)
{
    uint8_t temperature;

    /* 
     * Disconnect existing ADFT AMUX before measurement and 
     * restore it once measurement is done.
     */
    soln_adft_backup();
    soln_adft_set(ADFT_NONE, ASK_OUT_PORT, ASK_OUT_PIN);
    temperature = ccg_get_sensor_temperature(get_pdstack_context(CY_SOLN_BB_INDEX), 0);
    soln_adft_restore();
    return temperature;
}

static uint8_t cy_soln_measure_internal_temp(void)
{
    return Cy_USBPD_Adc_MeasureInternalTemp(get_usbpd_context(CY_SOLN_BB_INDEX),
        CY_USBPD_ADC_ID_0,
        CY_USBPD_ADC_INPUT_BJT);
}

static void cy_soln_fault_poll_task(void)
{
    cy_en_soln_fault_context_t * faultCtx = &get_soln_context()->faultCtx;
    const cy_stc_fault_protect_cfg_t * faultCfg =
        get_wireless_fault_config(get_qistack_context()->ptrCfg);
	
	/** if UVP has not been acted upon by Source Do force recovery */
    if(true == get_soln_context()->faultCtx.UVPForceRecExpired)
    {
        get_soln_context()->faultCtx.UVPForceRecExpired = false;
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT HANDLE: PD_RECOVER for UVP\r\n");
        Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
        Cy_QiStack_Stop(get_soln_context()->qistack_ctx);
        cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
        /*
        * Try a PD Hard Reset to recover from fault.
        * If not successful (not in PD contract), try Type-C error recovery.
        */
        if (Cy_PdStack_Dpm_SendHardReset(get_pdstack_context(CY_SOLN_BB_INDEX),
                CY_PDSTACK_HARDRES_REASON_NONE) != CY_PDSTACK_STAT_SUCCESS)
        {
            Cy_PdStack_Dpm_SendTypecCommand(get_pdstack_context(CY_SOLN_BB_INDEX),
                CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY,
                NULL);
        }
    }

    if(true == faultCtx->faultPollPending)
    {
#if CY_SOLN_OTP_ENABLE
        /* Measure NTC only when there is no active ASK */
        if(false == get_qistack_context()->qiCommStat.askBmc.isActive)
        {
            faultCtx->faultPollPending = false;

            /* Check NTC OTP entry/exit */
            if(0 != faultCfg->faultCoilOTPTrigThr)
            {
                get_soln_context()->ntcTemp = cy_soln_measure_ntc();

                if(get_soln_context()->ntcTemp >= faultCfg->faultCoilOTPTrigThr)
                {
                    if(false == faultCtx->otpNtc)
                    {
                        faultCtx->otpNtc = true;
                        cy_soln_fault_handler(CY_SOLN_FAULT_OTP);
                    }
                }
                if((true == faultCtx->otpNtc) && 
                    (get_soln_context()->ntcTemp <= faultCfg->faultCoilOTPRelThr))
                {
                    faultCtx->otpNtc = false;
                    cy_soln_fault_state_request(CY_SOLN_FAULT_QI_RECOVER);
                }
            }

            /* Check BJT OTP entry/exit */
            if(0 != faultCfg->faultSysOTPTrigThr)
            {
                get_soln_context()->dieTemp = cy_soln_measure_internal_temp();
                
                if(get_soln_context()->dieTemp >= faultCfg->faultSysOTPTrigThr)
                {
                    if(false == faultCtx->otpBjt)
                    {
                        faultCtx->otpBjt = true;
                        cy_soln_fault_handler(CY_SOLN_FAULT_OTP);
                    }
                }
                if((true == faultCtx->otpBjt) && 
                    (get_soln_context()->dieTemp <= faultCfg->faultSysOTPRelThr))
                {
                    faultCtx->otpBjt = false;
                    cy_soln_fault_state_request(CY_SOLN_FAULT_QI_RECOVER);
                }
            }            
        }
#endif /* CY_SOLN_OTP_ENABLE */
    }

    /* Run polling task timer */
    if(false == cy_sw_timer_is_running(get_timer_context(), CY_SOLN_TIMER_FAULT_POLL))
    {
        faultCtx->faultPollPending = false;

        cy_sw_timer_start(get_timer_context(), get_soln_context(),
            CY_SOLN_TIMER_FAULT_POLL,
            CY_SOLN_TIMER_FAULT_POLL_TIME_MS,
            cy_cb_soln_fault_poll_time);
    }
}

void cy_soln_fault_task(void)
{
    cy_en_soln_fault_context_t * faultCtx = &get_soln_context()->faultCtx;
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    uint32_t state;
    (void)state;

    /* Handle fault requests */
    switch(faultCtx->state)
    {
        case CY_SOLN_FAULT_NONE:
        {
            /* No faults active, run Qi */
        }
        break;
        case CY_SOLN_FAULT_QI_STOP_RETRY:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT HANDLE: QI_STOP_AND_RETRY\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            /* Start recovery timer */
            cy_sw_timer_start(get_timer_context(), get_soln_context(),
                CY_SOLN_TIMER_FAULT_RETRY,
                CY_SOLN_TIMER_FAULT_RETRY_TIME_MS,
                cy_cb_soln_fault_retry_time);
            cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
        }
        break;
        case CY_SOLN_FAULT_QI_STOP_NO_RETRY:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT HANDLE: QI_STOP_AND_NO_RETRY\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
        }
        break;
        case CY_SOLN_FAULT_QI_RECOVER:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT HANDLE: QI_RECOVER\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT_RECOVER);
            Cy_QiStack_Start(soln_ctx->qistack_ctx);
            cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
        }
        break;

#if !CY_SOLN_QI_OVERRIDE_PD

        case CY_SOLN_FAULT_PD_RECOVER:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFAULT HANDLE: PD_RECOVER\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
            /*
             * Try a PD Hard Reset to recover from fault.
             * If not successful (not in PD contract), try Type-C error recovery.
             */
            if (Cy_PdStack_Dpm_SendHardReset(get_pdstack_context(CY_SOLN_BB_INDEX),
                    CY_PDSTACK_HARDRES_REASON_NONE) != CY_PDSTACK_STAT_SUCCESS)
            {
                Cy_PdStack_Dpm_SendTypecCommand(get_pdstack_context(CY_SOLN_BB_INDEX),
                    CY_PDSTACK_DPM_CMD_TYPEC_ERR_RECOVERY,
                    NULL);
            }
        }
        break;

#endif /* CY_SOLN_QI_OVERRIDE_PD */

        case CY_SOLN_FAULT_DEVICE_RESET:
        {
            Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT HANDLE: DEVICE_RESET\r\n");
            Cy_QiStack_LED_State_Set(get_qistack_context(), CY_QI_LED_STATE_FAULT);
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            cy_soln_fault_state_request(CY_SOLN_FAULT_NONE);
            state = Cy_SysLib_EnterCriticalSection();
            /* 
             * Enter critical section for soft reset trigger.
             * Backup any data if required and trigger a soft reset
             * as VDDD is not in safe range for silicon to work. 
             */
            Cy_SysLib_DelayUs(100);
            NVIC_SystemReset();
        }
        break;
        default:
        break;
    }

    /* Run polling fault tasks */
    cy_soln_fault_poll_task();
}

/* [] END OF FILE */
