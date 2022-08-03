/***************************************************************************//**
* \file solution.c
* \version 1.00
*
* Source file of solution layer.
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
#include "solution.h"
#include "fault.h"
#include "config.h"
#include "app.h"
#include "pdo.h"
#include "cy_pdstack_utils.h"
#include "cy_usbpd_buck_boost.h"
#include "cy_qistack_console.h"
#include "cy_qistack_utils.h"
#if CCG_HPI_ENABLE
#include "hpi.h"
#endif /* CCG_HPI_ENABLE */
/*******************************************************************************
*                        Static Function Definitions
*******************************************************************************/

static void soln_led_task(
        cy_stc_qi_context_t *qiCtx);

uint16_t soln_cbl_mon_trim_adj(cy_stc_usbpd_context_t * ctx, 
    uint8_t gain, 
    uint16_t csa_vref)
{
    int32_t result, vref, v1, v2, c1, c2;
    uint8_t port = ctx->port;

    /*
     * What we get from ADC is Vref. we need 10x scale to compensate for integer division.
     * Also, we need to scale this Vref to match the Rsense value used for actual measurement.
     * TRIM data is stored assuming 10mOhm Rsense.
     */
    vref = (csa_vref * 10u * 100u) / ctx->vbusCsaRsense;

    if (gain == CSA_GAIN_VALUE_HIGH)
    {
        v1 = CBL_GAIN150_TRIM_P250A_ROOM(port);
        v2 = CBL_GAIN150_TRIM_P500A_ROOM(port);
        c1 = 250u;
        c2 = 500u;
    }
    else /* This is CSA_GAIN_VALUE_LOW */
    {
        if (csa_vref < CBL_GAIN40_TRIM_2A_ROOM(port))
        {
            v1 = CBL_GAIN40_TRIM_1A_ROOM(port);
            v2 = CBL_GAIN40_TRIM_2A_ROOM(port);
            c1 = 1000u;
            c2 = 2000u;
        }
        else if (csa_vref < CBL_GAIN40_TRIM_3A_ROOM(port))
        {
            v1 = CBL_GAIN40_TRIM_2A_ROOM(port);
            v2 = CBL_GAIN40_TRIM_3A_ROOM(port);
            c1 = 2000u;
            c2 = 3000u;
        }
        else
        {
            v1 = CBL_GAIN40_TRIM_3A_ROOM(port);
            v2 = CBL_GAIN40_TRIM_4A_ROOM(port);
            c1 = 3000u;
            c2 = 4000u;
        }
    }

    /*
     * Now we adjust for CSA error based on TRIM. We do curve fitting. We are going to assume 
     * linear relation in multiple steps: we use slope generated based on nearest stored TRIM
     * data.
     * c in the equations is the current and v is the vref voltage. 
     * The basic formula applied is:
     * cp = c1 + dc/dv * (vp - v1). dc = c2 - c1 and dv = v2 - v1. cp is the actual current
     * and vp is the sampled Vref. We do a 10x scaling overall to all numberator element to
     * adjust for loss via integer division.
     * In 2 byte case, the TRIM data is 10X. So we need to scale accordingly.
     */
    vref *= 10;

    result = c1 * 10 + (((c2 - c1) * (vref - (v1 * 10))) / (v2 - v1));

    /* Round off for result. */
    result = (result + 5) / 10;
    
    if (result < 0)
    {
        result = 0;
    }

    /* Current in mA units */
    return (uint16_t)result;
}

/* 
 * This function enables ADFT on AMUXA for a selected signal 
 * from the list.
 */
void soln_adft_set(cy_en_soln_adft_t adft, GPIO_PRT_Type * port, uint32_t pin)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();

    CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl, 
        PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 0u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl, 
        PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 0u);

    switch(adft)
    {
        case ADFT_P0_VOUT_MON:
            CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 7u);
            break;
        case ADFT_P1_VOUT_MON:
            CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 7u);
            break;
        case ADFT_P0_VOUT_CBL:
            CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 1u);
            break;
        case ADFT_P1_VOUT_CBL:
            CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 1u);
            break;
        case ADFT_P0_VOUT_CC:
            CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 13u);
            break;
        case ADFT_P1_VOUT_CC:
            CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl, 
                PDSS_CSA_SCP_0_CTRL_CSA_ADFT_CTRL, 13u);
            break;
        default:
            break;
    }

    if (adft == ADFT_NONE)
    {
        Cy_GPIO_SetHSIOM(port, pin, HSIOM_SEL_GPIO);
        Cy_GPIO_SetDrivemode(port, pin, CY_GPIO_DM_ANALOG);
    }
    else
    {
        Cy_GPIO_SetHSIOM(port, pin, HSIOM_SEL_AMUXA);
        Cy_GPIO_SetDrivemode(port, pin, CY_GPIO_DM_ANALOG);
    }

    if(soln_ctx->adftBackup == false)
    {
        soln_ctx->adft = adft;
        soln_ctx->adft_port = port;
        soln_ctx->adft_pin = pin;
    }
}

/*
 * This function keeps backup of previous ADFT signal and pin.
 */
void soln_adft_backup(void)
{
    get_soln_context()->adftBackup = true;
}

/*
 * This function restores previous ADFT signal and pin.
 */
void soln_adft_restore(void)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();

    soln_ctx->adftBackup = false;
    soln_adft_set(soln_ctx->adft,
            soln_ctx->adft_port,
            soln_ctx->adft_pin);
}

/* 
 * Sub-micro second delay function.
 * 800nS is implicit delay with 48MHz SYS_CLK.
 * Each cycle is 21nS.
 * 1600nS is implicit delay with 24MHz SYS_CLK.
 * Each cycle is 42nS.
 */
void soln_delay_sys_clk_cycles(uint32_t sys_clk_cnt)
{
    Cy_SysLib_DelayCycles((uint32)sys_clk_cnt);
}

/*******************************************************************************
*                              Function Definitions
*******************************************************************************/
static void cy_cb_soln_auto_logs_time(cy_timer_id_t id, void * context)
{
#if CY_QI_AUTOMATION_DEBUG_EN
    get_soln_context()->runAutmoationTask = true;
#endif
}

void soln_policy_init(cy_stc_soln_policy_ctx_t * soln_ctx,
        cy_stc_qi_context_t * qistack_ctx,
        cy_stc_pdstack_context_t * pdstack_ctx)
{
    /* Initialize solution policy */
    (void)memset ((void *)soln_ctx, 0, sizeof(cy_stc_soln_policy_ctx_t));

    /* Validate the parameters */
    if ((soln_ctx == NULL) || (qistack_ctx == NULL) || (pdstack_ctx == NULL))
    {
        return;
    }

    soln_ctx->qistack_ctx = qistack_ctx;
    soln_ctx->pdstack_ctx = pdstack_ctx;
    soln_ctx->uartSCBInstance = UART_HW;

#if CY_SOLN_QI_DYNAMIC_PD_EN
    soln_ctx->pdoRequest = CY_PD_VSAFE_5V;
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */

    soln_ctx->runAutmoationTask = false;

    cy_sw_timer_start(  get_timer_context(), 
                        get_soln_context(),
                        CY_SOLN_TIMER_AUTOMATION_LOGS,
                        CY_SOLN_TIMER_AUTO_LOGS_TIME_MS,
                        cy_cb_soln_auto_logs_time);
}

void soln_automation_logs_task(cy_stc_soln_policy_ctx_t * soln_ctx)
{
    uint16_t vInput; 

#if CY_QI_AUTOMATION_DEBUG_EN
    if (true == soln_ctx->runAutmoationTask)
    {
        soln_ctx->runAutmoationTask = false;      
        vInput = soln_vbus_get_value(soln_ctx->pdstack_ctx);  
                         
        Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nVIN_V = %dmV\r\n", vInput); 
        Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nNTC TEMP = %d'C\r\n", soln_ctx->ntcTemp);
        Cy_Console_Printf(soln_ctx->qistack_ctx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nDIE TEMP = %d'C\r\n", soln_ctx->dieTemp);
        

        cy_sw_timer_start(  get_timer_context(), 
                        get_soln_context(),
                        CY_SOLN_TIMER_AUTOMATION_LOGS,
                        CY_SOLN_TIMER_AUTO_LOGS_TIME_MS,
                        cy_cb_soln_auto_logs_time);
    }
#endif
}

void soln_policy_task(cy_stc_soln_policy_ctx_t * soln_ctx)
{
#if CY_SOLN_QI_DYNAMIC_PD_EN
    cy_en_soln_pdo_t newPdoRequest = soln_ctx->pdoRequest;
    cy_en_pdstack_status_t dpmStatus = CY_PDSTACK_STAT_FAILURE;
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */
    cy_stc_pdstack_pd_packet_t* srcCap = soln_ctx->pdstack_ctx->dpmStat.srcCapP;
    uint16_t maxVinVolt = 0;
    uint8_t src_pdo_index = 0;

    /* Handle solution policy */

#if CY_SOLN_QI_OVERRIDE_PD

    soln_ctx->vinVolt = soln_vbus_get_value(soln_ctx->pdstack_ctx);

    if(soln_ctx->vinVolt < CY_SOLN_MIN_VIN_FOR_QI)
    {
        if(soln_ctx->qistack_ctx->qiStat.run == true)
        {
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
        }
    }
    else
    {
        if(soln_ctx->qistack_ctx->qiStat.run == false)
        {
            Cy_QiStack_Start(soln_ctx->qistack_ctx);
        }    
    }

#else

    if(true == soln_ctx->contract)
    {
        if(true == soln_ctx->contractUpdate)
        {
            soln_ctx->contractUpdate = false;
            cy_sw_timer_stop(soln_ctx->qistack_ctx->ptrTimerContext, 
                CY_SOLN_TIMER_FAULT_UVP_FORCE_RECOVER);

            /** 
             * Check for maximum SRC supported PDO and update Qi PTx CAP.
             */
            for(src_pdo_index = 0u; src_pdo_index < srcCap->len; src_pdo_index++)
            {
                if (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                {
                    maxVinVolt = (srcCap->dat[src_pdo_index].fixed_src.voltage * 50);
                }
            }

            if(maxVinVolt == CY_PD_VSAFE_5V)
            {
                /**
                 * Qi is not supported with 5V contract
                 */
                if(true == soln_ctx->qistack_ctx->qiStat.run)
                {
                    Cy_QiStack_Stop(soln_ctx->qistack_ctx);
                    Cy_QiStack_LED_State_Set(soln_ctx->qistack_ctx, CY_QI_LED_STATE_END_CHARGE_PROGRESS);
                }
            }
            else
            {
                /**
                 * Qi is supported.
                 * But, update capability as per VIN support.
                 */
                if(maxVinVolt <= CY_PD_VSAFE_9V)
                {
                    Cy_QiStack_Update_Guaranteed_Max_Power(soln_ctx->qistack_ctx, 
                        CY_QI_EPP_5W_VAL, CY_QI_EPP_5W_VAL);
                }
                else
                {
                    Cy_QiStack_Update_Guaranteed_Max_Power(soln_ctx->qistack_ctx, 
                        CY_QI_EPP_15W_VAL, CY_QI_EPP_15W_VAL);
                }

                if(false == soln_ctx->qistack_ctx->qiStat.run)
                {
                    Cy_QiStack_Start(soln_ctx->qistack_ctx);
                }
            }
        }
    }
    else
    {
        if(true == soln_ctx->qistack_ctx->qiStat.run)
        {
            Cy_QiStack_Stop(soln_ctx->qistack_ctx);
            Cy_QiStack_LED_State_Set(soln_ctx->qistack_ctx, CY_QI_LED_STATE_END_CHARGE_PROGRESS);
        }
    }

#if CY_SOLN_QI_DYNAMIC_PD_EN
    if (true == get_wireless_vin_config(soln_ctx->qistack_ctx->ptrCfg)->pdPowerOptimizationEnable)
    {        
        /** Dynamic PD contract monitoring in Qi active condition */
        if(true == soln_ctx->qistack_ctx->qiStat.run)
        {
            if((true == soln_ctx->objDetected) ||
                (true == soln_ctx->objRemoved) ||
                (true == soln_ctx->cepRcvd) ||
                (true == soln_ctx->qiReset))
            {
                if(true == soln_ctx->qiReset)
                {
                    soln_ctx->qiReset = false;
                    /* Do nothing */
                }

                if(true == soln_ctx->objDetected)
                {
                    soln_ctx->objDetected = false;
                    newPdoRequest = CY_SOLN_PDO_VSAFE9V;
                }

                if(true == soln_ctx->objRemoved)
                {
                    soln_ctx->objRemoved = false;
                    newPdoRequest = CY_SOLN_PDO_VSAFE5V;
                }

                /* Once CEP is received, evaluate Vin and Vout and initiate PD change request */
                if(true == soln_ctx->cepRcvd)
                {
                    soln_ctx->cepRcvd = false;

                    if(soln_ctx->pdoRequest == CY_SOLN_PDO_VSAFE9V)
                    {
                        if(soln_ctx->qistack_ctx->qiPowerStat.coilVolt >= CY_SOLN_VIN_LVL_VSAFE_9V_EXIT)
                        {
                            newPdoRequest = CY_SOLN_PDO_VSAFE15V;
                        }
                    }
                    else if(soln_ctx->pdoRequest == CY_SOLN_PDO_VSAFE15V)
                    {
                        if(soln_ctx->qistack_ctx->qiPowerStat.coilVolt >= CY_SOLN_VIN_LVL_VSAFE_15V_EXIT)
                        {
                            newPdoRequest = CY_SOLN_PDO_VSAFE20V;
                        }

                        if(soln_ctx->qistack_ctx->qiPowerStat.coilVolt <= CY_SOLN_VIN_LVL_VSAFE_9V_ENTRY)
                        {
                            newPdoRequest = CY_SOLN_PDO_VSAFE9V;
                        }
                    }
                    else if(soln_ctx->pdoRequest == CY_SOLN_PDO_VSAFE20V)
                    {
                        if(soln_ctx->qistack_ctx->qiPowerStat.coilVolt <= CY_SOLN_VIN_LVL_VSAFE_15V_ENTRY)
                        {
                            newPdoRequest = CY_SOLN_PDO_VSAFE15V;
                        }
                    }
                }

                if(newPdoRequest != soln_ctx->pdoRequest)
                {
                    /** Update latest PDO contract */
                    soln_ctx->pdoRequest = newPdoRequest;

                    soln_ctx->dpmPending = true;
                }
            }

            /** Run DPM command if pending */
            if(true == soln_ctx->dpmPending)
            {
                dpmStatus = Cy_PdStack_Dpm_SendPdCommand(get_pdstack_context(CY_SOLN_BB_INDEX), 
                    CY_PDSTACK_DPM_CMD_GET_SRC_CAP, 
                    NULL, false, NULL);

                if(CY_PDSTACK_STAT_SUCCESS == dpmStatus)
                {
                    soln_ctx->dpmPending = false;
                }
                else
                {
                    /** Keep dpm Pending to retry */
                }
            }
        }
        else
        {
            /** Nothing to handle */
        }
    }
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */
#endif /* CY_SOLN_QI_OVERRIDE_PD */

    /* Run LED task independent of Qi state */
    soln_led_task(soln_ctx->qistack_ctx);
    soln_automation_logs_task(soln_ctx);
}

void soln_policy_sleep(cy_stc_soln_policy_ctx_t * soln_ctx)
{
    uint32_t state;
    bool dpm_slept = false;

    /* If solution tasks are idle, enter sleep mode for power saving. */
    state = Cy_SysLib_EnterCriticalSection();

    /* Check if Qi stack is IDLE */
    if(true == Cy_QiStack_Is_DeepSleep_Alowed(soln_ctx->qistack_ctx))
    {
        /* Setup DPM hardware for deep sleep mode. */
        Cy_PdStack_Dpm_PrepareDeepSleep(soln_ctx->pdstack_ctx, &dpm_slept);

        /* Check if 
         * DPM PD stack is IDLE
         * HPI I2C is IDLE
         * Legacy charging is IDLE.
         */
        if ((true == dpm_slept) 
#if CCG_HPI_ENABLE
            &&
            (true == i2c_scb_is_idle(HPI_SCB_INDEX)) 
#endif /* CCG_HPI_ENABLE */
#if BATTERY_CHARGING_ENABLE
            &&
            (0 == bc_get_status(soln_ctx->pdstack_ctx)->bc_evt) &&
            (false == cy_sw_timer_is_running(soln_ctx->pdstack_ctx->ptrTimerContext, APP_BC_GENERIC_TIMER1)) &&
            (false == cy_sw_timer_is_running(soln_ctx->pdstack_ctx->ptrTimerContext, APP_BC_GENERIC_TIMER2)) &&
            (false == cy_sw_timer_is_running(soln_ctx->pdstack_ctx->ptrTimerContext, APP_BC_DP_DM_DEBOUNCE_TIMER)) &&
            (false == cy_sw_timer_is_running(soln_ctx->pdstack_ctx->ptrTimerContext, APP_BC_DETACH_DETECT_TIMER))
#endif /* BATTERY_CHARGING_ENABLE */
           )
        {
            cy_sw_timer_enter_sleep(soln_ctx->pdstack_ctx->ptrTimerContext);

            /**
             * Configure Refgen block to use Deepsleep Reference input instead of Bandgap
             * reference which is not available in deep sleep.
             */
            PDSS0->refgen_0_ctrl &= ~(PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);
            Cy_SysLib_DelayUs(20);

            /**
             * Enter deep sleep once references are set.
             */
            Cy_SysPm_CpuEnterDeepSleepNoCallbacks();

            /**
             * Configure Refgen block to use Bandgap Reference input instead of Deep sleep
             * reference.
             */
            Cy_SysLib_DelayUs(20);
            PDSS0->refgen_0_ctrl |= (PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_SEL |
                PDSS_REFGEN_0_CTRL_REFGEN_VREFIN_MON_SEL);

            /* Resume DPM */
            Cy_PdStack_Dpm_Resume(soln_ctx->pdstack_ctx, &dpm_slept);

            /* Initialize the PWM after wakeup */
            Cy_TCPWM_TriggerReloadOrIndex(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_MASK);
            Cy_TCPWM_Counter_Enable(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_NUM);
            Cy_TCPWM_TriggerStart(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_MASK);
        }
    }

    Cy_SysLib_ExitCriticalSection(state);
    return;
}

bool soln_vconn_enable(cy_stc_pdstack_context_t * port_ctx, uint8_t channel)
{
    CY_UNUSED_PARAMETER(port_ctx);
    CY_UNUSED_PARAMETER(channel);
    return false;
}

void soln_vconn_disable(cy_stc_pdstack_context_t * port_ctx, uint8_t channel)
{
    CY_UNUSED_PARAMETER(port_ctx);
    CY_UNUSED_PARAMETER(channel);
}

bool soln_vconn_is_present(cy_stc_pdstack_context_t * port_ctx)
{
    CY_UNUSED_PARAMETER(port_ctx);
    return false;
}

bool soln_vbus_is_present(cy_stc_pdstack_context_t * port_ctx, uint16_t volt, int8_t per)
{
    bool retVal = false;

    /* 
     * WLC1 uses VIN as Type-C VBUS for PD sink configuration. 
     * Also, this signal is mapped to Port 1 USBPD instance VBUS_IN path.
     */
    if(apply_threshold((int32_t)volt, per) <
            Cy_USBPD_Adc_MeasureVbusIn(get_usbpd_context(TYPEC_PORT_1_IDX),
            CY_USBPD_ADC_ID_0,
            CY_USBPD_ADC_INPUT_AMUX_B))
    {
        retVal = true;
    }
    else
    {
        retVal = false;
    }

    return retVal;
}

uint16_t soln_vbus_get_value(cy_stc_pdstack_context_t * port_ctx)
{
    /* 
     * WLC1 uses VIN as Type-C VBUS for PD sink configuration. 
     * Also, this signal is mapped to Port 1 USBPD instance VBUS_IN path.
     */
    return Cy_USBPD_Adc_MeasureVbusIn(get_usbpd_context(TYPEC_PORT_1_IDX),
            CY_USBPD_ADC_ID_0,
            CY_USBPD_ADC_INPUT_AMUX_B);
}

uint16_t soln_coil_voltage_measure(cy_stc_qi_context_t * qiCtx)
{
    /* 
     * WLC1 uses Port0 VBUS point as coil Vbridge point.
     */
    return Cy_USBPD_Adc_MeasureVbus(get_usbpd_context(TYPEC_PORT_0_IDX),
            CY_USBPD_ADC_ID_0,
            CY_USBPD_ADC_INPUT_AMUX_B);
}

void soln_psnk_enable (cy_stc_pdstack_context_t * port_ctx)
{
    return;
}

void soln_psnk_disable (cy_stc_pdstack_context_t * port_ctx, cy_pdstack_sink_discharge_off_cbk_t snk_discharge_off_handler)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    uint32_t state = 0;

    state = Cy_SysLib_EnterCriticalSection();

    /* Update VIN variables */
    soln_ctx->vinVolt = CY_PD_VSAFE_0V;
    soln_ctx->vinVoltContract = CY_PD_VSAFE_0V;
    
    soln_ctx->vinDir = CY_SOLN_VIN_NO_TRANSITION;

    /* Disable the VIN faults */

    /* Disable the old fault levels first */
#if CY_SOLN_VIN_OVP_EN
    cy_soln_fault_vin_ovp_dis();
#endif /* CY_SOLN_VIN_OVP_EN */
#if CY_SOLN_VIN_UVP_EN
    cy_soln_fault_vin_uvp_dis();
#endif /* CY_SOLN_VIN_UVP_EN */

    Cy_SysLib_ExitCriticalSection(state);

    return;
}

#if CY_SOLN_QI_DYNAMIC_PD_EN
static cy_pd_pd_do_t form_rdo(cy_stc_pdstack_context_t* context, uint8_t pdo_no, bool capMisMatch, bool giveBack)
{
    cy_stc_soln_policy_ctx_t * solnCtx = get_soln_context();
#if (CY_PD_REV3_ENABLE)
    const cy_stc_pd_dpm_config_t* dpm = &(context->dpmConfig);
#endif /* CY_PD_REV3_ENABLE */
    cy_pd_pd_do_t snkRdo;

    snkRdo.val = 0u;
    snkRdo.rdo_gen.noUsbSuspend = context->dpmStat.snkUsbSuspEn;
    snkRdo.rdo_gen.usbCommCap = context->dpmStat.snkUsbCommEn;
    snkRdo.rdo_gen.capMismatch = capMisMatch;
    snkRdo.rdo_gen.objPos = pdo_no;
    snkRdo.rdo_gen.giveBackFlag = (capMisMatch) ? false : giveBack;
    snkRdo.rdo_gen.opPowerCur = solnCtx->pd_op_cur;
    snkRdo.rdo_gen.minMaxPowerCur = solnCtx->pd_max_cur;

    if (
            (snkRdo.rdo_gen.giveBackFlag == false) &&
            (snkRdo.rdo_gen.opPowerCur > snkRdo.rdo_gen.minMaxPowerCur)
       )
    {
        snkRdo.rdo_gen.minMaxPowerCur = snkRdo.rdo_gen.opPowerCur;
    }

#if (CCG6_SROM_CODE_ENABLE || CY_PD_REV3_ENABLE)
    if(dpm->specRevSopLive >= CY_PD_REV3)
    {
        snkRdo.rdo_gen.unchunkSup = true;
    }
#endif /* (CCG6_SROM_CODE_ENABLE | CY_PD_REV3_ENABLE) */

    return snkRdo;
}
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */

/*
 * This function evaluates SRC_CAP as required by solution.
 */
void soln_eval_src_cap(cy_stc_pdstack_context_t* context, const cy_stc_pdstack_pd_packet_t* srcCap, cy_pdstack_app_resp_cbk_t app_resp_handler)
{
#if CY_SOLN_QI_DYNAMIC_PD_EN
    if (false == get_wireless_vin_config(get_soln_context()->qistack_ctx->ptrCfg)->pdPowerOptimizationEnable)
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */
    {         
        /** Call default src_cap handler */
        eval_src_cap(context, srcCap, app_resp_handler);

    #if ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN))

        uint8_t port = context->port;
        cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
        uint16_t vin_volt_old = 0;
        /* 
        * It's a new PD contract and new VIN voltage.
        * Set new VIN protection levels.
        */
        vin_volt_old = soln_ctx->vinVolt;
        soln_ctx->vinVolt = gl_contract_voltage[port];
      
        /*
        * If sink voltage did not change, then do not update faults.
        * In upward transition:
        * Enable OVP before voltage transition, UVP after transition.
        * In downward transition:
        * Enable UVP before voltage transition, OVP after transition.
        */
        if(soln_ctx->vinVolt != vin_volt_old)
        {
            if(soln_ctx->vinVolt > vin_volt_old)
            {
                soln_ctx->vinDir = CY_SOLN_VIN_UP_TRANSITION;
#if CY_SOLN_VIN_OVP_EN
                cy_soln_fault_vin_ovp_en();
#endif /* CY_SOLN_VIN_OVP_EN */
            }
            else
            {
                soln_ctx->vinDir = CY_SOLN_VIN_DOWN_TRANSITION;
#if CY_SOLN_VIN_UVP_EN
                cy_soln_fault_vin_uvp_en();
#endif /* CY_SOLN_VIN_UVP_EN */
            }
        }
        #endif /* ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN)) */
    }
#if CY_SOLN_QI_DYNAMIC_PD_EN
    else
    {
        cy_pd_pd_do_t* snkPdo = (cy_pd_pd_do_t*)&context->dpmStat.curSnkPdo[0u];
        cy_stc_soln_policy_ctx_t * solnCtx = get_soln_context();
        uint8_t port = context->port;
        uint8_t src_pdo_index, snk_pdo_index;

        uint16_t src_vsafe5_cur = srcCap->dat[0u].fixed_src.maxCurrent; /* Source max current for first PDO */
        cy_pd_pd_do_t snkRdo;
        bool match = false;

        for(snk_pdo_index = 0u; snk_pdo_index < context->dpmStat.curSnkPdocount; snk_pdo_index++)
        {
            for(src_pdo_index = 0u; src_pdo_index < srcCap->len; src_pdo_index++)
            {
                if (srcCap->dat[src_pdo_index].fixed_src.supplyType == CY_PDSTACK_PDO_FIXED_SUPPLY)
                {
                    if(((srcCap->dat[src_pdo_index].fixed_src.voltage * 50u) == solnCtx->pdoRequest) &&
                        ((snkPdo[snk_pdo_index].fixed_snk.voltage * 50u) == solnCtx->pdoRequest))
                    {
                        solnCtx->pd_op_cur = GET_MIN(srcCap->dat[src_pdo_index].fixed_src.maxCurrent,
                            snkPdo[snk_pdo_index].fixed_snk.opCurrent);
                        solnCtx->pd_max_cur = GET_MIN(srcCap->dat[src_pdo_index].fixed_src.maxCurrent,
                            snkPdo[snk_pdo_index].fixed_snk.opCurrent);
                        snkRdo = form_rdo(context, (src_pdo_index + 1u), false,
                                (context->dpmStat.curSnkMaxMin[snk_pdo_index] & CY_PD_GIVE_BACK_MASK));
                        match = true;
                        break;
                    }
                }
            }

            if(match == true)
            {
                break;
            }
        }

        if(match == false)
        {
            if(0u == snk_pdo_index)
            {
                /* Capability mismatch: Ask for vsafe5v PDO with CapMismatch */
                snkRdo = form_rdo(context, 1u, true, false);
                solnCtx->contract = false;
            }
            else
            {
                /* 
                * Stay in same previous contract.
                * This is to avoid resetting contract to VSAFE_5V in case 
                * of higher PDO unavailability.
                */
                snkRdo = context->dpmStat.snkRdo;
            }
        }

        /** Update PD variables */
        solnCtx->pd_volt = snkPdo[snk_pdo_index].fixed_snk.voltage;
        solnCtx->pd_op_cur = snkPdo[snk_pdo_index].fixed_snk.opCurrent;
        solnCtx->pd_pwr = div_round_up(
                solnCtx->pd_volt * solnCtx->pd_op_cur, 500u);

        if(src_vsafe5_cur < solnCtx->pd_op_cur)
        {
            /* SNK operation current can't be bigger than SRC maxCurrent */
            solnCtx->pd_op_cur = src_vsafe5_cur;
        }

        solnCtx->pd_max_cur = context->dpmStat.curSnkMaxMin[0u];

        /** 
         * Update UVP threshold proactively as sink_set_voltage is 
         * not invoked by pdstack for downward voltage transitions.
         * If contract is not successful, fault levels are updated in
         * PD negotiaion complete event.
         */
        if((solnCtx->pd_volt * 50u) < solnCtx->vinVolt)
        {
            solnCtx->vinVolt = (solnCtx->pd_volt * 50u);
    #if CY_SOLN_VIN_UVP_EN
            cy_soln_fault_vin_uvp_en();
    #endif /* CY_SOLN_VIN_UVP_EN */
        }

        (app_get_resp_buf(port))->respDo = snkRdo;
        app_resp_handler(context, app_get_resp_buf(context->port));
    }
#endif /* CY_SOLN_QI_DYNAMIC_PD_EN */
}

void soln_psnk_set_current (cy_stc_pdstack_context_t * port_ctx, uint16_t cur_10mA)
{
    return;
}

void soln_psnk_set_voltage (cy_stc_pdstack_context_t * port_ctx, uint16_t volt_mV)
{

    uint32_t state = 0;

    state = Cy_SysLib_EnterCriticalSection();

#if ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN))
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    uint16_t vin_volt_old = 0;
    if(CY_PD_VSAFE_0V != volt_mV)
    {
        /* 
        * It's a new PD contract and new VIN voltage.
        * Set new VIN protection levels.
        */
        vin_volt_old = soln_ctx->vinVolt;
        soln_ctx->vinVolt = volt_mV;
      
        /*
        * If sink voltage did not change, then do not update faults.
        * In upward transition:
        * Enable OVP before voltage transition, UVP after transition.
        * In downward transition:
        * Enable UVP before voltage transition, OVP after transition.
        */
        if(volt_mV != vin_volt_old)
        {
            if(volt_mV > vin_volt_old)
            {
                soln_ctx->vinDir = CY_SOLN_VIN_UP_TRANSITION;
#if CY_SOLN_VIN_OVP_EN
                cy_soln_fault_vin_ovp_en();
#endif /* CY_SOLN_VIN_OVP_EN */
            }
            else
            {
                soln_ctx->vinDir = CY_SOLN_VIN_DOWN_TRANSITION;
#if CY_SOLN_VIN_UVP_EN
                cy_soln_fault_vin_uvp_en();
#endif /* CY_SOLN_VIN_UVP_EN */
            }
        }
    }
    else
    {
#if CY_SOLN_VIN_OVP_EN
            cy_soln_fault_vin_ovp_dis();
#endif /* CY_SOLN_VIN_OVP_EN */

#if CY_SOLN_VIN_UVP_EN
            cy_soln_fault_vin_uvp_dis();
#endif /* CY_SOLN_VIN_UVP_EN */
    }
    
#endif /* ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN)) */

    Cy_SysLib_ExitCriticalSection(state);

    return;
}

void soln_pd_event_handler(cy_stc_pdstack_context_t * port_ctx, cy_en_pdstack_app_evt_t evt, const void *data)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
#if ((CCG_HPI_PD_ENABLE == 1u) || (CCG_HPI_AUTO_CMD_ENABLE))
    /* Pass the callback to HPI */
	CALL_MAP(hpi_pd_event_handler)(port_ctx, evt, data);
#endif /* ((CCG_HPI_PD_ENABLE == 1u) || (CCG_HPI_AUTO_CMD_ENABLE)) */
#if ((BATTERY_CHARGING_ENABLE) && (QC_AFC_SNK_EN))
    const bc_status_t *bc_stat = bc_get_status(port_ctx);
#endif /* ((BATTERY_CHARGING_ENABLE) && (!QC_AFC_CHARGING_DISABLED)) */
    uint16_t vinVolt = 0;

    if (APP_EVT_PKT_RCVD != evt)
    {
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_DEBUG_LV1, "\r\nPD Event: 0x%x", evt);
    }

    switch(evt)
    {
        case APP_EVT_DISCONNECT:
        case APP_EVT_HARD_RESET_RCVD:
        case APP_EVT_HARD_RESET_SENT:
        {
            soln_ctx->contract = false;
        }
        break;

        case APP_EVT_CONNECT:
        {
#if ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN))
            /*
             * In upward transition:
             * OVP before voltage transition, UVP after transition.
             * In downward transition:
             * UVP before voltage transition, OVP after transition.
             */
            if(CY_SOLN_VIN_DOWN_TRANSITION == soln_ctx->vinDir)
            {
#if CY_SOLN_VIN_OVP_EN
                cy_soln_fault_vin_ovp_en();
#endif /* CY_SOLN_VIN_OVP_EN */
            }
            else if(CY_SOLN_VIN_UP_TRANSITION == soln_ctx->vinDir)
            {
#if CY_SOLN_VIN_UVP_EN
                cy_soln_fault_vin_uvp_en();
#endif /* CY_SOLN_VIN_UVP_EN */
            }
#endif /* ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN)) */

            break;
        }
        case APP_EVT_PD_CONTRACT_NEGOTIATION_COMPLETE:
        {
            vinVolt = port_ctx->dpmStat.contract.maxVolt;
            Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, 
                "\r\nUSBPD VIN: %dmV", vinVolt);

            /**
             * Take latest contract voltage, either success or failure.
             * This is to keep vinVolt data updated and enable faults accordingly.
             */
            soln_ctx->vinVolt = vinVolt;
            soln_ctx->vinVoltContract = vinVolt;

#if ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN))
            /**
             * Re-enable VIN OVP and UVP faults as per contracted voltage.
             */
#if CY_SOLN_VIN_OVP_EN
            cy_soln_fault_vin_ovp_en();
#endif /* CY_SOLN_VIN_OVP_EN */
#if CY_SOLN_VIN_UVP_EN
            cy_soln_fault_vin_uvp_en();
#endif /* CY_SOLN_VIN_UVP_EN */
#endif /* ((CY_SOLN_VIN_OVP_EN) || (CY_SOLN_VIN_UVP_EN)) */

            if (CY_PDSTACK_CONTRACT_NEGOTIATION_SUCCESSFUL == port_ctx->peStat.contractEvtResp.status)
            {
                soln_ctx->contractUpdate = true;
                soln_ctx->contract = true;
            }
            else
            {
                soln_ctx->contract = false;
            }
            break;
        }
        case APP_EVT_BC_DETECTION_COMPLETED:
        {
            vinVolt = app_get_status(port_ctx->port)->psnk_volt;

            Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, 
                "\r\nLegacy VIN: %dmV", vinVolt);

            /** Check if voltage is in required range */
            if(CY_SOLN_MIN_VIN_FOR_QI < vinVolt)
            {
                soln_ctx->vinVolt = vinVolt;
                soln_ctx->vinVoltContract = vinVolt;
                soln_ctx->contractUpdate = true;
                soln_ctx->contract = true;
            }
            else
            {
                soln_ctx->contract = false;
            }

            break;
        }
        case APP_EVT_CC_OVP:
        {
            cy_soln_fault_handler(CY_SOLN_FAULT_CC_OVP);
            break;
        }

        default:
        {
            break;
        }
    }
}

static void Cy_Cb_LED_timr(
    cy_timer_id_t id, void * ctx)
{
    CY_UNUSED_PARAMETER(id);
    cy_stc_qi_context_t *qiCtx = (cy_stc_qi_context_t *)ctx;
    qiCtx->qiStat.stLed.runLEDtask = true;
}

/**
 * Static function definitions.
 */
static void soln_led_task(
        cy_stc_qi_context_t *qiCtx)
{
    if (true == qiCtx->qiStat.stLed.runLEDtask)
    {
        qiCtx->qiStat.stLed.runLEDtask = false;
        qiCtx->qiStat.stLed.ledTimeout = 0;

        switch (qiCtx->qiStat.stLed.presentLEDstatus)
        {
            case CY_QI_LED_STATE_INIT:
            {
                qiCtx->qiStat.stLed.blueLEDstatus = false;
                if (false == qiCtx->qiObjectStat.fod)
                {
                    qiCtx->qiStat.stLed.redLEDstatus = false;
                }
            }
            break;
            
            case CY_QI_LED_STATE_POWER_ON:  /* Blink RED and BLUE */
            {
                if (qiCtx->qiStat.stLed.previousLEDstatus != qiCtx->qiStat.stLed.presentLEDstatus)
                {
                    qiCtx->qiStat.stLed.redLEDstatus = true;
                    qiCtx->qiStat.stLed.blueLEDstatus = true;
                    qiCtx->qiStat.stLed.ledBlinkCount = 0;
                }
                else
                {
                    qiCtx->qiStat.stLed.redLEDstatus = !qiCtx->qiStat.stLed.redLEDstatus;
                    qiCtx->qiStat.stLed.blueLEDstatus = !qiCtx->qiStat.stLed.blueLEDstatus;
                }
                if (qiCtx->qiStat.stLed.ledBlinkCount < CY_QI_LED_BLINK_COUNT_POWER_ON)
                {
                    qiCtx->qiStat.stLed.ledBlinkCount++;
                    qiCtx->qiStat.stLed.ledTimeout = CY_QI_TIMEOUT_LED_POWER_ON;
                }
                else
                {
                    qiCtx->qiStat.stLed.ledBlinkCount = 0;
                    qiCtx->qiStat.stLed.redLEDstatus = false;
                    qiCtx->qiStat.stLed.blueLEDstatus = false;
                }
            }
            break;
            
            case CY_QI_LED_STATE_CHARGING_PROGRESS: /* Solid BLUE */
            {
                qiCtx->qiStat.stLed.blueLEDstatus = true;
                qiCtx->qiStat.stLed.redLEDstatus = false;
            }
            break;
            
            case CY_QI_LED_STATE_FOD: /* Solid RED */
            {
               if (false == ((CY_QI_PLOSS_REASON_EXCEED_THRES == qiCtx->qiObjectStat.powerLoss.pwrlossFodReason) &&
                   (0 != qiCtx->qiObjectStat.powerLoss.powerCycleCount)))
                {
                    qiCtx->qiStat.stLed.blueLEDstatus = false;
                    qiCtx->qiStat.stLed.redLEDstatus = true;
                }
            }
            break;

            case CY_QI_LED_STATE_FAULT: /* Solid RED */
            {
                qiCtx->qiStat.stLed.blueLEDstatus = false;
                qiCtx->qiStat.stLed.redLEDstatus = true;
            }
            break;
            
            case CY_QI_LED_STATE_NO_FOD: /* Turn off RED */
            case CY_QI_LED_STATE_FAULT_RECOVER: /* Turn off RED */
            {
                qiCtx->qiStat.stLed.redLEDstatus = false;
            }
            break;
            
            case CY_QI_LED_STATE_VALID_RX_NO_COMM: /* Blink BLUE */
            {
                if (qiCtx->qiStat.stLed.previousLEDstatus != qiCtx->qiStat.stLed.presentLEDstatus)
                {
                    qiCtx->qiStat.stLed.blueLEDstatus = true;
                    qiCtx->qiStat.stLed.redLEDstatus = false;
                    qiCtx->qiStat.stLed.ledBlinkCount = 0; 
                }
                else
                {
                    qiCtx->qiStat.stLed.blueLEDstatus = !qiCtx->qiStat.stLed.blueLEDstatus;
                }
                qiCtx->qiStat.stLed.ledTimeout = CY_QI_TIMEOUT_LED_OBJ_NO_ASK; 
            }
            break;
            
            case CY_QI_LED_STATE_RX_EPT_REQ_FAILURE: /* Blink RED */
            {
                if (qiCtx->qiStat.stLed.previousLEDstatus != qiCtx->qiStat.stLed.presentLEDstatus)
                {
                    qiCtx->qiStat.stLed.redLEDstatus = true;
                    qiCtx->qiStat.stLed.blueLEDstatus = false;
                    qiCtx->qiStat.stLed.ledBlinkCount = 0;
                }
                else
                {
                    qiCtx->qiStat.stLed.redLEDstatus = !qiCtx->qiStat.stLed.redLEDstatus;
                }
                qiCtx->qiStat.stLed.ledTimeout = CY_QI_TIMEOUT_LED_RX_EPT;
            }
            break;
            
            case CY_QI_LED_STATE_END_CHARGE_PROGRESS:  /* BLUE off */
            case CY_QI_LED_STATE_RX_EPT_REQ_SUCCESS:   /* Turn off RED and BLUE */
            default:
            {
                qiCtx->qiStat.stLed.blueLEDstatus = false;
                qiCtx->qiStat.stLed.redLEDstatus = false;
            }
            break;
        }
        
        qiCtx->ptrAppCbk->led_set_pin_value(0, qiCtx->qiStat.stLed.redLEDstatus);
        qiCtx->ptrAppCbk->led_set_pin_value(1, qiCtx->qiStat.stLed.blueLEDstatus);

        qiCtx->qiStat.stLed.previousLEDstatus = qiCtx->qiStat.stLed.presentLEDstatus;
        
        if (0 != qiCtx->qiStat.stLed.ledTimeout)
        {
            cy_sw_timer_start(qiCtx->ptrTimerContext, qiCtx,
                                        CY_QI_TIMER_STATUS_LED_TIME_ID,
                                        qiCtx->qiStat.stLed.ledTimeout,
                                        Cy_Cb_LED_timr);
        }
    }
}

static void soln_qi_display_packet_header(struct cy_stc_qi_context *qiCtx)
{
    switch (qiCtx->qiCommStat.askCfg.askPkt.header)
    {            
        case CY_QI_ASK_RECEIVED_POWER_RP8:  /*0x04*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRP8");
            break;
        }
            
        case CY_QI_ASK_CHARGE_STATUS:  /*0x05*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nCHR_STS");
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PCH:  /*0x06*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nCFG_PCH");
            break;
        }
                    
        case CY_QI_ASK_GENERAL_REQUEST:  /*0x07*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nGRQ: ");
            switch(qiCtx->qiCommStat.askCfg.askPkt.msg[0])
            {
                case CY_QI_GRQ_PTX_ID:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "PTx ID \r\n");
                    break;
                }
                case CY_QI_GRQ_PTX_CAP:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "PTx Cap \r\n");
                    break;
                }
                case CY_QI_GRQ_PTX_XCAP:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "PTx Ext Cap \r\n");
                    break;
                }
                case CY_QI_GRQ_ADC:   
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "ADC \r\n");
                    break;
                }
                case CY_QI_GRQ_AUX_DATA_EVEN_1:
                case CY_QI_GRQ_AUX_DATA_ODD_1:
                case CY_QI_GRQ_AUX_DATA_EVEN_2:
                case CY_QI_GRQ_AUX_DATA_ODD_2:                
                case CY_QI_GRQ_AUX_DATA_EVEN_3:
                case CY_QI_GRQ_AUX_DATA_ODD_3:    
                case CY_QI_GRQ_AUX_DATA_EVEN_4:
                case CY_QI_GRQ_AUX_DATA_ODD_4:    
                case CY_QI_GRQ_AUX_DATA_EVEN_5:
                case CY_QI_GRQ_AUX_DATA_ODD_5:    
                case CY_QI_GRQ_AUX_DATA_EVEN_6:
                case CY_QI_GRQ_AUX_DATA_ODD_6:    
                case CY_QI_GRQ_AUX_DATA_EVEN_7:
                case CY_QI_GRQ_AUX_DATA_ODD_7:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "AUX Data \r\n");
                    break;
                }
                default:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "Unknown Request \r\n");
                    break;
                }
            }
            break;
        }
                
        case CY_QI_ASK_RENEGOTIATE:  /*0x09*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRENEGO");
            break;
        }
                        
        case CY_QI_ASK_DATA_STREAM_RESP:  /*0x15*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nDSR");
            break;
        }
                    
        case CY_QI_ASK_SPECIFIC_REQUEST:  /*0x20*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nSRQ: ");
            
            switch(qiCtx->qiCommStat.askCfg.askPkt.msg[0])
            {
                case CY_QI_SRQ_END_NEGOTIATION:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "End Neg\r\n");
                    break;
                }
                case CY_QI_SRQ_GUARANTEED_POWER:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "Guaranteed Pwr\r\n");
                    break;
                }
                case CY_QI_SRQ_RCVD_POWER_REPORTING:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "PwrPktType\r\n");
                    break;
                }
                case CY_QI_SRQ_FSK_PARAMS:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "FskParams\r\n");
                    break;
                }
                case CY_QI_SRQ_REFERENCE_POWER:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "Referene Pwr\r\n");
                    break;
                }
                case CY_QI_SRQ_REPING_DELAY:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "Reping Delay\r\n");
                    break;
                }
                case CY_QI_SRQ_RECALIB:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "Recalib\r\n");
                    break;
                }
                default:
                {
                    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "ND\r\n");
                    break;
                }
            }
            break;
        }
                    
        case CY_QI_ASK_FOD_STATUS:  /*0x22*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFOD_STAT");
            break;
        }
                        
        case CY_QI_ASK_DATA_AUX_DATA_CTRL:  /*0x25*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nADC");
            break;
        }
                
        case CY_QI_ASK_RECEIVED_POWER_RP:  /*0x31*/ 
        {/* Printed in Automation */
            break;
        }
                
        case CY_QI_ASK_CONFIGURATION:  /*0x51*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nCFG");
            break;
        }
                    
        case CY_QI_ASK_WPID_MSB:  /*0x54*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nWPTD_M");
            break;
        }
                            
        case CY_QI_ASK_WPID_LSB:  /*0x55*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nWPTD_L");
            break;
        }
                            
        case CY_QI_ASK_IDENTIFICATION:  /*0x71*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nID");
            break;
        }
                    
        case CY_QI_ASK_EXTENDED_IDENTIFICATION:  /*0x81*/ 
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nEXT_ID");
            break;
        }
            
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_1:  /*0x16*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_1:  /*0x17*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_2:  /*0x26*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_2:  /*0x27*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_3:  /*0x36*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_3:  /*0x37*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_4:  /*0x46*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_4:  /*0x47*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_5:  /*0x56*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_5:  /*0x57*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_6:  /*0x66*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_6:  /*0x67*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_EVEN_7:  /*0x76*/ 
        {
            break;
        }
                
        case CY_QI_ASK_DATA_AUX_DATA_ODD_7:  /*0x77*/ 
        {
            break;
        }
                
        case CY_QI_ASK_CONFIG_PROP_1:  /*0x18*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_2:  /*0x19*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_3:  /*0x28*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_4:  /*0x29*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_5:  /*0x38*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_6:  /*0x48*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_7:  /*0x58*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_8:  /*0x68*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_9:  /*0x78*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_10:  /*0x84*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_11:  /*0xA4*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_12:  /*0xC4*/ 
        {
            break;
        }
                    
        case CY_QI_ASK_CONFIG_PROP_13:  /*0xE2*/ 
        {
            break;
        }

        default:
        break;
    }
}

void soln_qi_event_handler(
            struct cy_stc_qi_context *qiCtx,
            cy_en_qi_app_evt_t evt
            )
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    CY_UNUSED_PARAMETER(soln_ctx);

    switch(evt)
    {
        case CY_QI_APP_EVT_INIT:
        {
            break;
        }        
        case CY_QI_APP_EVT_START:
        {
            break;
        }
        case CY_QI_APP_EVT_STOP:
        {
            break;
        }
        case CY_QI_APP_EVT_RESET:
        {
            /* Disable VBRG faults */
#if CY_SOLN_VBRG_OVP_EN
            cy_soln_fault_vbrg_ovp_dis();
#endif /* CY_SOLN_VBRG_OVP_EN */

#if CY_SOLN_VBRG_OCP_EN
            cy_soln_fault_vbrg_ocp_dis();
#endif /* CY_SOLN_VBRG_OCP_EN */

#if CY_SOLN_VBRG_SCP_EN
            cy_soln_fault_vbrg_scp_dis();
#endif /* CY_SOLN_VBRG_SCP_EN */

            soln_ctx->objDetected = false;
            soln_ctx->cepRcvd = false;
            soln_ctx->qiReset = true;

            break;
        }
        case CY_QI_APP_EVT_PING_START:
        {
            /* Enable VBRG faults */
#if CY_SOLN_VBRG_OVP_EN
            cy_soln_fault_vbrg_ovp_en();
#endif /* CY_SOLN_VBRG_OVP_EN */

#if CY_SOLN_VBRG_OCP_EN
            cy_soln_fault_vbrg_ocp_en(get_wireless_fault_config(get_qistack_context()->ptrCfg)->faultOCPThr);
#endif /* CY_SOLN_VBRG_OCP_EN */

#if CY_SOLN_VBRG_SCP_EN
            cy_soln_fault_vbrg_scp_en();
#endif /* CY_SOLN_VBRG_SCP_EN */

            break;
        }
        case CY_QI_APP_EVT_PING_PRX_DET:
        {
            break;
        }

        case CY_QI_APP_EVT_OBJ_DET_STARTED:
        {
            /** 
             * Set objDetected here also along with CY_QI_APP_EVT_OBJ_DET
             * to get status as soon as object is in vicinity even if it is not stable yet.
             */
            soln_ctx->objDetected = true;
            break;
        }

        case CY_QI_APP_EVT_OBJ_DET:
        {
            soln_ctx->objDetected = true;
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nOBJ DETECTED\r\n");
            break;
        }
        case CY_QI_APP_EVT_OBJ_REMOVED:
        {
            soln_ctx->objRemoved = true;

            /* If object was detected earlier */
            if (true == qiCtx->qiObjectStat.object)
            {
                Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nOBJ REMOVED\r\n");
            }
            break;
        }
        case CY_QI_APP_EVT_NEG_DONE:
        {            
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nRX_REQ_REF_PWR = %d\r\n", qiCtx->qiStat.pwrParams.referencePower);
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nRX_REQ_GUA_PWR = %d\r\n", qiCtx->qiStat.pwrParams.guaranteedPower);
          
            break;
        }
        case CY_QI_APP_EVT_CEP_RCVD:
        {
            soln_ctx->cepRcvd = true;
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nCEP = %d\r\n", qiCtx->qiPowerStat.pid.cep);
            break;
        }
        case CY_QI_APP_EVT_RPP_RCVD:
        {
            break;
        }
        case CY_QI_APP_EVT_RP0_RCVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRP0\r\n");
            break;
        }
        case CY_QI_APP_EVT_RP1_RCVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRP1\r\n");
            break;
        }
        case CY_QI_APP_EVT_RP2_RCVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRP2\r\n");
            break;
        }
        case CY_QI_APP_EVT_RP4_RCVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRP4\r\n");
            break;
        }
        case CY_QI_APP_EVT_EPT_RCVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nPRx EPT = %d\r\n", qiCtx->qiStat.prxEptReason);
            break;
        }

        case CY_QI_APP_EVT_LED_SET:
        {
            /** Even if CHarging status LED is requested, check whether FOD status is set */
            if ((true == qiCtx->qiObjectStat.fod) && 
                (CY_QI_LED_STATE_CHARGING_PROGRESS == qiCtx->qiStat.stLed.requestedLEDstate))
            {
                qiCtx->qiStat.stLed.requestedLEDstate = CY_QI_LED_STATE_FOD;
            }
            if (qiCtx->qiStat.stLed.requestedLEDstate != qiCtx->qiStat.stLed.presentLEDstatus)
            {
                qiCtx->qiStat.stLed.presentLEDstatus = qiCtx->qiStat.stLed.requestedLEDstate;
                if (qiCtx->qiStat.stLed.previousLEDstatus != qiCtx->qiStat.stLed.presentLEDstatus)
                {
                    Cy_Cb_LED_timr(CY_QI_TIMER_STATUS_LED_TIME_ID, qiCtx);
                }
            }
            break;
        }
        
        case CY_QI_APP_EVT_Q_FACTOR_DATA_READY:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nQ-Peaks: %d\r\n", qiCtx->qiObjectStat.qFactor.qHighPeakCount);
#if CY_QI_AUTOMATION_DEBUG_EN
            if (0 != qiCtx->qiObjectStat.qFactor.qFactorMPA1)
            {
                Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION,   "\r\nQ_MEAS: %d Q_MEAS_MPA1: %d\r\n", 
                                                                            qiCtx->qiObjectStat.qFactor.qFactor,
                                                                            qiCtx->qiObjectStat.qFactor.qFactorMPA1);
            }
            else
            {
                Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION,   "\r\nQ_MEAS: %d\r\n", qiCtx->qiObjectStat.qFactor.qFactor);
            }
#endif
            break;
        }

        case CY_QI_APP_EVT_RES_FREQ_DATA_READY:
        {    
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nFRES_MEAS: %d\r\n", qiCtx->qiObjectStat.qFactor.qFrequency);
#endif
            break;
        }
        case CY_QI_APP_EVT_CFG_RCVD:
        {
            break;
        }
        case CY_QI_APP_EVT_PROP:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nPROP");
            break;
        }
        case CY_QI_APP_EVT_RSVD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nRSVD");
            break;
        }
        case CY_QI_APP_EVT_NEG_EPT:
        {
            break;
        }
        case CY_QI_APP_EVT_NEG_BPP_FAIL:
        {
            break;
        }
        case CY_QI_APP_EVT_RX_SS:
        {            
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nRX_SS = %d\r\n", qiCtx->qiStat.rxSignalStr); 
#endif
            break;
        }

        case CY_QI_APP_EVT_SUDDEN_LOAD_REMOVAL:
        {
            /* Revert coil voltage to ping voltage */
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\n\rSudden Load Removal. Drop voltage to ping voltage.\r\n");
            break;
        }

        case CY_QI_APP_EVT_FREE_AIR_FOD:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFree air FOD\r\n");
            break;
        }
        case CY_QI_APP_EVT_OBJ_FOD_SET:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFO Detected\r\n");
            break;
        }

        case CY_QI_APP_EVT_OBJ_FOD_CLR:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFO Not Detected\r\n");
            break;
        }

        case CY_QI_APP_EVT_ASK_PATH_SWITCH_TIMEOUT:
        {            
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nASK path switch Pkt timeout\r\n");
            break;
        }

        case CY_QI_APP_EVT_ASK_PASS:
        {            
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK = PASS\r\n");
#endif
            soln_qi_display_packet_header(qiCtx);
            break;
        }
        
        case CY_QI_APP_EVT_ASK_NOISE:
        {            
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK = NOISE\r\n");
#endif
            break;
        }

        case CY_QI_APP_EVT_ASK_FAIL:
        {            
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK = FAIL\r\n");
#endif
            break;
        }

        case CY_QI_APP_EVT_FSK_ACK:
        {        
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ACK\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_NAK:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: NAK\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_ND:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ND\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_NULL:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: NULL\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_ATN:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ATN\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_ID:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ID\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_CAP:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: CAP\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_ADC:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ADC\r\n");
            break;
        }

        case CY_QI_APP_EVT_FSK_ADT:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFSK: ADT\r\n");
            break;
        }

        case CY_QI_APP_EVT_RXD_FOD_Q_FACTOR:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nRX_Q_FACTOR = %d\r\n", qiCtx->qiObjectStat.qFactor.rxdQfactor); 
#endif
            break;
        }

        case CY_QI_APP_EVT_RXD_REF_FREQ:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nRX_FRES = %d\r\n", qiCtx->qiObjectStat.qFactor.rxdRefFreq); 
#endif
            break;
        }

        case CY_QI_APP_EVT_PWR_LOSS_PARAM_READY:
        {                                             
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\n\nPRX = %dmW", qiCtx->qiObjectStat.powerLoss.rxPwrMw);
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nPTX = %dmW", qiCtx->qiObjectStat.powerLoss.txPwrMw);
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nVBRG_V = %dmV", qiCtx->qiObjectStat.powerLoss.vBusVolt);
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nVBRG_I = %dmA\r\n", qiCtx->qiObjectStat.powerLoss.vBusCur);
#endif
            break;
        }

        case CY_QI_APP_EVT_PWR_LOSS_THRES_READY:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nPLOSS_THRES = %dmW\r\n", qiCtx->qiObjectStat.powerLoss.threshold); 
#endif
            break;
        }

        case CY_QI_APP_EVT_PWR_LOSS_RETRY_INPROGRES:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nFO_DETECTED = %d\r\n", qiCtx->qiObjectStat.powerLoss.fodCount); 
#endif
            break;
        }
        
        case CY_QI_APP_EVT_PWR_LOSS_PWR_CYCLE_COUNT:
        {
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFO_PLOSS_PWR_CYCLE = %d\r\n", qiCtx->qiObjectStat.powerLoss.powerCycleCount); 
            break;
        }
        case CY_QI_APP_EVT_POWER_LOSS_FOUND:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\r\nPLOSS = %dmW\r\n", qiCtx->qiObjectStat.powerLoss.calcPwrLoss); 
#endif
            break;
        }
        
        case CY_QI_APP_EVT_POWER_LOSS_TX_CALIB_READY:
        {
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rPTX_CAL = %dmW\r\n", qiCtx->qiObjectStat.powerLoss.txPwrCalibMw);
#endif
            break;
        }

        case CY_QI_APP_EVT_PTX_EPT_REASON:
        {
            if ((qiCtx->qiStat.ptxEptReason >= CY_QI_PTX_EPT_CAUSE_FAULT_VBRG_OVP) && 
                (qiCtx->qiStat.ptxEptReason <= CY_QI_PTX_EPT_CAUSE_FAULT_CC_OVP))
            {
                Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "\n\rEPT = %d\r\n", qiCtx->qiStat.ptxEptReason);
            }
            else
            {
#if CY_QI_AUTOMATION_DEBUG_EN
                Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rEPT = %d\r\n", qiCtx->qiStat.ptxEptReason);
#endif
            }
            break;
        }
        
        default:
        {
            break;
        }
    }
}

void soln_qi_hardware_init(
        struct cy_stc_qi_context *qiCtx
        )
{
    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nQi HW Init\r\n");

    /* Initialize Buck boost controller */
    Cy_USBPD_BB_Init(get_usbpd_context(CY_SOLN_BB_INDEX));

    /* Initialize Inverter controller */
    Cy_USBPD_INV_Init(get_usbpd_context(CY_SOLN_INV_INDEX));

    /* 
     * Remove internal GND on P1.4, which is DECODER_INPUT.
     * Also, CSP only single ended terminal for CSA.
     */
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_bat2gnd_prot_cnfg, 
        PDSS_BB_BAT2GND_PROT_CNFG_BAT2GND_PROT_SEL, 5u);

    /* Enable CC comparators */
    PDSS1->cc_ctrl_0  |= PDSS_CC_CTRL_0_CMP_EN;

    /* 
     * Enable Rd for ASK external comparator pull down. 
     */
    PDSS1->cc_ctrl_0 |= PDSS_CC_CTRL_0_RD_CC2_EN;

    /* Power down HSRCP. */
    PDSS1->bb_gdrvo_0_ctrl |= PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSRCP_PD;
    
    /* Set gate drive override. */
    PDSS1->bb_ctrl_ovrd |= (PDSS_BB_CTRL_OVRD_SET_BOOST_OVRD_CTRL |
        PDSS_BB_CTRL_OVRD_SET_BOOST_OVRD_SEL);
    PDSS1->bb_ctrl_ovrd |= (PDSS_BB_CTRL_OVRD_SET_BUCK_OVRD_CTRL |
        PDSS_BB_CTRL_OVRD_SET_BUCK_OVRD_SEL);
    
    /* HSIOM routing for ext_set_boost and ext_set_buck */
    Cy_GPIO_Pin_FastInit(INV_BOOST_PORT, INV_BOOST_PIN, CY_GPIO_DM_STRONG, 0UL, P0_3_USBPD1_EXT_SET_BOOST);
    Cy_GPIO_Pin_FastInit(INV_BUCK_PORT, INV_BUCK_PIN, CY_GPIO_DM_STRONG, 0UL, P0_2_USBPD1_EXT_SET_BUCK);

    /* 20CSA and EA overrides. */
    /* Port 0 overrides */

    /* Enable 20CSA with max filter */
    PDSS0->csa_scp_0_ctrl = ((PDSS0->csa_scp_0_ctrl & 
        ~(PDSS_CSA_SCP_0_CTRL_PD_CSA |
        PDSS_CSA_SCP_0_CTRL_AV1_MASK |
        PDSS_CSA_SCP_0_CTRL_BW_CC_MASK)) |
        (7u << PDSS_CSA_SCP_0_CTRL_AV1_POS) | 
        PDSS_CSA_SCP_0_CTRL_CSA_ISO_N);
    PDSS0->csa_scp_1_ctrl |= PDSS_CSA_SCP_1_CTRL_CSA_EN_IBIAS;

    /* 
     * CC mode is not a requirement for wireless. 
     * But need CC to be enabled to make ASK current path to work.
     * For ASk current path, use lowest CC gain so that load does not hit
     * current foldback. (Rsense = 10mOhm)
     */
    PDSS0->bb_ea_0_ctrl |= PDSS_BB_EA_0_CTRL_BB_EA_EN_CCAMP;
    CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl, PDSS_CSA_SCP_0_CTRL_AV1, 0u);

    /* Port 1 overrides */

    PDSS1->csa_scp_1_ctrl = (PDSS1->csa_scp_1_ctrl &
                ~(CSA_VOUT_CBL_GAIN_MASK << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS)) |
                 (CSA_VOUT_CBL_GAIN_SEL_HIGH << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS);
    PDSS1->csa_scp_1_ctrl = (PDSS1->csa_scp_1_ctrl &
                ~(CSA_VOUT_MON_GAIN_MASK << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS)) |
                 (CSA_VOUT_MON_GAIN_SEL_HIGH << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS);

    PDSS1->csa_scp_0_ctrl = ((PDSS1->csa_scp_0_ctrl &
        ~(PDSS_CSA_SCP_0_CTRL_PD_CSA |
        PDSS_CSA_SCP_0_CTRL_AV1_MASK |
        PDSS_CSA_SCP_0_CTRL_BW_CC_MASK)) |
        (7u << PDSS_CSA_SCP_0_CTRL_AV1_POS) | 
        PDSS_CSA_SCP_0_CTRL_CSA_ISO_N);
    PDSS1->csa_scp_1_ctrl |= PDSS_CSA_SCP_1_CTRL_CSA_EN_IBIAS;

    /* Configure and enable 20CSA (VOUT sense) */
    CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIMS1->trim_bb_20csa_2, PDSS_TRIM_BB_20CSA_2_OS_EL, 0u);

    /* Update Trim values for GM amplifier. Only CC trim needs update. */
    CY_USBPD_REG_FIELD_UPDATE(PDSS_TRIMS1->trim_bb_ea_1, PDSS_TRIM_BB_EA_1_TRIM_GM_CV, 0x16u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_ea_0_ctrl, PDSS_BB_EA_0_CTRL_BB_EA_TRIM_GMBOOSTN_CV, 0u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_ea_0_ctrl, PDSS_BB_EA_0_CTRL_BB_EA_TRIM_GMBOOSTN_CC, 7u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_ea_0_ctrl, PDSS_BB_EA_0_CTRL_BB_EA_TRIM_GMBOOSTP_CV, 0u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_ea_1_ctrl, PDSS_BB_EA_1_CTRL_BB_EA_TRIM_BOOSTP_CC, 7u);
    CY_USBPD_REG_FIELD_UPDATE(PDSS1->bb_ea_1_ctrl, PDSS_BB_EA_1_CTRL_BB_EA_TRIM_PLOAD, 7u);

    /* Enable EA only for CC loop */
    PDSS1->bb_ea_3_ctrl &= ~(PDSS_BB_EA_3_CTRL_BB_ISNK_DAC_CTRL_MASK |
    PDSS_BB_EA_3_CTRL_BB_ISRC_DAC_CTRL_MASK);
    PDSS1->bb_ea_0_ctrl = (PDSS1->bb_ea_0_ctrl &
        ~(PDSS_BB_EA_0_CTRL_BB_EA_ISNK_EN |
        PDSS_BB_EA_0_CTRL_BB_EA_ISRC_EN |
        PDSS_BB_EA_0_CTRL_BB_EA_PD |
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CV |
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CVAMP)) |
        PDSS_BB_EA_0_CTRL_BB_EA_EN_CCAMP;

    /* Remove EA_OUT clamp and make it Vddd/Gnd TTL. Clear <7:0> in T_EA */
    PDSS1->bb_ea_2_ctrl &= ~(0x000000FFu);

    /* Release pull down on EA_OUT */
    PDSS1->bb_ea_2_ctrl &= ~(1u << 15u <<
    PDSS_BB_EA_2_CTRL_BB_EA_T_EA_POS);

    /* ADFT initialization */
    soln_adft_set(ADFT_P1_VOUT_CC, ASK_OUT_PORT, ASK_OUT_PIN);

    /* Inverter PWM initialization */
    /* 
     * Calculate PWM period and configure PWM.
     * -1 count offset to account for the PWM period counter rollover.
     */
    qiCtx->qiStat.invPwmPeriod = (((CY_QI_SYS_CLK_FREQ_KHZ * 10u) +
        (((get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->digPingFreq)/10) >> 1u)) / ((get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->digPingFreq)/10));
    qiCtx->qiStat.invPwm = INV_PWM_HW;
    qiCtx->qiStat.invPwmIndex = INV_PWM_NUM;
    qiCtx->qiStat.invPwmMask = INV_PWM_MASK;

    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_MESSAGE, "\r\ninvPwmPeriod: %d\r\n", qiCtx->qiStat.invPwmPeriod);
    Cy_TCPWM_PWM_Init(INV_PWM_HW, INV_PWM_NUM, &INV_PWM_config);
    Cy_TCPWM_PWM_SetPeriod0(INV_PWM_HW, INV_PWM_NUM, 
        (qiCtx->qiStat.invPwmPeriod - 1u));
    Cy_TCPWM_PWM_SetCompare0(INV_PWM_HW, INV_PWM_NUM, 
        (qiCtx->qiStat.invPwmPeriod >> 1u));

    /* Free running counter initialization and start */
    qiCtx->qiObjectStat.qFactor.freeCounter = FREE_RUN_COUNTER_HW;
    qiCtx->qiObjectStat.qFactor.freeCounterIndex = FREE_RUN_COUNTER_NUM;
    qiCtx->qiObjectStat.qFactor.freeCounterMask = FREE_RUN_COUNTER_MASK;
    Cy_TCPWM_Counter_Init(FREE_RUN_COUNTER_HW, 
        FREE_RUN_COUNTER_NUM, 
        &FREE_RUN_COUNTER_config);
    Cy_TCPWM_TriggerReloadOrIndex(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_MASK);
    Cy_TCPWM_Counter_Enable(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_NUM);
    Cy_TCPWM_TriggerStart(FREE_RUN_COUNTER_HW, FREE_RUN_COUNTER_MASK);
}

static void soln_qi_bmc_rx_spi_intr_handler(void)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();

    qiCtx->qiCommStat.askBmc.scb_int_handler(qiCtx);
}

static void  soln_qi_bmc_rx_cmp_intr_handler(void *context, bool compOut)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(compOut);

    qiCtx->qiCommStat.askBmc.cmp_int_handler(qiCtx);
}

void soln_qi_ask_bmc_init(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t regval;
    CySCB_Type * scb_p = CY_QI_BMC_RX_SPI_INDEX;

    /* Allocate SCB index to be used for BMC */
    qiCtx->qiCommStat.askBmc.scb = CY_QI_BMC_RX_SPI_INDEX;

    /* Enable and configure the SPI clock. */
    scb_p->CTRL = 0;

    /* Configure the SPI. */
    regval = (CY_QI_BMC_RX_SPI_OVS_VALUE << SCB_CTRL_OVS_Pos);
    regval |= (CY_SCB_CTRL_MODE_SPI << SCB_CTRL_MODE_Pos);
#if (CY_QI_BMC_RX_BIT_SPI_OVER_SAMPLE <= 8u)
    regval |= SCB_CTRL_BYTE_MODE_Msk;
#endif
    regval |= SCB_CTRL_ENABLED_Msk;
    scb_p->CTRL = regval;
    
    scb_p->SPI_CTRL = (SCB_SPI_CTRL_CONTINUOUS_Msk | SCB_SPI_CTRL_MASTER_MODE_Msk);

    /* Set the data width to 8 bits. */
    scb_p->TX_CTRL = CY_QI_BMC_RX_BIT_SPI_OVER_SAMPLE - 1;
    scb_p->RX_CTRL = CY_QI_BMC_RX_BIT_SPI_OVER_SAMPLE - 1;

    /*
     * NOTE: Here we are assuming clock source for SCB is same as SCB index.
     * If this is different for a device family, code has to be updated.
     */
    Cy_SysClk_PeriphSetFrequency(BMC_RX_SPI_CLK_HW, BMC_RX_SPI_CLK_NUM,
            CY_QI_BMC_RX_FREQ * CY_QI_BMC_RX_BIT_SPI_OVER_SAMPLE * (CY_QI_BMC_RX_SPI_OVS_VALUE + 1));
    Cy_SysClk_PeriphAssignDivider(CY_QI_BMC_RX_SPI_PORT_INDEX,
            BMC_RX_SPI_CLK_HW, BMC_RX_SPI_CLK_NUM);

    Cy_GPIO_Pin_FastInit(BMC_RX_SPI_MISO_PORT, BMC_RX_SPI_MISO_PIN, 
        CY_GPIO_DM_HIGHZ, 1UL, P0_1_SCB2_SPI_MISO);

    NVIC_DisableIRQ(CY_QI_BMC_RX_SPI_INTR_NUM(CY_QI_BMC_RX_SPI_PORT_INDEX));
    (void)Cy_SysInt_SetVector(CY_QI_BMC_RX_SPI_INTR_NUM(CY_QI_BMC_RX_SPI_PORT_INDEX),
            soln_qi_bmc_rx_spi_intr_handler);
    NVIC_EnableIRQ(CY_QI_BMC_RX_SPI_INTR_NUM(CY_QI_BMC_RX_SPI_PORT_INDEX));

    /* Register Packet start and end detect comparator callback */
    qiCtx->ptrUsbPd1Context->ccDnCbk = soln_qi_bmc_rx_cmp_intr_handler;
}

static void  soln_qi_ask_cc_up_cmp_intr_handler(void *context, bool compOut)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(compOut);
    qiCtx->qiCommStat.askCfg.cc_up_cmp_int_handler(qiCtx);
}

void soln_qi_ask_cc_up_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();
   
    /* UP comparator level */
    CY_USBPD_REG_FIELD_UPDATE(pd->cc_ctrl_0, PDSS_CC_CTRL_0_CMP_UP_VSEL, level);

    /* Filter setting */
    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_RESET |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS);
    CY_USBPD_REG_FIELD_UPDATE(pd->intr1_cfg_vcmp_up_down_ls, 
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG, 
        CY_USBPD_VBUS_FILTER_CFG_POS_EN_NEG_DIS);
    pd->intr1_cfg_vcmp_up_down_ls |= PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS;

    /* Register CC up comp callback */
    qiCtx->ptrUsbPd1Context->ccUpCbk = soln_qi_ask_cc_up_cmp_intr_handler;
    qiCtx->qiCommStat.askCfg.cc_up_cmp_int_handler = cbk;

    /* Enable interrupt */
    pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_MASK_VCMP_UP_CHANGED_MASK;

    Cy_SysLib_ExitCriticalSection(state);
}

static void  soln_qi_cc_up_cmp_intr_handler(void *context, bool compOut)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(compOut);

    qiCtx->qiObjectStat.qFactor.cc_up_cmp_int_handler(qiCtx);
}

void soln_qi_cc_up_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();
   
    /* UP comparator level */
    CY_USBPD_REG_FIELD_UPDATE(pd->cc_ctrl_0, PDSS_CC_CTRL_0_CMP_UP_VSEL, level);

    /* Filter setting */
    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_EN |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_RESET |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_SEL_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS);
    CY_USBPD_REG_FIELD_UPDATE(pd->intr1_cfg_vcmp_up_down_ls, 
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_CFG, 
        CY_USBPD_VBUS_FILTER_CFG_POS_EN_NEG_DIS);
    pd->intr1_cfg_vcmp_up_down_ls |= PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_UP_FILT_BYPASS;

    /* Register CC up comp callback */
    qiCtx->ptrUsbPd1Context->ccUpCbk = soln_qi_cc_up_cmp_intr_handler;
    qiCtx->qiObjectStat.qFactor.cc_up_cmp_int_handler = cbk;

    /* Enable interrupt */
    pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_MASK_VCMP_UP_CHANGED_MASK;

    Cy_SysLib_ExitCriticalSection(state);
}

void soln_qi_cc_up_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();

    /* Keep CC comparators enabled always */

    /* Disable interrupt */
    pd->intr1_mask &= ~PDSS_INTR1_MASK_VCMP_UP_CHANGED_MASK;
    pd->intr1 = PDSS_INTR1_VCMP_UP_CHANGED;
    qiCtx->ptrUsbPd1Context->ccUpCbk = NULL;

    Cy_SysLib_ExitCriticalSection(state);
}

static void  soln_qi_cc_dn_cmp_intr_handler(void *context, bool compOut)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(compOut);

    qiCtx->qiObjectStat.qFactor.cc_dn_cmp_int_handler(qiCtx);
}

void soln_qi_cc_dn_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_cc_up_level_t level,
        void * cbk
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();
   
    /* Down comparator level */
    CY_USBPD_REG_FIELD_UPDATE(pd->cc_ctrl_0, PDSS_CC_CTRL_0_CMP_DN_VSEL, level);

    /* Filter setting */ 
    pd->intr1_cfg_vcmp_up_down_ls &= ~(PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_EN |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_RESET |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_SEL_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG_MASK |
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS);
    CY_USBPD_REG_FIELD_UPDATE(pd->intr1_cfg_vcmp_up_down_ls, 
        PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_CFG, 
        CY_USBPD_VBUS_FILTER_CFG_POS_DIS_NEG_EN);
    pd->intr1_cfg_vcmp_up_down_ls |= PDSS_INTR1_CFG_VCMP_UP_DOWN_LS_VCMP_DN_FILT_BYPASS;

    /* Register CC up comp callback */
    qiCtx->ptrUsbPd1Context->ccDnCbk = soln_qi_cc_dn_cmp_intr_handler;
    qiCtx->qiObjectStat.qFactor.cc_dn_cmp_int_handler = cbk;

    /* Enable interrupt */
    pd->intr1 = PDSS_INTR1_VCMP_DN_CHANGED;
    pd->intr1_mask |= PDSS_INTR1_MASK_VCMP_DN_CHANGED_MASK;

    Cy_SysLib_ExitCriticalSection(state);
}

void soln_qi_cc_dn_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();

    /* Keep CC comparators enabled always */

    /* Disable interrupt */
    pd->intr1_mask &= ~PDSS_INTR1_MASK_VCMP_DN_CHANGED_MASK;
    pd->intr1 = PDSS_INTR1_VCMP_DN_CHANGED;
    qiCtx->ptrUsbPd1Context->ccDnCbk = NULL;

    Cy_SysLib_ExitCriticalSection(state);
}

static void  soln_qi_pds_scp_cmp_intr_handler(void *context, bool compOut)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();
    CY_UNUSED_PARAMETER(context);
    CY_UNUSED_PARAMETER(compOut);

    qiCtx->qiObjectStat.qFactor.pds_scp_cmp_int_handler(qiCtx);
}

void soln_qi_pds_scp_cmp_enable(
        struct cy_stc_qi_context *qiCtx,
        void * cbk
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();

    /* Power up CSA block. */
    pd->csa_scp_0_ctrl &= ~PDSS_CSA_SCP_0_CTRL_PD_CSA;

    /* Enable ZCD (PDS SCP) comparator */

    /* Register interrupt callback */
    qiCtx->ptrUsbPd1Context->pdsScpCbk = soln_qi_pds_scp_cmp_intr_handler;
    qiCtx->qiObjectStat.qFactor.pds_scp_cmp_int_handler = cbk;

    /* Set filter in bypass mode */
    pd->intr15_cfg_pds_scp = ((pd->intr15_cfg_pds_scp &
        ~(PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_CFG_MASK)) |
        (CY_USBPD_VBUS_FILTER_CFG_POS_EN_NEG_DIS << PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_CFG_POS) |
        PDSS_INTR15_CFG_PDS_SCP_SCP_OUT_FILT_BYPASS);

    /* Enable PDS SCP */
    pd->pds_scp_ctrl |= (PDSS_PDS_SCP_CTRL_SCP_EN | 
        PDSS_PDS_SCP_CTRL_SCP_ISO_N);
    Cy_SysLib_DelayUs(20);

    /* Clear and enable PDS SCP interrupt. */
    pd->intr15 = PDSS_INTR15_PDS_SCP_CHANGED;
    pd->intr15_mask |= PDSS_INTR15_PDS_SCP_CHANGED;

    Cy_SysLib_ExitCriticalSection(state);
}

void soln_qi_pds_scp_cmp_disable(
        struct cy_stc_qi_context *qiCtx
        )
{
    PPDSS_REGS_T pd = PDSS1;
    uint32_t state;

    state = Cy_SysLib_EnterCriticalSection();

    /* Disable comparator */
    pd->pds_scp_ctrl &= ~(PDSS_PDS_SCP_CTRL_SCP_EN | 
        PDSS_PDS_SCP_CTRL_SCP_ISO_N);

    /* Disable and clear interrupt */
    pd->intr15_mask &= ~PDSS_INTR15_PDS_SCP_CHANGED;
    pd->intr15 = PDSS_INTR15_PDS_SCP_CHANGED;

    qiCtx->ptrUsbPd1Context->pdsScpCbk = NULL;

    Cy_SysLib_ExitCriticalSection(state);
}

void soln_qi_set_ask_path(
        struct cy_stc_qi_context *qiCtx,
        cy_en_qi_ask_path_t askPath
        )
{
    Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_DEBUG_LV1, "\r\nASK path: %d\r\n", askPath);

    switch(askPath)
    {
        case CY_QI_ASK_PATH_VOLT_H:
        {
            CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl,
                PDSS_CSA_SCP_0_CTRL_AV1, CY_ADFT_VOLT_H_GAIN);
            soln_adft_set(ADFT_P1_VOUT_CC, ASK_OUT_PORT, ASK_OUT_PIN);
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK MODE = VOLT_H");
#endif
            break;
        }
        case CY_QI_ASK_PATH_VOLT_L:
        {
            CY_USBPD_REG_FIELD_UPDATE(PDSS1->csa_scp_0_ctrl,
                PDSS_CSA_SCP_0_CTRL_AV1, CY_ADFT_VOLT_L_GAIN);
            soln_adft_set(ADFT_P1_VOUT_CC, ASK_OUT_PORT, ASK_OUT_PIN);
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK MODE = VOLT_L");
#endif
            break;
        }
        case CY_QI_ASK_PATH_CUR_L:
        {
            CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl,
                PDSS_CSA_SCP_0_CTRL_AV1, CY_ADFT_CUR_L_GAIN);
            soln_adft_set(ADFT_P0_VOUT_CC, ASK_OUT_PORT, ASK_OUT_PIN);
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK MODE = CUR_L");
#endif
            break;
        }
        case CY_QI_ASK_PATH_CUR_H:
        {
            CY_USBPD_REG_FIELD_UPDATE(PDSS0->csa_scp_0_ctrl,
                PDSS_CSA_SCP_0_CTRL_AV1, CY_ADFT_CUR_H_GAIN);
            soln_adft_set(ADFT_P0_VOUT_CC, ASK_OUT_PORT, ASK_OUT_PIN);
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\rASK MODE = CUR_H");
#endif
            break;
        }
        default:
            break;
    }
#if CY_QI_AUTOMATION_DEBUG_EN
            Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_AUTOMATION, "\n\r");
#endif

    qiCtx->qiCommStat.askCfg.askPath = askPath;

    /* Reset error count, since new path is set */
    qiCtx->qiCommStat.askCfg.askPktErrCnt = 0;
}

static void soln_qi_fsk_edge_intr_handler(void)
{
    cy_stc_qi_context_t *qiCtx = get_qistack_context();

    qiCtx->qiCommStat.fskOper.edge_int_handler(qiCtx);

    /* Clear interrupt */
    Cy_TCPWM_ClearInterrupt(FSK_EDGE_COUNTER_HW, 
        FSK_EDGE_COUNTER_NUM, CY_TCPWM_INT_ON_CC_OR_TC);
}

void soln_qi_fsk_oper_init(
        struct cy_stc_qi_context *qiCtx
        )
{
    /* Initialize Edge counter */
    Cy_TCPWM_Counter_Init(FSK_EDGE_COUNTER_HW, FSK_EDGE_COUNTER_NUM, &FSK_EDGE_COUNTER_config);
    NVIC_DisableIRQ(FSK_EDGE_COUNTER_IRQ);
    (void)Cy_SysInt_SetVector(FSK_EDGE_COUNTER_IRQ,
            soln_qi_fsk_edge_intr_handler);
    NVIC_SetPriority(FSK_EDGE_COUNTER_IRQ, 3u);
    NVIC_EnableIRQ(FSK_EDGE_COUNTER_IRQ);
    Cy_TCPWM_SetInterruptMask(FSK_EDGE_COUNTER_HW, FSK_EDGE_COUNTER_NUM, CY_TCPWM_INT_ON_TC);

    /* Keep counter context in FSK */
    qiCtx->qiCommStat.fskOper.edgeCounter = FSK_EDGE_COUNTER_HW;
    qiCtx->qiCommStat.fskOper.edgeCounterIndex = FSK_EDGE_COUNTER_NUM;
    qiCtx->qiCommStat.fskOper.edgeCounterMask = FSK_EDGE_COUNTER_MASK;
    qiCtx->qiCommStat.fskOper.edgeCounterIRQType = FSK_EDGE_COUNTER_IRQ;
}

void soln_qi_inv_fb_enable(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    /* Set Boost drive to PWM */
    PDSS1->bb_ctrl_ovrd |= PDSS_BB_CTRL_OVRD_SET_BOOST_OVRD_SEL;
    Cy_SysLib_DelayUs(20);
    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_qi_inv_fb_disable(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    /* 
     * Set Boost drive to static setting: 
     * LG2 = HIGH, 
     * HG2 = LOW
     */
    PDSS1->bb_ctrl_ovrd = ((PDSS1->bb_ctrl_ovrd &
        ~(PDSS_BB_CTRL_OVRD_SET_BOOST_OVRD_SEL))|
        PDSS_BB_CTRL_OVRD_SET_BOOST_OVRD_VAL);
    Cy_SysLib_DelayUs(20);
    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_qi_inv_send_analog_ping(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint8_t i = 0;

    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    /* 
     * Send Analog PWM pulses in a tight loop.
     * Calculate and use sys clk cycles for required 
     * Analog Ping resonance frequency.
     */
    Cy_GPIO_SetHSIOM(PWM_OUT_PORT, PWM_OUT_PIN, HSIOM_SEL_GPIO);

    while(i < CY_QI_ANALOG_PING_PULSE_COUNT)
    {
        Cy_GPIO_Set(PWM_OUT_PORT, PWM_OUT_PIN);
        soln_delay_sys_clk_cycles(220);
        Cy_GPIO_Clr(PWM_OUT_PORT, PWM_OUT_PIN);
        soln_delay_sys_clk_cycles(220);
        i++;
    }

    Cy_GPIO_SetHSIOM(PWM_OUT_PORT, PWM_OUT_PIN, P0_4_TCPWM_LINE1);

    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_qi_inv_start_digital_ping(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    Cy_GPIO_Pin_FastInit(PWM_OUT_PORT, PWM_OUT_PIN, CY_GPIO_DM_STRONG, 0UL, P0_4_TCPWM_LINE1);
    Cy_TCPWM_PWM_Enable(INV_PWM_HW, INV_PWM_NUM);
    Cy_TCPWM_TriggerStart(INV_PWM_HW, INV_PWM_MASK);
    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_qi_inv_stop_digital_ping(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();

    Cy_TCPWM_PWM_Disable(INV_PWM_HW, INV_PWM_NUM);
    Cy_GPIO_Clr(PWM_OUT_PORT, PWM_OUT_PIN);
    Cy_GPIO_SetHSIOM(PWM_OUT_PORT, PWM_OUT_PIN, HSIOM_SEL_GPIO);
    Cy_SysLib_ExitCriticalSection(intStat);
}

static int16_t soln_idac_to_volt(int16_t idac)
{
    /* The function coverts the iDAC value into voltage. */
    return ((int16_t)CY_PD_VSAFE_5V + (idac * 20u));
}

static void cy_cb_soln_dis_timeout_time(cy_timer_id_t id, void * context)
{
    uint16_t volt_mv;
    cy_stc_usbpd_context_t * usbpd_ctx = get_usbpd_context(CY_SOLN_BB_INDEX);

    /** If VBTR is busy, abort and start new transition */
    if(false == Cy_USBPD_VBTR_IsIdle(usbpd_ctx))
    {
        Cy_USBPD_VBTR_Abort(usbpd_ctx);
        /**
         * Since we are aborting the transition midway, we need to load
         * the voltage with the current voltage information.
         */
        if(true == Cy_USBPD_BB_IsReady(usbpd_ctx))
        {
            volt_mv = soln_idac_to_volt(Cy_USBPD_Hal_Get_Fb_Dac(usbpd_ctx));
            get_qistack_context()->qiPowerStat.coilVoltReq = volt_mv;
            get_qistack_context()->qiPowerStat.coilVolt = volt_mv;
        }
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_DEBUG_LV1, "\r\nVBTR lock up detected. Aborting.\r\n");
    }
    soln_qi_coil_src_discharge_dis(get_qistack_context());
}

void soln_qi_coil_src_enable(
        struct cy_stc_qi_context *qiCtx
        )
{
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();

    /**
     * Make sure discharge is reset.
     */
    soln_qi_coil_src_discharge_dis(get_qistack_context());

    Cy_USBPD_BB_Enable(get_usbpd_context(CY_SOLN_BB_INDEX));
    qiCtx->qiPowerStat.coilVolt = BB_ENABLE_VOUT_VOLT;
    qiCtx->qiPowerStat.coilVoltReq = BB_ENABLE_VOUT_VOLT;
    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_qi_coil_src_disable(
        struct cy_stc_qi_context *qiCtx
        )
{
    cy_stc_usbpd_context_t * usbpd_ctx = get_usbpd_context(CY_SOLN_BB_INDEX);
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();

    /**
     * Make sure discharge is reset.
     */
    soln_qi_coil_src_discharge_dis(get_qistack_context());

    Cy_USBPD_BB_Disable(usbpd_ctx);
#if CY_SOLN_VBTR_EN
    Cy_USBPD_VBTR_Abort(usbpd_ctx);
#endif /** CY_SOLN_VBTR_EN */
    qiCtx->qiPowerStat.coilVolt = 0;
    qiCtx->qiPowerStat.coilVoltReq = 0;
    Cy_SysLib_ExitCriticalSection(intStat);
}

bool soln_qi_coil_src_enable_status(
        struct cy_stc_qi_context *qiCtx
        )
{
    bool retVal = false;

    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    if(Cy_USBPD_BB_IsEnabled(get_usbpd_context(CY_SOLN_BB_INDEX)))
    {
        retVal = true;
    }
    Cy_SysLib_ExitCriticalSection(intStat);

    return retVal;
}

/** Forward declaration */
static void soln_qi_vbtr_multislope(cy_stc_usbpd_context_t * usbpd_ctx);

static void soln_coil_src_vbtr_single_cb(void * callbackCtx, bool value)
{
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackCtx;
    (void)value;
    (void)usbpd_ctx;

    /** Update new voltage value */
    get_qistack_context()->qiPowerStat.coilVolt = get_qistack_context()->qiPowerStat.coilVoltReq;

    /** Stop the transition timeout timer. */
    cy_sw_timer_stop(get_timer_context(), CY_SOLN_TIMER_VBRG_TIMEOUT_ID);

    /**
     * Disable discharge as transition is completed.
     */
    soln_qi_coil_src_discharge_dis(get_qistack_context());
}

static void soln_coil_src_vbtr_cb(void * callbackCtx, bool value)
{
    cy_stc_usbpd_context_t *usbpd_ctx = (cy_stc_usbpd_context_t *)callbackCtx;
    (void)value;

    /* 
     * Transition is completed, call multislope handler.
     */
    soln_qi_vbtr_multislope(usbpd_ctx);
}

static void soln_qi_vbtr_multislope(cy_stc_usbpd_context_t * usbpd_ctx)
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    pwr_params_t *pwr_cfg = pd_get_ptr_pwr_tbl(usbpd_ctx);
    uint32_t hfclk_mhz = (Cy_SysClk_ClkHfGetFrequency() / 1000000);
    int16_t dac_cur; /** Keep signed */
    int16_t dac_new; /** Keep signed */
    int8_t slopeDivider = 1; /** Keep signed */
    uint32_t intStat;
    uint16_t widthMultiplier = 1;

    intStat = Cy_SysLib_EnterCriticalSection();

    if(soln_ctx->vbtrSlopeIndex < CY_QI_VBTR_MULTISLOPE_COUNT)
    {
        soln_ctx->vbtrSlopeIndex++;

        switch(soln_ctx->vbtrSlopeIndex)
        {
            case 1:
            {
                widthMultiplier = CY_QI_VBTR_MULTISLOPE_1_WIDTH_MUL;
                slopeDivider = CY_QI_VBTR_MULTISLOPE_1_TARGET_DIV;
                break;
            }
            case 2:
            {
                widthMultiplier = CY_QI_VBTR_MULTISLOPE_2_WIDTH_MUL;
                slopeDivider = CY_QI_VBTR_MULTISLOPE_2_TARGET_DIV;
                break;
            }
            case 3:
            {
                widthMultiplier = CY_QI_VBTR_MULTISLOPE_3_WIDTH_MUL;
                slopeDivider = CY_QI_VBTR_MULTISLOPE_3_TARGET_DIV; /** Last divider must be 1 */
                break;
            }
            default:
            {
                /** Reach required target with configured step size */
                widthMultiplier = 1;
                slopeDivider = 1;
                break;
            }
        }

        /** Get the current IDAC value */
        dac_cur = Cy_USBPD_Hal_Get_Fb_Dac(usbpd_ctx);

        /** Get new slope target IDAC */
        dac_new = (dac_cur + ((soln_ctx->vbtrDacTarget - dac_cur) / slopeDivider));

        /* We still need to ensure that the DAC is not same. */
        if (dac_cur != dac_new)
        {
            /** Configure VBTR operation */
            Cy_USBPD_VBTR_Config(usbpd_ctx, dac_cur, dac_new, soln_coil_src_vbtr_cb);

            /** Set VBTR step width as required for new slope */
            if(soln_ctx->vbtrDacTarget > dac_cur)
            {
                Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0U, 
                        (hfclk_mhz * (pwr_cfg->vbtr_up_step_width * widthMultiplier)));
            }
            else
            {
                Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_16_BIT, 0U, 
                        (hfclk_mhz * (pwr_cfg->vbtr_down_step_width * widthMultiplier)));
            }

            /** Start VBTR operation */
            Cy_USBPD_VBTR_Start(usbpd_ctx);
        }
        else
        {
            /**
             * Since we have to move state machine forward, we have to
             * proceed with next round. Easiest way is to create the
             * callback again. The only risk here is that we will create
             * a recursive call. It should be ok as there is explicit
             * count for trial.
             */
            soln_coil_src_vbtr_cb(usbpd_ctx, true);
        }
    }
    else
    {
        /** VBTR completed */

        /** Update new voltage value */
    	get_qistack_context()->qiPowerStat.coilVolt = get_qistack_context()->qiPowerStat.coilVoltReq;
        /**
         * Disable discharge as transition is completed.
         */
        soln_qi_coil_src_discharge_dis(get_qistack_context());
        /** Stop the transition timeout timer. */
        cy_sw_timer_stop(get_timer_context(), CY_SOLN_TIMER_VBRG_TIMEOUT_ID);
    }

    Cy_SysLib_ExitCriticalSection(intStat);
}

void soln_policy_correct_ocp_threshold(struct cy_stc_qi_context *qiCtx)
{
    uint16_t ocpFaultThr = 0;
    uint16_t getMinVal = 0;    

    ocpFaultThr = ((uint32_t)((qiCtx->qiStat.pwrParams.guaranteedPower>>1) * 1000 * 1000 *
                    (get_wireless_fault_config(get_qistack_context()->ptrCfg)->maxPowThr/100)) / 
                    qiCtx->qiPowerStat.coilVolt);

    getMinVal = GET_MIN(ocpFaultThr, 
        get_wireless_fault_config(get_qistack_context()->ptrCfg)->faultOCPThr);
        
    cy_soln_fault_vbrg_ocp_en(getMinVal);
}   

void soln_qi_coil_src_set_voltage(
        struct cy_stc_qi_context *qiCtx,
        uint16_t volt_mV,
        bool multiSlope
        )
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    cy_stc_usbpd_context_t * usbpd_ctx = get_usbpd_context(CY_SOLN_BB_INDEX);
    int16_t new_dac = 0;
    int16_t old_dac = 0;

    uint32_t intStat = Cy_SysLib_EnterCriticalSection();

    if(true == Cy_USBPD_BB_IsReady(usbpd_ctx))
    {
        qiCtx->qiPowerStat.coilVoltReq = volt_mV;

        /**
         * Make sure discharge is reset.
         */
        soln_qi_coil_src_discharge_dis(get_qistack_context());

        new_dac = Cy_USBPD_Vbus_GetTrimIdac(usbpd_ctx, volt_mV);
        old_dac  =Cy_USBPD_Hal_Get_Fb_Dac(usbpd_ctx);

#if CY_SOLN_VBTR_EN
        /** If VBTR is busy, abort and start new transition */
        if(false == Cy_USBPD_VBTR_IsIdle(usbpd_ctx))
        {
            Cy_USBPD_VBTR_Abort(usbpd_ctx);
        }
        /** Stop the timeout timer, if it is running. */
        cy_sw_timer_stop(get_timer_context(), CY_SOLN_TIMER_VBRG_TIMEOUT_ID);

        if (old_dac == new_dac)
        {
            /** 
             * Update new voltage value as same idac may match 
             * if number of trimmed steps are lower than actual 20mV steps.
             */
            qiCtx->qiPowerStat.coilVolt = qiCtx->qiPowerStat.coilVoltReq;

            /* Hardware locks up if the same IDAC value is given for VBTR. */
            Cy_SysLib_ExitCriticalSection(intStat);
            return;
        }
#endif /* CY_SOLN_VBTR_EN */

#if CY_SOLN_VBTR_EN
        /**
         * Enable discharge if voltage transition is downward.
         */
        if(old_dac > new_dac)
        {
            soln_qi_coil_src_discharge_en(get_qistack_context());
        }
        if(false == multiSlope)
        {
            /** Use VBTR single slope to reach target voltage. */
            Cy_USBPD_VBTR_SetIdac(usbpd_ctx, new_dac, soln_coil_src_vbtr_single_cb);
        }
        else
        {
            /** 
             * Use VBTR multislope to reach,
             * 50% target voltage in configured step size.
             * Next 25% in 1/4 speed.
             * Next 25% in 1/10 speed.
             */
            soln_ctx->vbtrSlopeIndex = 0;
            soln_ctx->vbtrDacTarget = new_dac;
            soln_qi_vbtr_multislope(usbpd_ctx);
        }
        /**
         * Start voltage transition timeout timer. If for any reason
         * the voltage transition failed, we need to abort the
         * transition. This is applicable only if there is VBTR.
         */
        cy_sw_timer_start(get_timer_context(), 
                get_soln_context(),
                CY_SOLN_TIMER_VBRG_TIMEOUT_ID,
                CY_SOLN_TIMER_VBRG_TIMEOUT,
                cy_cb_soln_dis_timeout_time);
#else /* !CY_SOLN_VBTR_EN */
        /* Set Target idac directly */
        Cy_USBPD_Hal_Set_Fb_Dac(usbpd_ctx, new_dac);
        /** Update new voltage value */
        qiCtx->qiPowerStat.coilVolt = qiCtx->qiPowerStat.coilVoltReq;
#endif /* CY_SOLN_VBTR_EN */
        soln_policy_correct_ocp_threshold(qiCtx);
    }
    Cy_SysLib_ExitCriticalSection(intStat);
}

bool soln_qi_coil_src_ready_status(
        struct cy_stc_qi_context *qiCtx
        )
{
    bool retVal = false;

    uint32_t intStat = Cy_SysLib_EnterCriticalSection();
    if(Cy_USBPD_BB_IsReady(get_usbpd_context(CY_SOLN_BB_INDEX)))
    {
        /**
         * Check VBTR status if enabled.
         * check if the target voltage has reached if VBTR is disabled (5% range)
         */
#if CY_SOLN_VBTR_EN
        if(true == Cy_USBPD_VBTR_IsIdle(get_usbpd_context(CY_SOLN_BB_INDEX)))
#else
        if(true == cy_value_in_range(soln_coil_voltage_measure(qiCtx), 
            qiCtx->qiPowerStat.coilVoltReq, 
            5u))
            
#endif
        {
            retVal = true;
        }
    }
    Cy_SysLib_ExitCriticalSection(intStat);

    return retVal;
}


uint16_t soln_qi_coil_src_get_voltage(
        struct cy_stc_qi_context *qiCtx
        )
{
    return qiCtx->qiPowerStat.coilVolt;
}

uint16_t soln_qi_coil_src_get_current(
        struct cy_stc_qi_context *qiCtx,
        uint8_t avgSamples
        )
{
    uint8_t gain, count, reading;
    cy_stc_usbpd_context_t * ctx = get_usbpd_context(CY_SOLN_BB_INDEX);
    PPDSS_REGS_T pd = ctx->base;
    uint32_t regval;
    cy_en_usbpd_adc_id_t adc = 0;

    /* Sample the VBus current using ADC. */
    uint32_t intStat = Cy_SysLib_EnterCriticalSection();

    /* Ensure that CSA block is enabled. */
    pd->csa_scp_0_ctrl &= ~PDSS_CSA_SCP_0_CTRL_PD_CSA;

    /* Connect ADC_0 to internal AMUX having VBUS_MON signal */
    adc = CY_USBPD_ADC_ID_0;

    /* Select VBUS_MON for ADC measurement */
    pd->amux_ctrl |= (1u << CY_AMUX_ADC_CCG7D_VBUS_MON_SEL);

    /* By default, use the high gain setting. */
    gain = CSA_GAIN_VALUE_HIGH;
    pd->csa_scp_1_ctrl = (pd->csa_scp_1_ctrl & 
            ~(CSA_VOUT_MON_GAIN_MASK << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS)) |
             (CSA_VOUT_MON_GAIN_SEL_HIGH << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS);
    Cy_SysLib_DelayUs(10);

    /* Measure the amplified Vsense. */
    reading = (uint8_t)Cy_USBPD_Adc_Sample(ctx, adc, CY_USBPD_ADC_INPUT_AMUX_B);
    regval = (uint16_t)Cy_USBPD_Adc_LevelToVolt(ctx, adc, reading);

    if (regval > CY_CSA_GAIN_MAX_VALUE)
    {
        gain = CSA_GAIN_VALUE_LOW;
        pd->csa_scp_1_ctrl = (pd->csa_scp_1_ctrl & 
                ~(CSA_VOUT_MON_GAIN_MASK << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS)) |
                 (CSA_VOUT_MON_GAIN_SEL_LOW << PDSS_CSA_SCP_1_CTRL_T_CSA_SCP_EXT_POS);

        Cy_SysLib_DelayUs(10);
        reading = Cy_USBPD_Adc_Sample(ctx, adc, CY_USBPD_ADC_INPUT_AMUX_B);
        regval = (uint16_t)Cy_USBPD_Adc_LevelToVolt(ctx, adc, reading);

        for (count = 1; count < avgSamples; count++)
        {
            reading = Cy_USBPD_Adc_Sample(ctx, adc, CY_USBPD_ADC_INPUT_AMUX_B);
            regval += (uint16_t)Cy_USBPD_Adc_LevelToVolt(ctx, adc, reading);
        }
    }
    else
    {
        for (count = 1; count < avgSamples; count++)
        {
            reading = Cy_USBPD_Adc_Sample(ctx, adc, CY_USBPD_ADC_INPUT_AMUX_B);
            regval += (uint16_t)Cy_USBPD_Adc_LevelToVolt(ctx, adc, reading);
        }
    }

    if (avgSamples != 0)
    {
        regval = (regval + (avgSamples >> 1)) / avgSamples;
    }

    /* Use CBL_MON TRIM adjustment only if it is available. */
    if (CBL_GAIN150_TRIM_P250A_ROOM(ctx->port) != 0)
    {
        regval = soln_cbl_mon_trim_adj(ctx, gain, regval);
    }
    else
    {
        /* Apply 20CSA trims */
        uint16_t offset = (((ctx->trimsConfig.offset_20csa * gain) + (100u >> 1)) / 100u);
        regval = (regval > offset) ? (regval - offset) : 0u;

        regval = (((regval * 10000u) + (((uint32_t)ctx->vbusCsaRsense * gain) >> 1)) / 
                ((uint32_t)ctx->vbusCsaRsense * gain));
    }

    /* Revert the ADFT and MUX configurations. */
    pd->amux_ctrl &= ~(1u << CY_AMUX_ADC_CCG7D_VBUS_MON_SEL);
    Cy_SysLib_DelayUs(10);

    Cy_SysLib_ExitCriticalSection(intStat);

    /* Current in mA units */
    return regval;
}

void soln_qi_coil_src_discharge_en(cy_stc_qi_context_t *qiCtx)
{
    Cy_USBPD_Vbus_DischargeOn(get_usbpd_context(CY_SOLN_BB_INDEX));
    get_soln_context()->coilSrcDishStatus = true;
}

void soln_qi_coil_src_discharge_dis(cy_stc_qi_context_t *qiCtx)
{
    if(true == get_soln_context()->coilSrcDishStatus)
    {
        Cy_USBPD_Vbus_DischargeOff(get_usbpd_context(CY_SOLN_BB_INDEX));
        get_soln_context()->coilSrcDishStatus = false;
    }
}

void soln_qi_led_set_pin_value(uint8_t pin, bool setValue)
{
    /** Red LED */
    if(0 == pin)
    {
        Cy_GPIO_Write(LED_RED_PORT, LED_RED_PIN, setValue);
    }
    /** Blue LED */
    else
    {
        Cy_GPIO_Write(LED_BLUE_PORT, LED_BLUE_PIN, setValue);
    }
}

void soln_auth_init_pwm()
{
	// Initialize PWM for Authentication.
	Cy_TCPWM_Counter_Init(COUNTER_AUTHENTICATION_HW, COUNTER_AUTHENTICATION_NUM, &COUNTER_AUTHENTICATION_config);
	Cy_TCPWM_TriggerReloadOrIndex(COUNTER_AUTHENTICATION_HW, COUNTER_AUTHENTICATION_MASK);
	Cy_TCPWM_Counter_Enable(COUNTER_AUTHENTICATION_HW, COUNTER_AUTHENTICATION_NUM);
	Cy_TCPWM_TriggerStart(COUNTER_AUTHENTICATION_HW, COUNTER_AUTHENTICATION_MASK);
}

void soln_qi_uart_write_string(char_t const string[])
{
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    Cy_SCB_WriteString(soln_ctx->uartSCBInstance, (char *)string);
}

/**
 * This task reads the UART queue buffer until it gets empty and
 * print it on Console
 */
bool cy_soln_uart_task(
       cy_stc_soln_policy_ctx_t *soln_ctx)
{
    bool retVal = false;
    bool status = false;
    CySCB_Type *base = soln_ctx->uartSCBInstance;
    struct cy_stc_qi_context *qiCtx = soln_ctx->qistack_ctx;

    uint16_t loadableData = 0;
    uint8_t data = 0;
    uint32_t fifoSize;

    if ((false == Cy_RingBuf_IsEmpty(&qiCtx->qiStat.qiStUartRingBuf)) &&
        (false == get_qistack_context()->qiCommStat.fskActive))
    {
        fifoSize = Cy_SCB_GetFifoSize(base);
        /* Wait for free space to be available */
        loadableData = fifoSize - Cy_SCB_GetNumInTxFifo(base);

        while (0 != loadableData--)
        {
            status = Cy_RingBuf_Get(&qiCtx->qiStat.qiStUartRingBuf, &data);

            if (true == status)
            {
                if (0 != data)
                {
                    Cy_SCB_WriteTxFifo(base, (uint32_t) data);
                    retVal = true;
                }
                else
                {
                    /* Break if the data is zero */
                    break;
                }
            }
            else
            {
                break;
            }
        }
    }
    return retVal;
}

/* 
 * Inverter initialization.
 * Internally buck-boost, overridden as inverter.
 */
#define BBCLK_KHZ                               (CY_QI_SYS_CLK_FREQ_KHZ / (Cy_SysClk_PeriphGetDivider(CY_SYSCLK_DIV_8_BIT, 6U) + (uint32_t)1u))
#define BBCLK_MHZ                               (BBCLK_KHZ / 1000u)
#define FREQ_KHZ_TO_CYCLE_TIME(freq)            ((BBCLK_KHZ) / (freq))
#define NS_TO_BBCLK_CYCLE_CNT(ns)               ((BBCLK_MHZ * (ns)) / 1000u)
#define BB_SLOPE_COMP_24X                       (24u)
#define BB_SLOPE_COMP_24X_L                     (6800u)
#define BB_SLOPE_COMP_24X_R                     (5000u)
#define BB_HS1_LS2_MIN_DUTY_CYCLE_CLK           (6u)
#define BB_HS1_LS2_MAX_DUTY_CYCLE_CLK_OFST      (3u)
#define BB_BUCK_BOOST                           (0u)
#define BB_FORCED_BUCK                          (1u)
#define BB_FORCED_BOOST                         (2u)

static const uint8_t gl_bb_gdrv_dead_time[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x14};

pwr_params_t pwr_tbl_1 =
{
        .fb_type = 0,
        .vbus_min_volt = 0,
        .vbus_max_volt = 0,
        .vbus_dflt_volt = 0,
        .cable_resistance = 0,
        .vbus_offset_volt = 0,
        .current_sense_res = 50,
        .src_gate_drv_str = 0,
        .vbtr_up_step_width = 40,
        .vbtr_down_step_width = 150,
        .prim_sec_turns_ratio = 0,
        .sr_enable = 0,
        .sr_rise_time = 0,
        .sr_fall_time = 0,
        .sr_async_thresh = 0,
        .sr_supply_doubler = 0,
        .buck_boost_operating_mode = 0,
        .pwm_mode = 4,
        .pwm_min_freq = 80,
        .pwm_max_freq = 80,
        .pwm_fix_freq = 80,
        .max_pwm_duty_cycle = 0,
        .min_pwm_duty_cycle = 0,
        .pwm_gate_pull_up_drv_strnth_LS1 = 7,
        .pwm_gate_pull_up_drv_strnth_LS2 = 7,
        .pwm_gate_pull_up_drv_strnth_HS1 = 7,
        .pwm_gate_pull_up_drv_strnth_HS2 = 7,
        .pwm_dithering_type = 3,
        .pwm_dithering_freq_range = 15,
        .power_inductor_value = 68,
        .peak_current_sense_resistor = 50,
        .phase_angle_control = 0,
        .peak_current_limit = 60,
        .max_pwm_duty_cycle_high_line = 0,
        .vbtr_up_step_width_below_5v = 0,
        .vbtr_down_step_width_below_5V = 0,
        .pwm_max_freq_ex = 0,
        .pwm_gate_pull_down_drv_strnth_LS1 = 7,
        .pwm_gate_pull_down_drv_strnth_LS2 = 7,
        .pwm_gate_pull_down_drv_strnth_HS1 = 7,
        .pwm_gate_pull_down_drv_strnth_HS2 = 7,
        .bbclk_freq = 24,
        .pwm_fix_freq_dith = 0,
        .pwm_dith_spread_cycles = 0x09
};

static uint16_t Cy_USBPD_BB_GetFixFreq(cy_stc_usbpd_context_t *context)
{
    uint16_t fix_freq;
    pwr_params_t *pwr_cfg = &pwr_tbl_1;

    fix_freq = (uint16_t)pwr_cfg->pwm_fix_freq * 5u;

    return fix_freq;
}

static uint16_t bb_ss_per_to_clk (cy_stc_usbpd_context_t *context, uint8_t percent)
{
    (void)context;
    uint32_t clk_cycle;

    clk_cycle = (((BBCLK_KHZ / (BB_SS_PWM_FREQ_KHZ)) * ((uint16_t)percent))/(100u));
    return (uint16_t)clk_cycle;
}

void Cy_USBPD_INV_Init(cy_stc_usbpd_context_t *context)
{
    /* Initialize the BB interface. */
    PPDSS_REGS_T pd = context->base;
    /* 
     * Take all BB parameter with static power structure,
     * except inverter gate drives. Inverter gate drives comes from config table.
     */
    const pwr_params_t *pwr_cfg = &pwr_tbl_1;

    const pwr_params_t *inv_cfg =
        get_wireless_invbridge_config(get_qistack_context()->ptrCfg,
        get_qistack_context()->coilNum);

    uint32_t fix_freq_khz = Cy_USBPD_BB_GetFixFreq(context);
    uint32_t regval;
    uint8_t hs_dead_time;
    uint8_t ls_dead_time;

    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_ff_strtup, PDSS_BBCTRL_FF_STRTUP_BBCTRL_SS_FREQ, /* PRQA S 2985 */
        FREQ_KHZ_TO_CYCLE_TIME(BB_SS_PWM_FREQ_KHZ));
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_clk_ctrl1, PDSS_BBCTRL_CLK_CTRL1_BBCTRL_CLK_FIX_FREQ, /* PRQA S 2985 */
        (FREQ_KHZ_TO_CYCLE_TIME(fix_freq_khz) - 1u));  
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_clk_ctrl1, PDSS_BBCTRL_CLK_CTRL1_BBCTRL_HS1_BLANKING_TIME_TRAIL, 
        NS_TO_BBCLK_CYCLE_CNT(84u));

    /* Configure buck-boost operating mode */
    if (pwr_cfg->buck_boost_operating_mode == BB_FORCED_BUCK)
    {
        pd->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_FORCE_BUCK_MODE;
    }
    else if (pwr_cfg->buck_boost_operating_mode == BB_FORCED_BOOST)
    {
        pd->bbctrl_func_ctrl |= PDSS_BBCTRL_FUNC_CTRL_BBCTRL_FORCE_BOOST_MODE;
    }
    else
    {
        /* Default is buck-boost operation */
    }

    /* Duty cycle configuration for Buck and Boost. */
    pd->bbctrl_buck_sw_ctrl = ((pd->bbctrl_buck_sw_ctrl & 
        ~(PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MIN_DUTY_CYCLE_MASK | 
        PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MAX_DUTY_CYCLE_MASK |
        PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_SS_PW_HS1_MASK)) |
        ((bb_ss_per_to_clk(context, BB_SS_PWM_DUTY_PER) - (uint32_t)1u) <<
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_SS_PW_HS1_POS) |
        (BB_HS1_LS2_MIN_DUTY_CYCLE_CLK << 
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MIN_DUTY_CYCLE_POS) |
        ((FREQ_KHZ_TO_CYCLE_TIME(fix_freq_khz) - 
            BB_HS1_LS2_MAX_DUTY_CYCLE_CLK_OFST) << 
            PDSS_BBCTRL_BUCK_SW_CTRL_BBCTRL_HS1_MAX_DUTY_CYCLE_POS));

    pd->bbctrl_boost_sw_ctrl = ((pd->bbctrl_boost_sw_ctrl & 
        ~(PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_LS2_MIN_DUTY_CYCLE_MASK | 
        PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_LS2_MAX_DUTY_CYCLE_MASK |
        PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_SS_PW_LS2_MASK)) |
        ((bb_ss_per_to_clk(context, BB_SS_PWM_DUTY_PER) - (uint32_t)1u) <<
            PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_SS_PW_LS2_POS) |
        (BB_HS1_LS2_MIN_DUTY_CYCLE_CLK << 
            PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_LS2_MIN_DUTY_CYCLE_POS) |
        ((FREQ_KHZ_TO_CYCLE_TIME(fix_freq_khz) - 
            BB_HS1_LS2_MAX_DUTY_CYCLE_CLK_OFST) << 
            PDSS_BBCTRL_BOOST_SW_CTRL_BBCTRL_LS2_MAX_DUTY_CYCLE_POS));

    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_bb_fixed_cycle_sw_ctrl, PDSS_BBCTRL_BB_FIXED_CYCLE_SW_CTRL_HS1_FIXD_DUTY_CYCLE_BBOOST, /* PRQA S 2985 */
        NS_TO_BBCLK_CYCLE_CNT(2000u));
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_bb_fixed_cycle_sw_ctrl, PDSS_BBCTRL_BB_FIXED_CYCLE_SW_CTRL_LS2_FIXD_DUTY_CYCLE_BBUCK,
        NS_TO_BBCLK_CYCLE_CNT(500u));

    /* Transient load sample and hold configuration */
    pd->bbctrl_clk_ctrl2 = ((pd->bbctrl_clk_ctrl2 &
        ~(PDSS_BBCTRL_CLK_CTRL2_BBCTRL_PW_ILOAD_MASK |
        PDSS_BBCTRL_CLK_CTRL2_BBCTRL_PER_ILOAD_MASK)) |
        (BBCTRL_TRANS_ILOAD_PW << PDSS_BBCTRL_CLK_CTRL2_BBCTRL_PW_ILOAD_POS) |
        (BBCTRL_TRANS_ILOAD_PER << PDSS_BBCTRL_CLK_CTRL2_BBCTRL_PER_ILOAD_POS));

    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_audio_t_cnfg, PDSS_BBCTRL_AUDIO_T_CNFG_BBCTRL_AUDIO_TMIN, (fix_freq_khz / 21u)); /* PRQA S 2985 */
    CY_USBPD_REG_FIELD_UPDATE(pd->bbctrl_audio_t_cnfg, PDSS_BBCTRL_AUDIO_T_CNFG_BBCTRL_AUDIO_TMAX, (fix_freq_khz / 20u));

    /* Configure Buck-Boost gate drivers (HS1, LS1, HS2, LS2) */

    /* HS1 drive strength */
    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvi_1_ctrl), PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_HSDR1_PCONFIG, 
        ((uint32_t)inv_cfg->pwm_gate_pull_up_drv_strnth_HS1));

    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvi_1_ctrl), PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_HSDR1_NCONFIG, 
        ((uint32_t)inv_cfg->pwm_gate_pull_down_drv_strnth_HS1));

    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvi_0_ctrl), PDSS_BB_GDRVI_0_CTRL_BB_GDRVI_LSDR1_PCONFIG, /* PRQA S 2985 */
        (inv_cfg->pwm_gate_pull_up_drv_strnth_LS1));
    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvi_1_ctrl), PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_LSDR1_NCONFIG,  /* PRQA S 2985 */
        (inv_cfg->pwm_gate_pull_down_drv_strnth_LS1));

    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvo_0_ctrl), PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSDR2_PCONFIG, /* PRQA S 2985 */
        (inv_cfg->pwm_gate_pull_up_drv_strnth_HS2));
    CY_USBPD_REG_FIELD_UPDATE((pd->bb_gdrvo_0_ctrl), PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSDR2_NCONFIG, 
        (inv_cfg->pwm_gate_pull_down_drv_strnth_HS2));

    /* LS2 drive strength */
    CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvo_1_ctrl, PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_LSDR2_PCONFIG, 
        (uint32_t)inv_cfg->pwm_gate_pull_up_drv_strnth_LS2);
    CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvo_0_ctrl, PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_LSDR2_NCONFIG, 
        (uint32_t)inv_cfg->pwm_gate_pull_down_drv_strnth_LS2);

    /* Filter configuration for gate drivers */
    CY_USBPD_REG_FIELD_UPDATE(pd->intr17_cfg_7, PDSS_INTR17_CFG_7_BB_GDRVO_VBST_COMP_OUT_FILT_CFG, 3u);
    CY_USBPD_REG_FIELD_UPDATE(pd->intr17_cfg_8, PDSS_INTR17_CFG_8_BB_GDRVI_VBST_COMP_OUT_FILT_CFG, 3u);

    /* Increase leading edge blanking time if drive strengths are weak */
    if((inv_cfg->pwm_gate_pull_down_drv_strnth_LS2 < 3u) ||
            (inv_cfg->pwm_gate_pull_up_drv_strnth_HS2 < 3u))
    {
        if((inv_cfg->pwm_gate_pull_down_drv_strnth_LS2 < 3u) &&
                (inv_cfg->pwm_gate_pull_up_drv_strnth_HS2 < 3u))
        {
            CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvo_2_ctrl, PDSS_BB_GDRVO_2_CTRL_BB_GDRVO_HSRCP_LEB_TIME_CONFIG, (uint32_t)0x7776u);
        }
        else
        {
            CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvo_2_ctrl, PDSS_BB_GDRVO_2_CTRL_BB_GDRVO_HSRCP_LEB_TIME_CONFIG, (uint32_t)0x7576u);
        }
    }

    if((inv_cfg->pwm_gate_pull_down_drv_strnth_LS1 < 3u) ||
            (inv_cfg->pwm_gate_pull_up_drv_strnth_HS1 < 3u))
    {
        if((inv_cfg->pwm_gate_pull_down_drv_strnth_LS1 < 3u) &&
                (inv_cfg->pwm_gate_pull_up_drv_strnth_HS1 < 3u))
        {
            CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvi_2_ctrl, PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_LSZCD_LEB_TIME_CONFIG, 0x7377u);
        }
        else
        {
            CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvi_2_ctrl, PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_LSZCD_LEB_TIME_CONFIG, 0x7177u);
        }
    }

    /* Dead time configuration */
    hs_dead_time = gl_bb_gdrv_dead_time[(BB_GDRV_HS_DEAD_TIME_NS / 5u)];
    ls_dead_time = gl_bb_gdrv_dead_time[(BB_GDRV_LS_DEAD_TIME_NS / 5u)];

    CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvi_4_ctrl, 
        PDSS_BB_GDRVI_4_CTRL_BB_GDRVI_LSDR1_DEAD_TIME_CONFIG, ls_dead_time);
    CY_USBPD_REG_FIELD_UPDATE(pd->bb_gdrvo_3_ctrl,
        PDSS_BB_GDRVO_3_CTRL_BB_GDRVO_HSDR2_DEAD_TIME_CONFIG, hs_dead_time);

    /* iLim filter configuration. */
    regval = pd->intr17_cfg_5;
    regval &= ~(PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_EN | 
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_CFG_MASK |
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_SEL_MASK | 
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_RESET |
            PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_BYPASS);
    regval |= ((uint8_t)CY_USBPD_VBUS_FILTER_CFG_POS_EN_NEG_EN << PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_CFG_POS) |
        (16u << PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_SEL_POS) |
        PDSS_INTR17_CFG_5_BB_40CSA_ILIM_DIG_FILT_EN;
    pd->intr17_cfg_5 = regval;

    /* 
     * Disable the following faults to BB controller
     */
    pd->bbctrl_func_ctrl3 |= (PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VOUT_OV |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VREG_INRUSH |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_PDS_SCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VDDD_BOD |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_OCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_SCP |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VSRC_NEW_N |
        PDSS_BBCTRL_FUNC_CTRL3_BBCTRL_FAULT_DET_DIS_VSRC_NEW_P);

    /* Configure and enable PWM control */
    pd->bb_pwm_0_ctrl |= PDSS_BB_PWM_0_CTRL_BB_PWM_EN_PWMCOMP;
    pd->bb_pwm_1_ctrl |= (PDSS_BB_PWM_1_CTRL_BB_PWM_ISO_N |
        PDSS_BB_PWM_1_CTRL_BB_PWM_EN_MODE_DET |
        PDSS_BB_PWM_1_CTRL_BB_PWM_EN_SKIP_COMP |
        PDSS_BB_PWM_1_CTRL_BB_PWM_ENABLE_PWM);
    pd->bb_pwm_2_ctrl |= PDSS_BB_PWM_2_CTRL_BB_PWM_EN_VIN_RES;
    /* PWM skip configuration */
    pd->bbctrl_func_ctrl2 = ((pd->bbctrl_func_ctrl2 &
        ~(PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_RST_SW_BLNK_TIM_MASK |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_LOW_MASK |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_HIGH_MASK |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SCAP_RST_CNT_WDTH_MASK)) |
        (NS_TO_BBCLK_CYCLE_CNT(167u) << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_RST_SW_BLNK_TIM_POS) |
        ((NS_TO_BBCLK_CYCLE_CNT(375u) - 1u) << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_LOW_POS) |
        ((NS_TO_BBCLK_CYCLE_CNT(542u) - 1u) << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SKIP_WIDTH_HIGH_POS) |
        (NS_TO_BBCLK_CYCLE_CNT(625u) << PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_SCAP_RST_CNT_WDTH_POS) |
        PDSS_BBCTRL_FUNC_CTRL2_BBCTRL_VBST_REFRESH_DISABLE);

    /* Disable buck and boost refresh */
    pd->bbctrl_func_ctrl |= (PDSS_BBCTRL_FUNC_CTRL_BBCTRL_DISABLE_BOOST_REFRESH |
        PDSS_BBCTRL_FUNC_CTRL_BBCTRL_DISABLE_BUCK_REFRESH);

    /* Set skip comparator reference to 400mV */
    CY_USBPD_REG_FIELD_UPDATE(pd->refgen_3_ctrl, PDSS_REFGEN_3_CTRL_SEL9, 27u);

    /* Configure and enable 20CSA (VOUT sense). */
    pd->csa_scp_0_ctrl = ((pd->csa_scp_0_ctrl & 
        ~(PDSS_CSA_SCP_0_CTRL_PD_CSA |
        PDSS_CSA_SCP_0_CTRL_AV1_MASK | 
        PDSS_CSA_SCP_0_CTRL_EN_ITRAN_DET | 
        PDSS_CSA_SCP_0_CTRL_EN_ITRANCOMP_L2H | 
        PDSS_CSA_SCP_0_CTRL_EN_ITRANCOMP_H2L)) |
        (CC_GAIN_60_AV1_VALUE << PDSS_CSA_SCP_0_CTRL_AV1_POS) |
        PDSS_CSA_SCP_0_CTRL_CSA_ISO_N);
    CY_USBPD_REG_FIELD_UPDATE(pd->csa_scp_0_ctrl, 
        PDSS_CSA_SCP_0_CTRL_BW_CC, BBCTRL_20CSA_BW_CC_18_KHZ);

    CY_USBPD_REG_FIELD_UPDATE(pd->csa_scp_0_ctrl, 
        PDSS_CSA_SCP_0_CTRL_BW, BBCTRL_20CSA_BW_OCP_100_KHZ);
    pd->csa_scp_1_ctrl |= PDSS_CSA_SCP_1_CTRL_CSA_EN_IBIAS;

    /* Configure and enable 40CSA (VIN sense). */
    pd->bb_40csa_0_ctrl |= (PDSS_BB_40CSA_0_CTRL_BB_40CSA_EN_SCOMP_DAC_LV |
        PDSS_BB_40CSA_0_CTRL_BB_40CSA_EN_40CSA_LEV_LV);
    pd->bb_40csa_1_ctrl |= (PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_SCOMP_VOUTBYR_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_BIAS_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_EN_40CSA_STG1_LV |
        PDSS_BB_40CSA_1_CTRL_BB_40CSA_ENABLE_40CSA_LV);
    pd->bb_40csa_2_ctrl &= ~PDSS_BB_40CSA_2_CTRL_BB_40CSA_PD_ILIMCMP_LV;
    pd->bb_40csa_3_ctrl |= PDSS_BB_40CSA_3_CTRL_BB_40CSA_ISO_N;

    /* Calculate and update slope compensation. */
    regval = ((BB_SLOPE_COMP_24X * (uint32_t)BB_SLOPE_COMP_24X_L * 
               pwr_cfg->peak_current_sense_resistor) /
              (pwr_cfg->power_inductor_value * (uint32_t)BB_SLOPE_COMP_24X_R));

    CY_USBPD_REG_FIELD_UPDATE(pd->bb_40csa_3_ctrl, PDSS_BB_40CSA_3_CTRL_BB_40CSA_SEL_IDAC_LV, regval);

    /* Power up EA block, configuration is done during VBTR/IBTR */
    pd->bb_ea_0_ctrl = ((pd->bb_ea_0_ctrl &
        ~(PDSS_BB_EA_0_CTRL_BB_EA_PD)) |
        PDSS_BB_EA_0_CTRL_BB_EA_ISO_N);

    /* Enable Gate drivers */
    if (pwr_cfg->buck_boost_operating_mode != BB_FORCED_BUCK)
    {
        pd->bb_gdrvo_0_ctrl = ((pd->bb_gdrvo_0_ctrl &
            ~(PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSDR2_PD |
            PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_HSRCP_PD |
            PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_KEEPOFF_EN)) |
            PDSS_BB_GDRVO_0_CTRL_BB_GDRVO_VBST_COMP_EN);
        pd->bb_gdrvo_1_ctrl = ((pd->bb_gdrvo_1_ctrl &
            ~(PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_GDRV_PD |
            PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_LSDR2_PD)) |
            PDSS_BB_GDRVO_1_CTRL_BB_GDRVO_ISO_N);
    }
    if (pwr_cfg->buck_boost_operating_mode != BB_FORCED_BOOST)
    {
        pd->bb_gdrvi_1_ctrl = ((pd->bb_gdrvi_1_ctrl &
            ~(PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_LSDR1_PD |
            PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_HSDR1_PD |
            PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_LSZCD_PD |
            PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_KEEPOFF_EN)) |
            PDSS_BB_GDRVI_1_CTRL_BB_GDRVI_VBST_COMP_EN);
        pd->bb_gdrvi_2_ctrl = ((pd->bb_gdrvi_2_ctrl &
            ~(PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_GDRV_PD)) |
            PDSS_BB_GDRVI_2_CTRL_BB_GDRVI_ISO_N);
    }
}

uint16_t soln_qi_coil_src_max_supported_volt(
        struct cy_stc_qi_context *qiCtx
        )
{
    /* Handle solution policy */
    uint16_t vinVolt;
    uint16_t supportedVolt;
    
    cy_stc_soln_policy_ctx_t * soln_ctx = get_soln_context();
    const pwr_params_t *pwr_cfg = pd_get_ptr_pwr_tbl(get_usbpd_context(CY_SOLN_BB_INDEX));

    if(CY_QI_PROTO_EPP == qiCtx->qiStat.cfgParams.proto)
    {
        supportedVolt = get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->vbridgeMaxEPPVolt;
    }
    else
    {
        supportedVolt = get_wireless_coil_config((qiCtx->ptrCfg), qiCtx->coilNum)->vbridgeMaxBPPVolt;
    }

    if (pwr_cfg->buck_boost_operating_mode == BB_FORCED_BUCK)
    {
#if CY_SOLN_QI_OVERRIDE_PD
        soln_ctx->vinVolt = soln_vbus_get_value(soln_ctx->pdstack_ctx);
        vinVolt = (soln_ctx->vinVolt - CY_QI_VIN_BUCK_TOLERANCE);
#else /* !CY_SOLN_QI_OVERRIDE_PD */
        vinVolt = (soln_ctx->vinVoltContract - CY_QI_VIN_BUCK_TOLERANCE);
#endif /* CY_SOLN_QI_OVERRIDE_PD */

        supportedVolt = GET_MIN(vinVolt, supportedVolt);        
    }

    /** Apply threshold */
    return (supportedVolt);
}

void soln_hfclk_ext_clk_init(void)
{
    cy_stc_usbpd_context_t *ctx = get_usbpd_context(CY_SOLN_INV_INDEX);
    PPDSS_REGS_T pd = ctx->base;

    /* Enable PCTRL_1 to enable the external clock. */
    pd->bb_ngdo_0_gdrv_en_ctrl = PDSS_BB_NGDO_0_GDRV_EN_CTRL_DEFAULT;
    pd->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
    Cy_SysLib_DelayUs(50);
    pd->ngdo_ctrl |= (PDSS_NGDO_CTRL_NGDO_CP_EN);
    Cy_SysLib_DelayUs(50);
    pd->bb_ngdo_0_gdrv_en_ctrl = (PDSS_BB_NGDO_0_GDRV_EN_CTRL_SEL_ON_OFF |
            PDSS_BB_NGDO_0_GDRV_EN_CTRL_GDRV_EN_ON_VALUE);
    /* The gate driver requires about 5ms to fully turn ON. */
    Cy_SysLib_DelayUs(5000);
}

void soln_hfclk_ext_clk_deinit(void)
{
    cy_stc_usbpd_context_t *ctx = get_usbpd_context(CY_SOLN_INV_INDEX);
    PPDSS_REGS_T pd = ctx->base;

    /* Disable PCTRL_1 to disable external clock. */
    pd->bb_ngdo_0_gdrv_en_ctrl = PDSS_BB_NGDO_0_GDRV_EN_CTRL_DEFAULT;
    Cy_SysLib_DelayUs(50);
    pd->ngdo_ctrl &= ~(PDSS_NGDO_CTRL_NGDO_CP_EN);
    Cy_SysLib_DelayUs(50);
    pd->ngdo_ctrl &= ~(PDSS_NGDO_CTRL_NGDO_ISO_N | PDSS_NGDO_CTRL_NGDO_EN_LV);
}

void soln_hfclk_ext_clk_ctrl(
        bool enable
        )
{
    if (enable)
    {
        /* Switch to external clock. */
        Cy_SysClk_ClkHfSetSource(CY_SYSCLK_CLKHF_IN_EXTCLK);
        /* Disable IMO */
        Cy_SysClk_ImoDisable();
    }
    else
    {
        /* Enable IMO. */
        Cy_SysClk_ImoEnable();
        /* Switch to IMO. */
        Cy_SysClk_ClkHfSetSource(CY_SYSCLK_CLKHF_IN_IMO);
    }
}

void soln_debug_gpio_setvalue(struct cy_stc_qi_context *qiCtx, bool setorclear)
{
    CY_UNUSED_PARAMETER(qiCtx);
    if(setorclear)
    {
#if DEBUG_PIN_ENABLED
        Cy_GPIO_Set(DEBUG_PIN_PORT, DEBUG_PIN_PIN);
#endif /* DEBUG_PIN_ENABLED */
    }
    else
    {
#if DEBUG_PIN_ENABLED
        Cy_GPIO_Clr(DEBUG_PIN_PORT, DEBUG_PIN_PIN);
#endif /* DEBUG_PIN_ENABLED */
    }
}


/* [] END OF FILE */
