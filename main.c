/***************************************************************************//**
* \file main.c
* \version 1.0
* Description: This is the source code for the WiCG1: Wireless Charging EPP 
               Power Transmitter with USBPD input
*              Example for ModusToolbox.
*
* Related Document: See README.md
*
********************************************************************************
* \copyright
* Copyright 2021-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/



#include "cy_pdl.h"
#include "cybsp.h"
#include "config.h"
#include "flash.h"
#include "cy_usbpd_config_table.h"
#include "cy_sw_timer.h"
#include "cy_usbpd_common.h"
#include "cy_pdstack_common.h"
#include "cy_usbpd_typec.h"
#include "cy_pdstack_dpm.h"
#include "cy_usbpd_vbus_ctrl.h"
#include "cy_usbpd_phy.h"
#include "instrumentation.h"
#include "app.h"
#include "pdo.h"
#include "cy_usbpd_idac_ctrl.h"
#include "srom.h"
#include "gpio.h"
#include "system.h"
#include "app_version.h"
#include "ccgx_version.h"
#include "boot.h"
#include "solution.h"
#include "cy_qistack_common.h"

#include "swap.h"
#include "vdm.h"
#include "cy_usbpd_buck_boost.h"

#if CCG_HPI_ENABLE
#include "hpi.h"
#endif /* CCG_HPI_ENABLE */
#include "cy_qistack_pm.h"
#include "cy_qistack_console.h"
#include "cy_qistack_utils.h"

extern void * get_config(void);
/* Select target silicon ID for CYPD7291-68LDXS. */
#define CCG_DEV_SILICON_ID                      (0x3180u)
#define CCG_DEV_FAMILY_ID                       (0x11C1u)

#if (defined(__GNUC__) && !defined(__ARMCC_VERSION))
extern uint8_t __cy_fw_verify_start;
extern uint8_t __cy_fw_verify_length;
extern uint8_t __cy_config_fw_start;
extern uint8_t __cy_config_fw_length;

#define CY_DFU_APP1_VERIFY_START       ( (uint32_t)&__cy_fw_verify_start )
#define CY_DFU_APP1_VERIFY_LENGTH      ( (uint32_t)&__cy_fw_verify_length )
#define CY_APP_BOOT_ID                 (0xFFFF)
#define CY_DFU_APP1_CONFIG_FW_START    ( (uint32_t)&__cy_config_fw_start )
#define CY_DFU_APP1_CONFIG_FW_LENGTH   ( (uint32_t)&__cy_config_fw_length )
#define CY_DFU_METADATA_VERSION        ((uint32_t)0x01)
#define CY_DFU_METADATA_VALID          (0x4946)

CY_SECTION(".cy_app_signature") __USED static const uint32_t cy_dfu_appSignature;
CY_SECTION(".flash_padding") __USED static const uint32_t gl_flash_padding;
CY_SECTION(".meta_padding") __USED static const uint8_t gl_meta_padding[0x80];
CY_SECTION(".cy_metadata") __USED

static const uint32_t cy_dfu_metadata[] =
{
    CY_DFU_APP1_VERIFY_START, CY_DFU_APP1_VERIFY_LENGTH, /* The App1 base address and length */
    CY_APP_BOOT_ID,
    CY_DFU_APP1_CONFIG_FW_START, CY_DFU_APP1_CONFIG_FW_LENGTH,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    (CY_DFU_METADATA_VERSION | (CY_DFU_METADATA_VALID << 16))
    /* The rest does not matter */
};
#endif/* (defined(__GNUC__) && !defined(__ARMCC_VERSION)) */
#if defined(__GNUC__) || defined(__ARMCC_VERSION)
__attribute__ ((__section__(".cymeta"), used))
#elif defined(__ICCARM__)
#pragma  location=".cymeta"
#else
#error "Unsupported toolchain"
#endif
const uint8 cy_metadata[] = {
    0x00u, 0x02u, 0x31u, 0x80u, 0x11u, 0xC1u, 0x00u, 0x00u,
    0x31u, 0x80u, 0x11u, 0xC1u};

/*
 * Reserve 32 bytes of space for Customer related info.
 * Fill this with customer related info.
 * This will be placed at an offset of 0xC0 from the start of FW Image.
 */
__attribute__ ((section(".customer_region"), used))
static const uint32_t gl_customer_info[8] = {0u};

/* Place the bootloader version at a fixed location, so that firmware can retrieve this as well. */
__attribute__ ((section(".base_version"), used))
static const uint32_t gl_base_version = FW_BASE_VERSION;
__attribute__ ((section(".app_version"), used))
static const uint32_t gl_app_version  = APP_VERSION;
__attribute__ ((section(".dev_siliconid"), used))
static const uint32_t gl_ccg_silicon_id = MAKE_DWORD_FROM_WORD (CCG_DEV_SILICON_ID, CCG_DEV_FAMILY_ID);
__attribute__ ((section(".fw_reserved"), used))
static const uint32_t gl_reserved_buf[5] = {0u};

cy_stc_sw_timer_t timer_ctx;
cy_stc_usbpd_context_t usbpd_port0_ctx;
cy_stc_usbpd_context_t usbpd_port1_ctx;
cy_stc_pdstack_context_t pdstack_port0_ctx;
cy_stc_qi_context_t qistack_0_ctx;
cy_stc_scb_uart_context_t UART_context;
cy_stc_soln_policy_ctx_t soln_policy_ctx;

/*
 * Application callback functions for the Qi policy.
 */
cy_stc_qi_app_cbk_t qi_app_cbk =
{
    .app_event_handler=soln_qi_event_handler,
    .hardware_init=soln_qi_hardware_init,
    .ask_bmc_init=soln_qi_ask_bmc_init,
    .ask_cc_up_cmp_enable=soln_qi_ask_cc_up_cmp_enable,
    .cc_up_cmp_enable=soln_qi_cc_up_cmp_enable,
    .cc_up_cmp_disable=soln_qi_cc_up_cmp_disable,
    .cc_dn_cmp_enable=soln_qi_cc_dn_cmp_enable,
    .cc_dn_cmp_disable=soln_qi_cc_dn_cmp_disable,
    .pds_scp_cmp_enable=soln_qi_pds_scp_cmp_enable,
    .pds_scp_cmp_disable=soln_qi_pds_scp_cmp_disable,
    .set_ask_path=soln_qi_set_ask_path,
    .fsk_oper_init=soln_qi_fsk_oper_init,
    .inv_fb_enable=soln_qi_inv_fb_enable,
    .inv_fb_disable=soln_qi_inv_fb_disable,
    .inv_send_analog_ping=soln_qi_inv_send_analog_ping,
    .inv_start_digital_ping=soln_qi_inv_start_digital_ping,
    .inv_stop_digital_ping=soln_qi_inv_stop_digital_ping,
    .coil_src_enable=soln_qi_coil_src_enable,
    .coil_src_disable=soln_qi_coil_src_disable,
    .coil_src_enable_status=soln_qi_coil_src_enable_status,
    .coil_src_set_voltage=soln_qi_coil_src_set_voltage,
    .coil_src_ready_status=soln_qi_coil_src_ready_status,
    .coil_src_get_voltage=soln_qi_coil_src_get_voltage,
    .coil_src_get_current=soln_qi_coil_src_get_current,
	.coil_src_get_max_supported_volt=soln_qi_coil_src_max_supported_volt,
    .led_set_pin_value=soln_qi_led_set_pin_value,
    .Console_WriteString=soln_qi_uart_write_string,
	.auth_init_pwm=soln_auth_init_pwm,
    .debug_gpio_SetValue= soln_debug_gpio_setvalue,
};

#if CCG_HPI_ENABLE

uint32_t solution_get_version_details(cy_stc_pdstack_context_t *ptrPdStackContext)
{
    uint32_t version;

    CY_UNUSED_PARAMETER(ptrPdStackContext);
    (void)CALL_MAP(memcpy)((&version), sys_get_img1_fw_version(), 4); /* PRQA S 0315 */

    return version;
}
void update_hpi_regs ()
{
#if CCG_FIRMWARE_APP_ONLY
    uint8_t mode, reason;
    uint8_t ver_invalid[8] = {0u};

    /* Flash access is not allowed. */
    flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM, CCG_LAST_FLASH_ROW_NUM,
            CCG_BOOT_LOADER_LAST_ROW);

    /* Update HPI registers with default values. */
    mode   = 0x95u;              /* Dual boot, 256 byte flash, 2 ports, FW1 running. */
    reason = 0x08u;              /* FW2 is not valid. */
    CALL_MAP(hpi_set_mode_regs)(mode, reason);
    CALL_MAP(hpi_update_versions)(ver_invalid, (uint8_t *)&gl_base_version, ver_invalid);
    CALL_MAP(hpi_update_fw_locations)(0u, CCG_LAST_FLASH_ROW_NUM);
#else /* !CCG_FIRMWARE_APP_ONLY */
    uint8_t mode = 0x81u, reason = 0x00u;
    uint32_t fw1_ver, fw2_ver;
    uint16_t fw1_loc, fw2_loc;
    sys_fw_metadata_t *fw1_md, *fw2_md;
    uint8_t ver_invalid[8] = {0u};

    /* Set mode variables and flash access limits based on the active firmware. */
    if (sys_get_device_mode() == SYS_FW_MODE_FWIMAGE_1)
    {
        mode = 0x81u | ((NO_OF_TYPEC_PORTS - 1u) << 2u) | ((CCG_FLASH_ROW_SHIFT_NUM - 7u) << 4u);

        /* Check if FW2 is valid. */
        if (boot_validate_fw ((sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR) != CY_PDSTACK_STAT_SUCCESS)
        {
            reason = 0x08u;
        }
        /* Set the legal flash access range.
           Note: This needs to be updated whenever the FW1/FW2 reserved memory size is updated.
         */
        flash_set_access_limits (CCG_LAST_FLASH_ROW_NUM + 1u, CCG_LAST_FLASH_ROW_NUM + 1u,
                CCG_IMG2_METADATA_ROW_NUM, CCG_BOOT_LOADER_LAST_ROW);

    }

    CALL_MAP(hpi_set_mode_regs)(mode, reason);

    /* Calculate the version address from the firmware metadata. */
    fw1_md  = (sys_fw_metadata_t *)CCG_IMG1_FW_METADATA_ADDR;
    fw1_ver = fw1_md->fw_start + SYS_FW_VERSION_OFFSET;
    fw1_loc = (fw1_md->fw_start >> CCG_FLASH_ROW_SHIFT_NUM);

    if (reason == 0u)
    {
        fw2_md  = (sys_fw_metadata_t *)CCG_IMG2_FW_METADATA_ADDR;
        fw2_ver = fw2_md->fw_start + SYS_FW_VERSION_OFFSET;
        fw2_loc = (fw2_md->fw_start >> CCG_FLASH_ROW_SHIFT_NUM);
    }
    else
    {
        fw2_ver = (uint32_t)ver_invalid;
        fw2_loc = CCG_LAST_FLASH_ROW_NUM + 1u;
    }

    /* Update version information in the HPI registers. */
    CALL_MAP(hpi_update_versions)(
            (uint8_t *)SYS_BOOT_VERSION_ADDRESS,
            (uint8_t *)fw1_ver,
            (uint8_t *)fw2_ver
            );
    /* Update firmware location registers. */
    CALL_MAP(hpi_update_fw_locations)(fw1_loc, fw2_loc);

#endif /* CCG_FIRMWARE_APP_ONLY */
}

#endif

/** \cond INTERNAL */
CY_SECTION(".bootloaderruntype") __USED volatile uint32_t cyBtldrRunType;
#if (!CCG_LOAD_SHARING_ENABLE)
cy_en_pdstack_status_t i2cm_reg_write(uint8_t scb_index,
        uint8_t slave_addr,
        uint8_t *buffer,
        uint16_t size,
        uint8_t *reg_addr,
        uint8_t reg_size)
{
    (void)scb_index;
    (void)slave_addr;
    (void)buffer;
    (void)size;
    (void)reg_addr;
    (void)reg_size;
    return CY_PDSTACK_STAT_SUCCESS;
}

cy_en_pdstack_status_t i2cm_reg_read(uint8_t scb_index,
        uint8_t slave_addr,
        uint8_t *buffer,
        uint16_t size,
        uint8_t *reg_addr,
        uint8_t reg_size)
{
        (void)scb_index;
        (void)slave_addr;
        (void)buffer;
        (void)size;
        (void)reg_addr;
        (void)reg_size;
        return CY_PDSTACK_STAT_SUCCESS;

}
#endif /* !CCG_LOAD_SHARING_ENABLE */

const cy_stc_pdstack_dpm_params_t pdstack_port0_dpm_params =
{
        .dpmSnkWaitCapPeriod = 300, /** 300mS is absolute minimum as per spec */
        .dpmRpAudioAcc = CY_PD_RP_TERM_RP_CUR_DEF,
        .dpmDefCableCap = 300,
        .muxEnableDelayPeriod = 0,
        .typeCSnkWaitCapPeriod = 100,
        .defCur = 90
};

cy_stc_pdstack_context_t * pd_contexts[NO_OF_TYPEC_PORTS] =
{
	&pdstack_port0_ctx
};

cy_stc_usbpd_context_t * usbpd_contexts[NO_OF_TYPEC_PORTS] =
{
	&usbpd_port0_ctx,
	&usbpd_port1_ctx
};

bool mux_ctrl_init(uint8_t port)
{
    /* No MUXes to be controlled on the PMG1 proto kits. */
    CY_UNUSED_PARAMETER(port);
    return true;
}

const cy_stc_sysint_t wdt_interrupt_config =
{
    .intrSrc = (IRQn_Type)srss_interrupt_wdt_IRQn,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port0_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port0_DS_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr0_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_IRQ,
    .intrPriority = 0U,
};

const cy_stc_sysint_t usbpd_port1_intr1_config =
{
    .intrSrc = (IRQn_Type)mtb_usbpd_port1_DS_IRQ,
    .intrPriority = 0U,
};

cy_stc_pdstack_context_t * get_pdstack_context(uint8_t portIdx)
{
    return (pd_contexts[portIdx]);
}

cy_stc_usbpd_context_t * get_usbpd_context(uint8_t portIdx)
{
    return (usbpd_contexts[portIdx]);
}

cy_stc_qi_context_t * get_qistack_context(void)
{
    return &qistack_0_ctx;
}

cy_stc_soln_policy_ctx_t * get_soln_context(void)
{
    return &soln_policy_ctx;
}

cy_stc_sw_timer_t * get_timer_context(void)
{
    return &timer_ctx;
}

void instrumentation_cb(uint8_t port, inst_evt_t evt)
{
    /*
     * Since these are serious fails and any delay can cause additional issues,
     * we need to update data and proceed with reset.
     */
    if (INST_EVT_WDT_RESET == evt)
    {
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: WDT_RESET ----------------\r\n");
    }
    else
    {
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_CRITICAL, "\r\nFAULT: CPU_HARD_FAULT ----------------\r\n");
    }
}

static void wdt_interrupt_handler(void)
{
    /* Clear WDT pending interrupt */
    Cy_WDT_ClearInterrupt();

#if (TIMER_TICKLESS_ENABLE == 0)
    /* Load the timer match register. */
    Cy_WDT_SetMatch((Cy_WDT_GetCount() + timer_ctx.gl_multiplier))
#endif /* (TIMER_TICKLESS_ENABLE == 0) */

    /* Invoke the timer handler. */
    cy_sw_timer_interrupt_handler (&timer_ctx);
}

static void cy_usbpd0_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&usbpd_port0_ctx);
}

static void cy_usbpd0_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&usbpd_port0_ctx);
}

static void cy_usbpd1_intr0_handler(void)
{
    Cy_USBPD_Intr0Handler(&usbpd_port1_ctx);
}

static void cy_usbpd1_intr1_handler(void)
{
    Cy_USBPD_Intr1Handler(&usbpd_port1_ctx);
}

cy_stc_pd_dpm_config_t* Dpm_GetStatus(void)
{
    return &(pdstack_port0_ctx.dpmConfig);
}

/*
 * Application callback functions for the DPM. Since this application
 * uses the functions provided by the stack, loading the stack defaults.
 */
const cy_stc_pdstack_app_cbk_t app_callback =
{
    app_event_handler,
    soln_vconn_enable,
    soln_vconn_disable,
    soln_vconn_is_present,
    soln_vbus_is_present,
    vbus_discharge_on,
    vbus_discharge_off,
    soln_psnk_set_voltage,
    soln_psnk_set_current,
    soln_psnk_enable,
    soln_psnk_disable,
    soln_eval_src_cap,
    eval_dr_swap,
    eval_pr_swap,
    eval_vconn_swap,
    eval_vdm,
    soln_vbus_get_value
};

cy_stc_pdstack_app_cbk_t* app_get_callback_ptr(cy_stc_pdstack_context_t * context)
{
    (void)context;
    /* Solution callback pointer is same for all ports */
    return ((cy_stc_pdstack_app_cbk_t *)(&app_callback));
}

#if (FLASHING_MODE_PD_ENABLE == 1u)
/* Global to store Device Reset signature. */
static uint32_t gl_reset_sig = 0u;
static bool gl_port_disabled[NO_OF_TYPEC_PORTS] = {0u};
static void pa_port_disable_cb(cy_stc_pdstack_context_t *ptrPdStackContext, cy_en_pdstack_dpm_typec_cmd_resp_t resp)
{
    (void)ptrPdStackContext;
    (void)resp;
    uint8_t port = ptrPdStackContext->port;

    gl_port_disabled[port] = true;
    if(gl_port_disabled[0] == true
#if (NO_OF_TYPEC_PORTS > 1u)
     && gl_port_disabled[1] == true
#endif /* (NO_OF_TYPEC_PORTS > 1u) */
      )
#if !CCG_FIRMWARE_APP_ONLY
        /* Disable all interrupts. */
         __disable_irq();
        /*
         * NOTE: Using Bootloader component provided variable
         * to store the signature. This makes sure that this value
         * is never overwritten by compiler and it ratains the value
         * through resets and jumps. We use lower two bytes of this
         * variable to store the siganture. */
        /* NOTE: Signature will be zero for RESET command. */
        cyBtldrRunType |= gl_reset_sig;
#endif /* CCG_FIRMWARE_APP_ONLY */

        /* Call device reset routine. */
        NVIC_SystemReset ();
}

static void soln_handle_FWValidation_failure()
{
    soln_pd_event_handler(0u, APP_EVT_CONFIG_ERROR, NULL);

#if CCG_FIRMWARE_APP_ONLY
    /* Can't do anything if config table is not valid. */
    while (1);
#else /* !CCG_FIRMWARE_APP_ONLY */
    /* Move to Bootloader by updating the signature and Reset. */

    /*
    * NOTE: Using Bootloader component provided variable
    * to store the signature. This makes sure that this value
    * is never overwritten by compiler and it ratains the value
    * through resets and jumps. We use lower two bytes of this
    * variable to store the siganture. */
    /* NOTE: Signature will be zero for RESET command. */
    cyBtldrRunType |= CCG_BOOT_MODE_RQT_SIG;

    /* Now reset the device. */
    __NVIC_SystemReset ();
#endif /* CCG_FIRMWARE_APP_ONLY */
}

cy_en_pdstack_status_t uvdm_handle_device_reset(uint32_t reset_sig)
{
	if(0 == reset_sig)
	{
		/* Notify PD stack to disable Port. */
		if (Cy_PdStack_Dpm_SendTypecCommand (get_pdstack_context(0u), CY_PDSTACK_DPM_CMD_PORT_DISABLE, pa_port_disable_cb) ==
		CY_PDSTACK_STAT_SUCCESS)
		{
		#if (NO_OF_TYPEC_PORTS > 1u)
			if (Cy_PdStack_Dpm_SendTypecCommand (get_pdstack_context(1u), CY_PDSTACK_DPM_CMD_PORT_DISABLE, pa_port_disable_cb) !=
			CY_PDSTACK_STAT_SUCCESS)
			{
			/*
			* We have already triggered disable on one port. Now, we must
			* go ahead with the reset so as not to leave the device in bad
			* state. So mark the callback as completed for port 1.
			*/
			gl_port_disabled[1] = true;
			}
		#endif /* (NO_OF_TYPEC_PORTS > 1u) */

		/* Store Reset Signature. */
		gl_reset_sig = reset_sig;
		return CY_PDSTACK_STAT_NO_RESPONSE;
		}
		return CY_PDSTACK_STAT_FAILURE;
	}
	else
	{
		/* Store Reset Signature on Jump to Boot; Since WICG1 is Bus powered Just go through Reset without disabling Port*/
		gl_reset_sig = reset_sig;
		pa_port_disable_cb(0,0);
		return CY_PDSTACK_STAT_NO_RESPONSE;
	}
}
#else
cy_en_pdstack_status_t uvdm_handle_device_reset(uint32_t reset_sig)
{
    (void)reset_sig;
    return CY_PDSTACK_STAT_NO_RESPONSE;
}
#endif /*(FLASHING_MODE_PD_ENABLE == 1u) */ 

/*
 * Solution level callback functions for the application layer
 */
const app_sln_handler_t sln_cbk =
{
    soln_pd_event_handler,
    app_get_callback_ptr,
    get_pdstack_context,
    mux_ctrl_init,
    uvdm_handle_device_reset
};

const cy_stc_fault_vbus_ocp_cfg_t ocp_table =
{
    .enable = 1,
    .mode = 3,
    .threshold = 0,
    .debounce = 10,
    .retryCount = 255
};

const cy_stc_fault_vbus_scp_cfg_t scp_table =
{
    .enable = 1,
    .mode = 3,
    .debounce = 10,
    .retryCount = 255,
    .threshold = 0
};

const cy_stc_fault_vbus_uvp_cfg_t uvp_table =
{
    .enable = 1,
    .mode = 2,
    .debounce = 10,
    .retryCount = 255,
    .threshold = 70
};

const cy_stc_fault_vbus_ovp_cfg_t ovp_table =
{
    .enable = 1,
    .mode = 2,
    .debounce = 10,
    .retryCount = 255,
    .threshold = 20
};

const cy_stc_legacy_charging_cfg_t legacyChargingTable =
{
    .srcSel = 0,
    .snkSel = 0x0D,
    .enable = 0,
    .appleSrcId = 0,
    .qcSrcType = 0,
    .afcSrcCapCnt = 0
};

int main(void)
{
    cy_rslt_t result;

#if CCG_HPI_ENABLE
    uint8_t hpi_scb_idx;
#endif /* CCG_HPI_ENABLE */
    /* Enable this to delay the firmware execution under SWD connect. */
#ifdef BREAK_AT_MAIN
    uint8_t volatile x= 0;
    while(x==0);
#endif /* BREAK_AT_MAIN */

#if CCG_SROM_CODE_ENABLE
    /* Initialize SROM variables */
#endif /* CCG_SROM_CODE_ENABLE */

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    sys_set_device_mode ((sys_fw_mode_t)(0x01u));

    /* TODO: No fw version check or config table check enabled */

    Cy_SysInt_Init(&wdt_interrupt_config, &wdt_interrupt_handler);

    NVIC_EnableIRQ(wdt_interrupt_config.intrSrc);

    /*
     * Software timer initialize.
     */
    cy_sw_timer_init(&timer_ctx, Cy_SysClk_ClkSysGetFrequency());

    /* Enable global interrupts */
    __enable_irq();

	/* Initialize solution policy at the beginning */
	soln_policy_init(&soln_policy_ctx,
			&qistack_0_ctx,
			&pdstack_port0_ctx);

    /* Initialize the instrumentation related data structures. */
    instrumentation_init();

    /* Register callback function to be executed when instrumentation fault occurs. */
    instrumentation_register_cb((instrumentation_cb_t)instrumentation_cb);

    /* Register callbacks for solution level functions */
    register_soln_function_handler(&pdstack_port0_ctx, (app_sln_handler_t *)&sln_cbk);


    /* Initialize the USBPD interrupts */
    Cy_SysInt_Init(&usbpd_port0_intr0_config, &cy_usbpd0_intr0_handler);
    Cy_SysInt_Init(&usbpd_port0_intr1_config, &cy_usbpd0_intr1_handler);
    Cy_SysInt_Init(&usbpd_port1_intr0_config, &cy_usbpd1_intr0_handler);
    Cy_SysInt_Init(&usbpd_port1_intr1_config, &cy_usbpd1_intr1_handler);

    NVIC_EnableIRQ(usbpd_port0_intr0_config.intrSrc);
    NVIC_EnableIRQ(usbpd_port0_intr1_config.intrSrc);
    NVIC_EnableIRQ(usbpd_port1_intr0_config.intrSrc);
    NVIC_EnableIRQ(usbpd_port1_intr1_config.intrSrc);

    /* TODO: Code for bringup, to be removed later. */
    Cy_GPIO_Write(LED_BLUE_PORT, LED_BLUE_PIN, 1);
    Cy_GPIO_Write(LED_RED_PORT, LED_RED_PIN, 1);
    Cy_SCB_UART_Init(UART_HW, &UART_config, &UART_context);
    Cy_SCB_UART_Enable(UART_HW);

    /* Update the Main wireless table Pointer for HW context */
    usbpd_port0_ctx.cfg_table = get_config();
	usbpd_port1_ctx.cfg_table = get_config();

    /** Do Config Table CRC validation before usage */
    if(boot_validate_configtable ((uint8_t *)usbpd_port0_ctx.cfg_table) != CY_PDSTACK_STAT_SUCCESS)
    {
        soln_handle_FWValidation_failure();
    }

    /* Initialize the USBPD context parameters */
    usbpd_port0_ctx.vbusCsaRsense = pd_get_ptr_pwr_tbl(&usbpd_port0_ctx)->current_sense_res;
    usbpd_port0_ctx.peak_current_sense_resistor = pd_get_ptr_pwr_tbl(&usbpd_port0_ctx)->peak_current_sense_resistor;
    usbpd_port1_ctx.vbusCsaRsense = pd_get_ptr_pwr_tbl(&usbpd_port0_ctx)->current_sense_res;
    usbpd_port1_ctx.peak_current_sense_resistor = pd_get_ptr_pwr_tbl(&usbpd_port0_ctx)->peak_current_sense_resistor;

    /* Initialize the USBPD driver */
    Cy_USBPD_Init(&usbpd_port0_ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM, (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, Dpm_GetStatus);
    Cy_USBPD_Init(&usbpd_port1_ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM, (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, Dpm_GetStatus);

    const cy_stc_pdstack_port_cfg_t *pdstack_port0_config = pd_get_ptr_pdstack_tbl(&usbpd_port0_ctx);

    /* Initialize the USBPD Device Policy Manager. */
    Cy_PdStack_Dpm_Init(&pdstack_port0_ctx,
            &usbpd_port0_ctx,
            pdstack_port0_config,
            app_get_callback_ptr(&pdstack_port0_ctx),
            &pdstack_port0_dpm_params,
            &timer_ctx);

    /** Do Config table offset validation after DPM is Initialised */
    bool app_validation;
    app_validation = app_validate_configtable_offsets(&pdstack_port0_ctx);
    if (!app_validation)
    {
        soln_handle_FWValidation_failure();
    }

    /* Register USBPD drivers with Qi policy before Qi policy Init. */
    Cy_QiStack_Register_Usbpd(&qistack_0_ctx,
            &usbpd_port0_ctx,
            &usbpd_port1_ctx);


    /* Initialize the Qi policy manager */
    Cy_QiStack_Init(&qistack_0_ctx,
            get_config(),
            &qi_app_cbk,
            &timer_ctx);

    Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nReset...\r\n");
    Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFW Base Version: %d.%d.%d build %d.\r\n",
            FW_MAJOR_VERSION, FW_MINOR_VERSION, FW_PATCH_VERSION, FW_BUILD_NUMBER);
    Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nFW App Version: %d.%d.%d.\r\n",
            APP_MAJOR_VERSION, APP_MINOR_VERSION, APP_EXT_CIR_NUM);

    /* Check if the silicon has the necessary TRIMs. If not sending out a warning message. */
    if (0u == CBL_GAIN150_TRIM_P250A_ROOM(CY_SOLN_BB_INDEX))
    {
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE,
                "\r\nWARNING: The silicon does not have necessary TRIM. Using approximate values.\r\n");
    }

    pdstack_port0_ctx.ptrUsbPdContext->usbpdConfig->vbusOcpConfig = &ocp_table;
    pdstack_port0_ctx.ptrUsbPdContext->usbpdConfig->vbusScpConfig = &scp_table;
    pdstack_port0_ctx.ptrUsbPdContext->usbpdConfig->vbusOvpConfig = &ovp_table;
    pdstack_port0_ctx.ptrUsbPdContext->usbpdConfig->vbusUvpConfig = &uvp_table;

    /* Legacy charging table */
    pdstack_port0_ctx.ptrUsbPdContext->usbpdConfig->legacyChargingConfig = &legacyChargingTable;

    /* Initialize the fault configuration values */
    fault_handler_init_vars(&pdstack_port0_ctx);


    /* Initialize application layer. */
    app_init(&pdstack_port0_ctx);
    timer_init(&pdstack_port0_ctx);

    /* Initialize solution level faults */
    cy_soln_fault_init();

    soln_pd_event_handler(&pdstack_port0_ctx, APP_EVT_POWER_CYCLE, NULL);

    /* Start any timers or tasks associated with instrumentation or house keeping. */
    instrumentation_start();

    /** Switch to external clock if the configuration is set. */
    if (0u != get_wireless_main_config(get_config())->externalClockEnable)
    {
        soln_hfclk_ext_clk_init();
        soln_hfclk_ext_clk_ctrl(true);
        Cy_Console_Printf(get_qistack_context(), CY_QI_UART_VERBO_LVL_MESSAGE, "\r\nExternal clock enabled\r\n");
    }

    /* Start Type-C PD Device Policy Manager */
    Cy_PdStack_Dpm_Start(&pdstack_port0_ctx);

#if CCG_HPI_ENABLE
	CALL_MAP(hpi_set_fixed_slave_address)(HPI_I2C_DEFAULT_ADDR);
	hpi_scb_idx = HPI_SCB_INDEX;

	/* Initialize the HPI interface. */
	CALL_MAP(hpi_init)(pd_contexts[0], hpi_scb_idx);

	CALL_MAP(hpi_set_flash_params)(CY_FLASH_SIZE, CY_FLASH_SIZEOF_ROW, CCG_LAST_FLASH_ROW_NUM + 1u, CCG_BOOT_LOADER_LAST_ROW);

	update_hpi_regs ();

	/* Send a reset complete event to the EC. */
	CALL_MAP(hpi_send_fw_ready_event) ();
#endif /* CCG_HPI_ENABLE */
    for (;;)
    {
    	/* Handle the solution policy. */
    	soln_policy_task(&soln_policy_ctx);
        /* Handle solution faults */
        cy_soln_fault_task();

    	/* Perform the PD port policy tasks. */
        Cy_PdStack_Dpm_Task(&pdstack_port0_ctx);
        app_task(&pdstack_port0_ctx);

        /* Perform the Qi policy tasks. */
        Cy_QiStack_Task(&qistack_0_ctx);

        /* Task for printing UART queue data into Console */
        cy_soln_uart_task(&soln_policy_ctx);

        /* Perform tasks associated with instrumentation. */
        instrumentation_task();
#if CCG_HPI_ENABLE
            /* Handle any pending HPI commands. */
            CALL_MAP(hpi_task) ();
#endif /* CCG_HPI_ENABLE */

#if SYS_DEEPSLEEP_ENABLE
        /* Perform power saving task */
        soln_policy_sleep(&soln_policy_ctx);
#endif /* SYS_DEEPSLEEP_ENABLE */
    }
}

/* [] END OF FILE */
