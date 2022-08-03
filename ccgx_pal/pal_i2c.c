/**
* \copyright
* MIT License
*
* Copyright (c) 2019 Infineon Technologies AG
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE
*
* \endcopyright
*
* \author Infineon Technologies AG
*
* \file pal_i2c.c
*
* \brief   This file implements the platform abstraction layer(pal) APIs for I2C.
*
* \ingroup  grPAL
*
* @{
*/

#include "optiga/pal/pal_i2c.h"
#include "cy_scb_i2c.h"
#include "cycfg.h"
#include "cy_qistack_auth_optiga.h"

#define PAL_I2C_MASTER_MAX_BITRATE  (400U)

static volatile uint32_t g_entry_count = 0;
static pal_i2c_t * gp_pal_i2c_current_ctx;
static volatile uint8_t i2c_task_status = 0;
static cy_stc_scb_i2c_context_t i2cContext;

#define WAIT_FOR_I2C_COMPLETION(i2cstatus)\
    while (i2c_task_status != 1)\
    {\
        Cy_SysLib_Delay(1);\
        i2cstatus();\
    }


static pal_status_t pal_i2c_acquire(const void * p_i2c_context)
{
    // To avoid compiler errors/warnings. This context might be used by a target 
    // system to implement a proper mutex handling
    (void)p_i2c_context;
    
    if (0 == g_entry_count)
    {
        g_entry_count++;
        if (1 == g_entry_count)
        {
            return PAL_STATUS_SUCCESS;
        }
    }
    return PAL_STATUS_FAILURE;
}

static void pal_i2c_release(const void * p_i2c_context)
{
    // To avoid compiler errors/warnings. This context might be used by a target 
    // system to implement a proper mutex handling
    (void)p_i2c_context;
    
    g_entry_count = 0;
}

void invoke_upper_layer_callback (const pal_i2c_t * p_pal_i2c_ctx, optiga_lib_status_t event)
{
    upper_layer_callback_t upper_layer_handler;

    upper_layer_handler = (upper_layer_callback_t)p_pal_i2c_ctx->upper_layer_event_handler;

    upper_layer_handler(p_pal_i2c_ctx->p_upper_layer_ctx, event);

    //Release I2C Bus
    pal_i2c_release(p_pal_i2c_ctx->p_upper_layer_ctx);
}



// !!!OPTIGA_LIB_PORTING_REQUIRED
// The next 5 functions are required only in case you have interrupt based i2c implementation
void i2c_master_end_of_transmit_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
}

void i2c_master_end_of_receive_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_SUCCESS);
}

void i2c_master_error_detected_callback(void)
{
    invoke_upper_layer_callback(gp_pal_i2c_current_ctx, PAL_I2C_EVENT_ERROR);
}

void i2c_master_nack_received_callback(void)
{
    i2c_master_error_detected_callback();
}

void i2c_master_arbitration_lost_callback(void)
{
    i2c_master_error_detected_callback();
}


pal_status_t pal_i2c_init(const pal_i2c_t * p_i2c_context)
{
    (void)p_i2c_context;
    (void)p_i2c_context;
	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Init\r\n");

	Cy_SCB_I2C_Init(I2C_AUTH_HW, &I2C_AUTH_config, &i2cContext);
	Cy_SCB_I2C_Enable(I2C_AUTH_HW, &i2cContext);

    return PAL_STATUS_SUCCESS;
}

pal_status_t pal_i2c_deinit(const pal_i2c_t * p_i2c_context)
{
    (void)p_i2c_context;
	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_DeInit\r\n");
	Cy_SCB_I2C_Disable(I2C_AUTH_HW, &i2cContext);
	Cy_SCB_I2C_DeInit(I2C_AUTH_HW);
    return PAL_STATUS_SUCCESS;
}



pal_status_t pal_i2c_write(pal_i2c_t * p_i2c_context, uint8_t * p_data, uint16_t length)
{
	CySCB_Type *base = I2C_AUTH_HW;
	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Write\r\n");
    pal_status_t status = pal_i2c_acquire(p_i2c_context);

    //Acquire the I2C bus before read/write
    if (PAL_STATUS_SUCCESS == status)
    {
        gp_pal_i2c_current_ctx = p_i2c_context;

        //Invoke the low level i2c master driver API to write to the bus
        //uint32_t i2cstat = WiCCG_I2C_AUTH_I2CMasterSendStart(p_i2c_context->slave_address, 0, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT);
		cy_en_scb_i2c_status_t i2cstat = Cy_SCB_I2C_MasterSendStart(base, p_i2c_context->slave_address, CY_SCB_I2C_WRITE_XFER,
			QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
        if (i2cstat == CY_SCB_I2C_SUCCESS)
        {
            uint16_t iterate = 0;
            
            for (iterate = 0; iterate < length; iterate++ )
            {
                //uint32_t ret_sts = 0;
                //ret_sts = WiCCG_I2C_AUTH_I2CMasterWriteByte(p_data[iterate], QI_TIMER_AUTH_I2C_WRITE_TIMEOUT);   
				i2cstat = Cy_SCB_I2C_MasterWriteByte(base, p_data[iterate], QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
                if (i2cstat != CY_SCB_I2C_SUCCESS) break;
            }
               
                
			Cy_SCB_I2C_MasterSendStop(base, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
            
            i2c_master_end_of_transmit_callback();
			//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Write Success\r\n");
        }
        else
        {
            if (i2cstat == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK )
            {
                // Address NAK must follow with a STOP
				Cy_SCB_I2C_MasterSendStop(base, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
            }
            
            i2c_master_error_detected_callback();
            status = PAL_STATUS_FAILURE;
			//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Write Failure\r\n");
            
        }        
    }
       
    
    return status;
}

pal_status_t pal_i2c_read(pal_i2c_t * p_i2c_context, uint8_t * p_data, uint16_t length)
{
	CySCB_Type *base = I2C_AUTH_HW;

	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Read\r\n");

    pal_status_t status = pal_i2c_acquire(p_i2c_context);

    //Acquire the I2C bus before read/write
    if (PAL_STATUS_SUCCESS == status)
    {
        gp_pal_i2c_current_ctx = p_i2c_context;
        
        //uint32_t i2cstat = WiCCG_I2C_AUTH_I2CMasterSendStart(p_i2c_context->slave_address, 1, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT);
		cy_en_scb_i2c_status_t i2cstat = Cy_SCB_I2C_MasterSendStart(base, p_i2c_context->slave_address, CY_SCB_I2C_READ_XFER,
			QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
        if (i2cstat == CY_SCB_I2C_SUCCESS)
        {
            uint16 iterate = 0;
            for (iterate = 0; iterate < length; iterate++ )
            {
				i2cstat = Cy_SCB_I2C_MasterReadByte(base, ((iterate != length - 1) ? CY_SCB_I2C_ACK : CY_SCB_I2C_NAK), p_data + iterate, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
				if (i2cstat != CY_SCB_I2C_SUCCESS) break;
            }
			Cy_SCB_I2C_MasterSendStop(base, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
            
            i2c_master_end_of_receive_callback();
			//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Read Success\r\n");
        }
        else
        {            
            i2c_master_error_detected_callback();
            status = PAL_STATUS_FAILURE;
            if (i2cstat == CY_SCB_I2C_MASTER_MANUAL_ADDR_NAK)
            {
                // Address NAK must follow with a STOP
				Cy_SCB_I2C_MasterSendStop(base, QI_TIMER_AUTH_I2C_WRITE_TIMEOUT, &i2cContext);
            }
			//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal I2C_Read Failure\r\n");
        }
       
    }
    /*else
    {
        debug_print("\r\nI2C acquire Failed\r\n");   
    }*/
    
//    status = PAL_STATUS_I2C_BUSY;
//    ((upper_layer_callback_t)(p_i2c_context->upper_layer_event_handler))
//                                                        (p_i2c_context->p_upper_layer_ctx , PAL_I2C_EVENT_BUSY);
    
    return status;
}

pal_status_t pal_i2c_set_bitrate(const pal_i2c_t * p_i2c_context, uint16_t bitrate)
{
    pal_status_t return_status = PAL_STATUS_FAILURE;
    optiga_lib_status_t event = PAL_I2C_EVENT_ERROR;

    //Acquire the I2C bus before setting the bitrate
    if (PAL_STATUS_SUCCESS == pal_i2c_acquire(p_i2c_context))
    {
        // If the user provided bitrate is greater than the I2C master hardware maximum supported value,
        // set the I2C master to its maximum supported value.
        if (bitrate > PAL_I2C_MASTER_MAX_BITRATE)
        {
            bitrate = PAL_I2C_MASTER_MAX_BITRATE;
        }
        // This function is NOT absolutely required for the correct working of the system, but it's recommended
        // to implement it, though
        
        return_status = PAL_STATUS_SUCCESS;
        event = PAL_I2C_EVENT_SUCCESS;

    }
    else
    {
        return_status = PAL_STATUS_I2C_BUSY;
        event = PAL_I2C_EVENT_BUSY;
    }
    if (0 != p_i2c_context->upper_layer_event_handler)
    {
        ((callback_handler_t)(p_i2c_context->upper_layer_event_handler))(p_i2c_context->p_upper_layer_ctx , event);
    }
    //Release I2C Bus if its acquired 
    if (PAL_STATUS_I2C_BUSY != return_status)
    {
        pal_i2c_release((void * )p_i2c_context);
    }
    return return_status;
}

/**
* @}
*/
