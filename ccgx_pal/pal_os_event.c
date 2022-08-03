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
* \file pal_os_event.c
*
* \brief   This file implements the platform abstraction layer APIs for os event/scheduler.
*
* \ingroup  grPAL
*
* @{
*/

#include "optiga/pal/pal_os_event.h"
#include "cy_qistack_auth_optiga.h"

static pal_os_event_t pal_os_event_0 = {0, NULL, NULL, NULL};

void pal_os_event_start(pal_os_event_t * p_pal_os_event, register_callback callback, void * callback_args)
{
    if (0 == p_pal_os_event->is_event_triggered)
    {
        p_pal_os_event->is_event_triggered = TRUE;
        pal_os_event_register_callback_oneshot(p_pal_os_event,callback,callback_args,1000);
    }
}

void pal_os_event_stop(pal_os_event_t * p_pal_os_event)
{
    p_pal_os_event->is_event_triggered = 0;
}

pal_os_event_t * pal_os_event_create(register_callback callback, void * callback_args)
{
    if (( NULL != callback )&&( NULL != callback_args ))
    {
        pal_os_event_start(&pal_os_event_0,callback,callback_args);
    }
    return (&pal_os_event_0);
}

void pal_os_event_cbk(cy_timer_id_t id, void * ctx)
{
    (void) id;
    (void) ctx;
    pal_os_event_trigger_registered_callback();
}

void pal_os_event_trigger_registered_callback(void)
{
    register_callback callback;

    // User should take care to stop the timer if it sin't stoped automatically
    
    if (pal_os_event_0.callback_registered)
    {
        callback = pal_os_event_0.callback_registered;
        callback((void * )pal_os_event_0.callback_ctx);
    }

	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal callback \r\n");
}


void pal_os_event_register_callback_oneshot(pal_os_event_t * p_pal_os_event,
                                             register_callback callback,
                                             void * callback_args,
                                             uint32_t time_us)
{
	wicg_qi_authentication_context_param *auth_params = wicg_qi_auth_params_get();

    p_pal_os_event->callback_registered = callback;
    p_pal_os_event->callback_ctx = callback_args;


    // User should start the timer here with the 
    // pal_os_event_trigger_registered_callback() function as a callback
    // Attention. Please don't call directly the callback from here, otherwise you stack migth grow very fast
    
    if (time_us < 1000 ) time_us = 1000;  
	cy_sw_timer_start(auth_params->callbackTimerContext, auth_params->qiCtx, CY_QI_TIMER_PKT_AUTH_TIMEOUT_TIME_ID, (time_us / 1000), pal_os_event_cbk);
	//Cy_Console_Printf(qiCtx, CY_QI_UART_VERBO_LVL_CRITICAL, "pal oneshot %d mSec\r\n", (time_us/1000));
}

void pal_os_event_destroy(pal_os_event_t * pal_os_event)
{

	wicg_qi_authentication_context_param* auth_params = wicg_qi_auth_params_get();
    (void)pal_os_event;
    // User should take care to destroy the event if it's not required
   
	if (cy_sw_timer_is_running(auth_params->callbackTimerContext, CY_QI_TIMER_PKT_AUTH_TIMEOUT_TIME_ID))
		cy_sw_timer_stop(auth_params->callbackTimerContext, CY_QI_TIMER_PKT_AUTH_TIMEOUT_TIME_ID);
}

/**
* @}
*/
