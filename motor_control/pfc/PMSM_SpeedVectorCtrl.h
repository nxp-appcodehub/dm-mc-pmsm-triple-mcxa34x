/******************************************************************************
* 
* Copyright (c) 2013 - 2016 Freescale Semiconductor;
* All Rights Reserved
* SPDX-License-Identifier: BSD-3-Clause                       
*
******************************************************************************* 
*
* THIS SOFTWARE IS PROVIDED BY FREESCALE "AS IS" AND ANY EXPRESSED OR 
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES 
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  
* IN NO EVENT SHALL FREESCALE OR ITS CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
* IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF 
* THE POSSIBILITY OF SUCH DAMAGE.
*
***************************************************************************//*!
*
* @file      PMSM_SpeedVectorCtrl.h
* 
* @version   1.0.2.0
* 
* @date      May-23-2013
* 
* @brief     Macros definitions for application configuration
*
***************************************************************************
*
*  Application configuration file
*
*****************************************************************************/

#ifndef _PMSM_SPEEDVECTORCTRL_H_
#define _PMSM_SPEEDVECTORCTRL_H_

/*************** APP *************/
#define APP_FAST_CONTROL_LOOP_FREQ          500.0	/* Fast loop frequency [Hz] */
#define APP_SLOW_CONTROL_LOOP_FREQ          20.0	/* Slow loop frequency [Hz] */

#define APP_V_DCB_SCALE                     457.97	/* MAX measurable DCB voltage [V] */
#define APP_COMPRESSOR_SPEED_SCALE          M1_SPEED_SCALE
#define APP_FAN_SPEED_SCALE                 M2_SPEED_SCALE
#define APP_TEMP_SCALE                      128.0	/* Scale for temperatures [deg C]*/

#define APP_OVERVOLT_LIMIT                  420.0 	/* Max dc bus voltage [V]*/
#define APP_UNDERVOLT_LIMIT                 127.0 	/* Min dc bus voltage [V] */
#define APP_OVERTEMP_LIMIT                  57.0	/* Hot side max permitted temperature [deg C] */ 

#define APP_COMPRESSOR_SPEED_MAX            4500.0 	/* Max compressor speed [RPM] */
#define APP_COMPRESSOR_SPEED_MIN            1200.0 	/* Min compressor speed [RPM] */

#define APP_FAN_SPEED_MAX                   2000.0 	/* Max fan speed [RPM]*/
#define APP_FAN_SPEED_MIN                   500.0 	/* Min fan speed [RPM] */
#define APP_FAN_SPEED_ON                    1250.0 	/* Fan speed command where the compressor is turned on [RPM] */
#define APP_COMPRESSOR_SPEED_FAN_ON         M1_SPEED_MIN /* Min compressor speed to allow fan's turning on [RPM] */ 

#define APP_TEMP_SPEED_MIN                  30.0 	/* Min fan speed temperature [deg C] */
#define APP_TEMP_SPEED_MAX                  50.0 	/* Max fan speed temperature [deg C]  */

#define APP_HOT_SIDE_TEMP                   35.0	/* Level  [deg C]  to be maintained during refrigeration on hot side */ 

#define APP_DURATION_TASK_DEFAULT           0
#define APP_DURATION_TASK_FAULT_RELEASE     2.0		/* Duration after fault clear [s] */

#define APP_FILTER_UDCBUS_WINDOW            6		/* DC bus voltage filter shift */
#define APP_FILTER_HOT_TEMP_WINDOW          6		/* Hot side temperature filter shift */
#define APP_FILTER_COLD_TEMP_WINDOW 	    6		/* Cold side temperature filter shift */
#define APP_FILTER_AMBIENT_TEMP_WINDOW 	    6		/* Ambient side temperature filter shift */

/*************** COMMUNICATION *************/
//#define COMMUNICATION
#define COMM_USE_TX_DMA
#define COMM_CONTROL_LOOP_FREQ              10000.0	/* Control loop frequency [Hz] */
#define COMM_BUFFER_SIZE                    600		/* Phase current data record buffer size [bytes] */
#define COMM_FREQ                           200.0	/* Communication values update frequency [Hz] */
#define COMM_RECORDER_POINTS                200		/* Phase current recorder points  */

#define COMM_FILTER_IPFC_WINDOW             9		/* PFC current filter shift */
#define COMM_FILTER_IM1_WINDOW              9		/* Motor 1 current filter shift */
#define COMM_FILTER_IM2_WINDOW              9		/* Motor 2 current filter shift */


#endif /* _PMSM_SPEEDVECTORCTRL_H_ */
