/*****************************************************************************
 *
 * MODULE:              WSN_Profile.h
 *
 * COMPONENT:           $RCSfile: WSN_Profile.h,v $
 *
 * VERSION:             $Name:  $
 *
 * REVISION:            $Revision: 1.6 $
 *
 * DATED:               $Date: 2007/07/12 11:03:03 $
 *
 * STATUS:              $State: Exp $
 *
 * AUTHOR:
 *
 * DESCRIPTION:
 *
 * Update history
 * $Log: WSN_Profile.h,v $
 * Revision 1.6  2007/07/12 11:03:03  ndani
 * Add simple descriptor after network has started
 *
 *
 * LAST MODIFIED BY:    $Author: ndani $
 *                      $Modtime: $
 *
 ****************************************************************************
 *
 * This software is owned by Jennic and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on Jennic products. You, and any third parties must reproduce
 * the copyright and warranty notice and any other legend of ownership on each
 * copy or partial copy of the software.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS". JENNIC MAKES NO WARRANTIES, WHETHER
 * EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE,
 * ACCURACY OR LACK OF NEGLIGENCE. JENNIC SHALL NOT, IN ANY CIRCUMSTANCES,
 * BE LIABLE FOR ANY DAMAGES, INCLUDING, BUT NOT LIMITED TO, SPECIAL,
 * INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON WHATSOEVER.
 *
 * Copyright Jennic Ltd 2005, 2006. All rights reserved
 *
 ****************************************************************************/
#ifndef  WSN_PROFILE_H_INCLUDED
#define  WSN_PROFILE_H_INCLUDED

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define WSN_PROFILE_ID              0x123
#define WSN_CID_SENSOR_READINGS     0x12
#define WSN_PAN_ID				    0xAFED
#define WSN_CHANNEL				    17
#define WSN_DATA_SINK_ENDPOINT      0x40
#define WSN_DATA_SOURCE_ENDPOINT    0x41
#define CONTROL_CMD_GET_INFO        1
#define CONTROL_CMD_GET_DATA_S      2
#define CONTROL_CMD_GET_DATA_M      3

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif  /* WSN_PROFILE_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
