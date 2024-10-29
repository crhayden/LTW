/*************************************************************
 * Copyright (c) 2019 Apple Enterprises. This code is
 * proprietary and a trade secret of Apple Enterprises.
 *
 * $Workfile: Client.h $
 *
 * $Creator: Chris Apple $
 *
 * $Description: Handles high level USB HID communication (HID Reports). $
 *
 * $NoKeywords: $
 *************************************************************/

#ifndef __CLIENT_H__
#define __CLIENT_H__

#ifdef __cplusplus
extern "C"
{
#endif

extern void CLIENT_GetStatusReport( uint8_t *responsePacket );
extern void CLIENT_ProcessCommand( uint32_t hostCommand, const uint8_t *commandPacket);
extern bool shouldAllowReportingFromDevice;
extern bool isFirstReadFromHost;

#ifdef __cplusplus
}
#endif


#endif /* end __CLIENT_H__ */

/******************************************************************************
**                            End Of File
******************************************************************************/
