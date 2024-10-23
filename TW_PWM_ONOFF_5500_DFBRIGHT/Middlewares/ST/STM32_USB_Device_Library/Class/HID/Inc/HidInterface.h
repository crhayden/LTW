/*************************************************************
 * Copyright (c) 2019 Apple Enterprises. This code is
 * proprietary and a trade secret of Apple Enterprises.
 *
 * $Workfile:$
 *
 * $Creator: Chris Apple $
 *
 * $Description: Interface for HID Reports. $
 *
 * $Log:$
 *
 * $NoKeywords: $
 *************************************************************/
#ifndef __HIDINTERFACE_H__
#define __HIDINTERFACE_H__

#include "stdint.h"
#include "stdbool.h"

#pragma pack(push,1)        // force single-byte packing of structure fields

////////////////////////////////////////////////////////////////////////////////
// Constants
//

//#define HID_REPORT_SIZE 64              // all HID CTRL reports are fixed-size


////////////////////////////////////////////////////////////////////////////////
// Standard 8-bit packet header common to all HID CTRL endpoint reports
//
typedef struct
{
  uint8_t hidReportID;         // HID report length, including header
  uint8_t hidOpcode;         // HID report opcode
} HID_RptHdr;
#define USB_HID_HEADER_SIZE sizeof(HID_RptHdr)

// HID report packet IDs are placed in teh first byte for HID transfers
typedef enum _HID_REPORT_IDS {
  HID_REPORT_ID_STATUS = 1,  // data size is just the header
  HID_REPORT_ID_NO_DATA = 2,  // data size is just the header
  HID_REPORT_ID_SHORT = 3,    // data size is up to 8 bytes plus header
  HID_REPORT_ID_MEDIUM = 4,    // data size is bewtten 9 and 32 plus header
  HID_REPORT_ID_LONG = 5,      // data size is bewtten 33 and 60 plus header
  HID_REPORT_ID_SECTOR = 6,    // data size is size of flash sector Plus maximum command size plus header
  HID_REPORT_ID_TOTAL
} HID_REPORT_IDS;

// always make sure that the total length of a packet plus the header is not a multiple of 64.
typedef enum _HID_REPORT_LENGTHS {
  HID_REPORT_LENGTH_STATUS = 62,    // status sent every interrupt
  HID_REPORT_LENGTH_NO_DATA = 0,    // no data size just the header
  HID_REPORT_LENGTH_SHORT = 12,     // data size is up to 12 bytes plus header
  HID_REPORT_LENGTH_MEDIUM = 60,    // data size is between 12 and 56 plus header (one 64-byte packet)
  HID_REPORT_LENGTH_LONG = 120,     // data size is between 56 and 120 plus header (two 64-byte packets)
  HID_REPORT_LENGTH_SECTOR = 2048,  // data size is size of flash sector
  HID_REPORT_LENGTH_MAXIMUM = HID_REPORT_LENGTH_SECTOR
} HID_REPORT_LENGTHS;

// define size of maximum packet expected from either HID or CDC interface
#define USB_MAXIMUM_HOST_TO_DEVICE_BUFFER_SIZE    HID_REPORT_LENGTH_MAXIMUM
#define USB_MAXIMUM_HOST_TO_DEVICE_PACKET_SIZE    (HID_REPORT_LENGTH_MAXIMUM + USB_HID_HEADER_SIZE)
#define USB_MAXIMUM_DEVICE_TO_HOST_BUFFER_SIZE    HID_REPORT_LENGTH_MAXIMUM
#define USB_MAXIMUM_DEVICE_TO_HOST_PACKET_SIZE    (USB_MAXIMUM_DEVICE_TO_HOST_BUFFER_SIZE + USB_HID_HEADER_SIZE)

#pragma pack(pop)       // restore default structure field alignment

#endif // __HIDINTERFACE_H__
