/*++

Copyright (C) Microsoft Corporation, All Rights Reserved

Module Name:

    vhidmini.h

Abstract:

    This module contains the type definitions for the driver

Environment:

    Windows Driver Framework (WDF)

--*/

#ifdef _KERNEL_MODE
#include <ntddk.h>
#else
#include <windows.h>
#endif

#include <ntddk.h>
#include <wdm.h>
#include <ntstrsafe.h>

#include <spb.h>
#include <wdf.h>

#include <hidport.h>  // located in $(DDK_INC_PATH)/wdm

#include "common.h"

#define RESHUB_USE_HELPER_ROUTINES
#include "reshub.h"

#define DEFAULT_SPB_BUFFER_SIZE 256

#define GOODIX_SPI_READ         0xF1
#define GOODIX_SPI_WRITE        0xF0
#define ISP_RAM_ADDR			0x18400
#define GOODIX_FW_VERSION_ADDR  0x10014
#define GOODIX_IC_INFO_ADDR     0x10070
#define TOUCH_INFO_ADDR         0x10308
#define CMD_ADDR                0x10180
#define TOUCH_POOL_TAG          (ULONG)'dooG'

#define GOODIX_REPORT_RATE_120HZ 0
#define GOODIX_REPORT_RATE_240HZ 1
#define GOODIX_REPORT_RATE_360HZ 2
#define GOODIX_REPORT_RATE_480HZ 3
#define GOODIX_REPORT_RATE_960HZ 4

#define GOODIX_CMD_ACK_BUSY             0x02
#define GOODIX_CMD_ACK_BUFFER_OVERFLOW  0x03
#define GOODIX_CMD_ACK_CHECKSUM_ERROR   0x04
#define GOODIX_CMD_ACK_OK               0x80
#define GOODIX_CMD_RETRY                6

#define GOODIX_CMD_MAX_DATA_LEN         10
#define GOODIX_CMD_MAX_BUF_LEN          16

typedef struct _GOODIX_FW_VERSION {
    UINT8 RomPid[6];
    UINT8 RomVid[3];
    UINT8 RomVidReserved;
    UINT8 PatchPid[8];
    UINT8 PatchVid[4];
    UINT8 PatchVidReserved;
    UINT8 SensorId;
    UINT8 Reserved[2];
    UINT16 Checksum;
} GOODIX_FW_VERSION, *PGOODIX_FW_VERSION;

#pragma warning(push)
#pragma warning(disable:4201)
#pragma pack(push, 1)
typedef struct _GOODIX_CMD_PACKET {
    union {
        struct {
            UINT8 State;
            UINT8 Ack;
            UINT8 Length;
            UINT8 Command;
            UINT8 Data[GOODIX_CMD_MAX_DATA_LEN];
        };
        UINT8 Buffer[GOODIX_CMD_MAX_BUF_LEN];
    };
} GOODIX_CMD_PACKET, *PGOODIX_CMD_PACKET;
#pragma pack(pop)
#pragma warning(pop)

typedef UCHAR HID_REPORT_DESCRIPTOR, *PHID_REPORT_DESCRIPTOR;

DRIVER_INITIALIZE                   DriverEntry;
EVT_WDF_DRIVER_DEVICE_ADD           EvtDeviceAdd;
EVT_WDF_TIMER                       EvtTimerFunc;
EVT_WDF_OBJECT_CONTEXT_CLEANUP      EvtDriverCleanup;

EVT_WDF_DEVICE_PREPARE_HARDWARE      OnPrepareHardware;
EVT_WDF_DEVICE_RELEASE_HARDWARE      OnReleaseHardware;
EVT_WDF_DEVICE_D0_ENTRY              OnD0Entry;
EVT_WDF_DEVICE_D0_EXIT               OnD0Exit;

typedef struct _DEVICE_CONTEXT
{
    WDFDEVICE               Device;
    WDFQUEUE                DefaultQueue;
    WDFQUEUE                ManualQueue;
    HID_DEVICE_ATTRIBUTES   HidDeviceAttributes;
    BYTE                    DeviceData;
    HID_DESCRIPTOR          HidDescriptor;
    PHID_REPORT_DESCRIPTOR  ReportDescriptor;
    BOOLEAN                 ReadReportDescFromRegistry;

    LARGE_INTEGER           PeripheralId;
    WDFINTERRUPT            Interrupt;
    WDFIOTARGET             SpbController;
    WDFWAITLOCK             IoLock;
    BOOLEAN                 OnClose;
    UINT8                   LastTouchID;
    UINT8                   ReportRateLevel;
    UINT8                   ActiveReportRateLevel;
    UINT8                   SensorId;
    BOOLEAN                 VersionValid;
    ULONG                   TouchDataAddress;
    ULONG                   CommandAddress;
} DEVICE_CONTEXT, *PDEVICE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(DEVICE_CONTEXT, GetDeviceContext);

typedef struct _QUEUE_CONTEXT
{
    WDFQUEUE                Queue;
    PDEVICE_CONTEXT         DeviceContext;
    UCHAR                   OutputReport;

} QUEUE_CONTEXT, *PQUEUE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(QUEUE_CONTEXT, GetQueueContext);

NTSTATUS
QueueCreate(
    _In_  WDFDEVICE         Device,
    _Out_ WDFQUEUE          *Queue
    );

typedef struct _MANUAL_QUEUE_CONTEXT
{
    WDFQUEUE                Queue;
    PDEVICE_CONTEXT         DeviceContext;
    WDFTIMER                Timer;

} MANUAL_QUEUE_CONTEXT, *PMANUAL_QUEUE_CONTEXT;

WDF_DECLARE_CONTEXT_TYPE_WITH_NAME(MANUAL_QUEUE_CONTEXT, GetManualQueueContext);

NTSTATUS
ManualQueueCreate(
    _In_  WDFDEVICE         Device,
    _Out_ WDFQUEUE          *Queue
    );

NTSTATUS
ReadReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request,
    _Always_(_Out_)
          BOOLEAN*          CompleteRequest
    );

NTSTATUS
WriteReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    );

NTSTATUS
GetFeature(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    );

NTSTATUS
SetFeature(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    );

NTSTATUS
GetInputReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    );

NTSTATUS
SetOutputReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    );

NTSTATUS
GetString(
    _In_  WDFREQUEST        Request
    );

NTSTATUS
GetIndexedString(
    _In_  WDFREQUEST        Request
    );

NTSTATUS
GetStringId(
    _In_  WDFREQUEST        Request,
    _Out_ ULONG            *StringId,
    _Out_ ULONG            *LanguageId
    );

NTSTATUS
RequestCopyFromBuffer(
    _In_  WDFREQUEST        Request,
    _In_  PVOID             SourceBuffer,
    _When_(NumBytesToCopyFrom == 0, __drv_reportError(NumBytesToCopyFrom cannot be zero))
    _In_  size_t            NumBytesToCopyFrom
    );

NTSTATUS
RequestGetHidXferPacket_ToReadFromDevice(
    _In_  WDFREQUEST        Request,
    _Out_ HID_XFER_PACKET  *Packet
    );

NTSTATUS
RequestGetHidXferPacket_ToWriteToDevice(
    _In_  WDFREQUEST        Request,
    _Out_ HID_XFER_PACKET  *Packet
    );

BOOLEAN
OnInterruptIsr(
    _In_  WDFINTERRUPT FxInterrupt,
    _In_  ULONG        MessageID
);

NTSTATUS
SpbDeviceOpen(
    _In_  PDEVICE_CONTEXT  pDevice
);
VOID
SpbDeviceClose(
    _In_  PDEVICE_CONTEXT  pDevice
);

NTSTATUS
SpbDeviceWrite(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ PVOID pInputBuffer,
    _In_ size_t inputBufferLength
);

NTSTATUS
SpbDeviceWriteRead(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ PVOID pInputBuffer,
    _In_ PVOID pOutputBuffer,
    _In_ size_t inputBufferLength,
    _In_ size_t outputBufferLength
);

NTSTATUS
GoodixRead(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ UINT32 addr,
    _In_ UINT8* readBuf,
    _In_ UINT32 readLen
);

NTSTATUS
GoodixWrite(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ UINT32 addr,
    _In_ UINT8* writeBuf,
    _In_ UINT32 writeLen
);

NTSTATUS
GoodixReadVersion(
    _In_ PDEVICE_CONTEXT pDevice,
    _Out_ PGOODIX_FW_VERSION Version
    );

NTSTATUS
GoodixApplyReportRate(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ UINT8 ReportRateLevel
    );

NTSTATUS
ReadDescriptorFromRegistry(
    WDFDEVICE Device
);

//
// Misc definitions
//
#define CONTROL_FEATURE_REPORT_ID   0x54

//
// These are the device attributes returned by the mini driver in response
// to IOCTL_HID_GET_DEVICE_ATTRIBUTES.
//
#define HIDMINI_PID             0xFEED
#define HIDMINI_VID             0xDEED
#define HIDMINI_VERSION         0x0101
