/*++

Copyright (C) Microsoft Corporation, All Rights Reserved.

Module Name:

    vhidmini.cpp

Abstract:

    This module contains the implementation of the driver

Environment:

    Windows Driver Framework (WDF)

--*/
#include <initguid.h>
#include "vhidmini.h"
#include "goodix_9916r_blobs.h"
#include "goodix_9916r_firmware.h"
#include "kmdf/trace.h"
#include "vhidmini.tmh"

#define EVT_ID_NOEVENT						0x00	/*No Events*/
#define EVT_ID_CONTROLLER_READY				0x03	/*Controller ready, issued after a system reset.*/
#define EVT_ID_ENTER_POINT					0x13	/*Touch enter in the sensing area*/
#define EVT_ID_MOTION_POINT					0x23	/*Touch motion (a specific touch changed position)*/
#define EVT_ID_LEAVE_POINT					0x33	/*Touch leave the sensing area*/


#define GOODIX_TOUCH_EVENT 0x80
#define GOODIX_REQUEST_EVENT 0x40
#define GOODIX_GESTURE_EVENT    0x20
#define GOODIX_HOTKNOT_EVENT    0x10
#define GOODIX_DEFERRED_REQUEST_RECOVER 0xFF
#define BYTES_PER_COORD 0x8
#define BYTES_CHKSUM 0x2
#define MAX_POINT_NUM 0xA

#define GOODIX_CFG_BIN_HEAD_LEN            16U
#define GOODIX_CFG_BIN_VERSION_START       5U
#define GOODIX_CFG_PACKAGE_SENSOR_ANY      0xFFU
#define GOODIX_CFG_PACKAGE_CONST_INFO_LEN  56U
#define GOODIX_CFG_PACKAGE_REG_INFO_LEN    65U
#define GOODIX_CFG_PACKAGE_DATA_OFFSET     (GOODIX_CFG_PACKAGE_CONST_INFO_LEN + GOODIX_CFG_PACKAGE_REG_INFO_LEN)
#define GOODIX_CFG_TYPE_NORMAL             0x01U
#define GOODIX_CFG_CMD_LEN                 4U
#define GOODIX_CFG_CMD_START               0x04U
#define GOODIX_CFG_CMD_WRITE               0x05U
#define GOODIX_CFG_CMD_EXIT                0x06U
#define GOODIX_CFG_CMD_STATUS_PASS         0x80U
#define GOODIX_CFG_CMD_WAIT_RETRY          20U
#define GOODIX_CFG_TRANSFER_CHUNK          64U
#define GOODIX_CFG_TRANSFER_DELAY_MS       1U
#define GOODIX_SPB_XFER_TIMEOUT_MS         200U
#define GOODIX_TRANSPORT_FAILURE_THRESHOLD 3U

#define GOODIX_FW_HEADER_SIZE              512U
#define GOODIX_FW_SUBSYS_INFO_SIZE         10U
#define GOODIX_FW_SUBSYS_INFO_OFFSET       42U
#define GOODIX_FW_SUBSYS_MAX_NUM           47U
#define GOODIX_FW_FILE_CHECKSUM_OFFSET     8U
#define GOODIX_FW_PACKET_MAX_SIZE          4096U

#define GOODIX_FW_BRD_CHIP_TYPE            0x98U
#define GOODIX_FW_ISP_RAM_ADDR_BRD         0x23800U
#define GOODIX_FW_CPU_RUN_FROM_REG         0x10000U
#define GOODIX_FW_FLASH_CMD_REG_BRD        0x12400U
#define GOODIX_FW_ISP_BUFFER_REG_BRD       0x12410U
#define GOODIX_FW_MISCTL_REG_BRD           0xD804U
#define GOODIX_FW_WATCHDOG_REG_BRD         0xD040U
#define GOODIX_FW_HOLD_CPU_REG_W           0x0002U
#define GOODIX_FW_HOLD_CPU_REG_R           0x2000U
#define GOODIX_FW_ENABLE_MISCTL_BRD        0x20700000U

#define GOODIX_FLASH_CMD_TYPE_WRITE        0xBBU
#define GOODIX_FLASH_CMD_ACK_CHK_PASS      0xEEU
#define GOODIX_FLASH_CMD_STATUS_WRITE_OK   0xEEU
#define GOODIX_FLASH_CMD_STATUS_CHK_FAIL   0x33U
#define GOODIX_FLASH_CMD_STATUS_ADDR_ERR   0x44U
#define GOODIX_FLASH_CMD_STATUS_WRITE_ERR  0x55U
#define GOODIX_FLASH_CMD_LEN               11U

typedef struct _GOODIX_FW_SUBSYS {
    UINT8 Type;
    UINT32 Size;
    UINT32 FlashAddress;
    const UINT8* Data;
} GOODIX_FW_SUBSYS, *PGOODIX_FW_SUBSYS;

typedef struct _GOODIX_PARSED_FW {
    UINT8 FwPid[8];
    UINT8 FwVid[4];
    UINT8 SubsysNum;
    UINT8 ChipType;
    UINT8 ProtocolVer;
    UINT8 BusType;
    UINT8 FlashProtect;
    GOODIX_FW_SUBSYS Subsys[GOODIX_FW_SUBSYS_MAX_NUM];
} GOODIX_PARSED_FW, *PGOODIX_PARSED_FW;

static NTSTATUS GoodixOpenResetGpio(_In_ PDEVICE_CONTEXT DeviceContext);
static VOID GoodixCloseResetGpio(_In_ PDEVICE_CONTEXT DeviceContext);
static NTSTATUS GoodixResetDevice(_In_ PDEVICE_CONTEXT DeviceContext, _In_ ULONG DelayMs);
static NTSTATUS GoodixParseEmbeddedFirmware(_Out_ PGOODIX_PARSED_FW ParsedFirmware);
static NTSTATUS GoodixMaybeUpdateFirmware(_In_ PDEVICE_CONTEXT DeviceContext, _Inout_ PGOODIX_FW_VERSION Version);
static VOID GoodixHandleControllerRequest(_In_ PDEVICE_CONTEXT DeviceContext, _In_ UINT8 RequestCode);
static VOID GoodixQueueControllerRequest(_In_ PDEVICE_CONTEXT DeviceContext, _In_ UINT8 RequestCode);
static NTSTATUS GoodixCreateControlDevice(_In_ WDFDRIVER Driver);
static VOID GoodixSetActiveTouchDevice(_In_ WDFDRIVER Driver, _In_opt_ WDFDEVICE Device);
static PDEVICE_CONTEXT GoodixAcquireActiveDeviceContext(_In_ WDFDRIVER Driver, _Out_opt_ WDFDEVICE* ReferencedDevice);
static VOID GoodixReleaseActiveDevice(_In_opt_ WDFDEVICE Device);
static NTSTATUS GoodixReinitializeAfterReportRateChange(_In_ PDEVICE_CONTEXT DeviceContext);

typedef struct _GOODIX_CFG_PACKAGE_INFO {
    const UINT8* ConfigData;
    ULONG ConfigLength;
    UINT8 ConfigType;
    UINT8 SensorId;
} GOODIX_CFG_PACKAGE_INFO, *PGOODIX_CFG_PACKAGE_INFO;

ULONG XRevert = 0;
ULONG YRevert = 0;
ULONG XYExchange = 0;
ULONG XMin = 0;
ULONG XMax = 1080;
ULONG YMin = 0;
ULONG YMax = 2400;

static
VOID
GoodixDelayMilliseconds(
    _In_ LONG Milliseconds
    )
{
    LARGE_INTEGER interval;

    interval.QuadPart = -(10 * 1000 * Milliseconds);
    KeDelayExecutionThread(KernelMode, FALSE, &interval);
}

static
VOID
GoodixInitRequestSendOptions(
    _Out_ WDF_REQUEST_SEND_OPTIONS* RequestSendOptions,
    _In_ ULONG TimeoutMs
    )
{
    WDF_REQUEST_SEND_OPTIONS_INIT(
        RequestSendOptions,
        WDF_REQUEST_SEND_OPTION_TIMEOUT);
    WDF_REQUEST_SEND_OPTIONS_SET_TIMEOUT(
        RequestSendOptions,
        WDF_REL_TIMEOUT_IN_MS(TimeoutMs));
}

static
VOID
GoodixAppendChecksumU8Le(
    _Inout_updates_bytes_(Length + 2) PUINT8 Data,
    _In_ ULONG Length
    )
{
    ULONG checksum = 0;
    ULONG i;

    for (i = 0; i < Length; i++) {
        checksum += Data[i];
    }

    Data[Length] = (UINT8)(checksum & 0xFF);
    Data[Length + 1] = (UINT8)((checksum >> 8) & 0xFF);
}

static
VOID
GoodixAppendChecksumU16Le(
    _Inout_updates_bytes_(Length + 4) PUINT8 Data,
    _In_ ULONG Length
    )
{
    ULONG checksum = 0;
    ULONG i;

    for (i = 0; i < Length; i += 2) {
        USHORT value = Data[i];

        if ((i + 1) < Length) {
            value |= ((USHORT)Data[i + 1] << 8);
        }

        checksum += value;
    }

    Data[Length] = (UINT8)(checksum & 0xFF);
    Data[Length + 1] = (UINT8)((checksum >> 8) & 0xFF);
    Data[Length + 2] = (UINT8)((checksum >> 16) & 0xFF);
    Data[Length + 3] = (UINT8)((checksum >> 24) & 0xFF);
}

static
UINT16
GoodixReadU16Le(
    _In_reads_bytes_(2) const UINT8* Data
    )
{
    return (UINT16)Data[0] | ((UINT16)Data[1] << 8);
}

static
UINT32
GoodixReadU32Le(
    _In_reads_bytes_(4) const UINT8* Data
    )
{
    return (UINT32)Data[0] |
        ((UINT32)Data[1] << 8) |
        ((UINT32)Data[2] << 16) |
        ((UINT32)Data[3] << 24);
}

static
BOOLEAN
GoodixChecksumValidU8Le(
    _In_reads_bytes_(Length) const UINT8* Data,
    _In_ ULONG Length
    )
{
    ULONG checksum = 0;
    ULONG i;

    if (Length < 2) {
        return FALSE;
    }

    for (i = 0; i < Length - 2; i++) {
        checksum += Data[i];
    }

    return (((UINT16)checksum) == ((UINT16)Data[Length - 2] | ((UINT16)Data[Length - 1] << 8)));
}

static
BOOLEAN
GoodixConfigBinValid(
    _In_reads_bytes_(Length) const UINT8* Data,
    _In_ ULONG Length
    )
{
    ULONG checksum = 0;
    ULONG i;

    if (Length < GOODIX_CFG_BIN_HEAD_LEN) {
        return FALSE;
    }

    if (GoodixReadU32Le(Data) != Length) {
        return FALSE;
    }

    for (i = GOODIX_CFG_BIN_VERSION_START; i < Length; i++) {
        checksum += Data[i];
    }

    return (((UINT8)checksum) == Data[4]);
}

static
UINT8
GoodixNormalizeReportRateLevel(
    _In_ UINT8 ReportRateLevel
    )
{
    switch (ReportRateLevel) {
    case GOODIX_REPORT_RATE_120HZ:
    case GOODIX_REPORT_RATE_240HZ:
    case GOODIX_REPORT_RATE_480HZ:
    case GOODIX_REPORT_RATE_960HZ:
        return ReportRateLevel;
    case GOODIX_REPORT_RATE_360HZ:
    default:
        return GOODIX_REPORT_RATE_240HZ;
    }
}

static
NTSTATUS
GoodixPersistReportRateLevel(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 ReportRateLevel
    )
{
    WDFKEY hKey = NULL;
    UNICODE_STRING reportRateName;
    NTSTATUS status;

    status = WdfDeviceOpenRegistryKey(
        DeviceContext->Device,
        PLUGPLAY_REGKEY_DEVICE,
        KEY_SET_VALUE,
        WDF_NO_OBJECT_ATTRIBUTES,
        &hKey);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    RtlInitUnicodeString(&reportRateName, L"ReportRateLevel");
    status = WdfRegistryAssignULong(
        hKey,
        &reportRateName,
        (ULONG)GoodixNormalizeReportRateLevel(ReportRateLevel));

    WdfRegistryClose(hKey);
    return status;
}

static
NTSTATUS
GoodixWriteLarge(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT32 Address,
    _In_reads_bytes_(Length) const UINT8* Buffer,
    _In_ ULONG Length
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG offset = 0;

    while (offset < Length) {
        ULONG chunk = Length - offset;
        if (chunk > GOODIX_CFG_TRANSFER_CHUNK) {
            chunk = GOODIX_CFG_TRANSFER_CHUNK;
        }

        status = GoodixWrite(
            DeviceContext,
            Address + offset,
            (UINT8*)(Buffer + offset),
            chunk);
        if (!NT_SUCCESS(status)) {
            return status;
        }

        offset += chunk;

        if (offset < Length) {
            GoodixDelayMilliseconds(GOODIX_CFG_TRANSFER_DELAY_MS);
        }
    }

    return STATUS_SUCCESS;
}

static
NTSTATUS
GoodixReadLarge(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT32 Address,
    _Out_writes_bytes_(Length) UINT8* Buffer,
    _In_ ULONG Length
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG offset = 0;

    while (offset < Length) {
        ULONG chunk = Length - offset;
        if (chunk > GOODIX_CFG_TRANSFER_CHUNK) {
            chunk = GOODIX_CFG_TRANSFER_CHUNK;
        }

        status = GoodixRead(
            DeviceContext,
            Address + offset,
            Buffer + offset,
            chunk);
        if (!NT_SUCCESS(status)) {
            return status;
        }

        offset += chunk;

        if (offset < Length) {
            GoodixDelayMilliseconds(GOODIX_CFG_TRANSFER_DELAY_MS);
        }
    }

    return STATUS_SUCCESS;
}

static
NTSTATUS
GoodixSendCommand(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _Inout_ PGOODIX_CMD_PACKET Command
    )
{
    GOODIX_CMD_PACKET ack = { 0 };
    NTSTATUS status = STATUS_IO_TIMEOUT;
    ULONG retry;
    ULONG poll;

    WdfWaitLockAcquire(DeviceContext->IoLock, NULL);

    Command->State = 0;
    Command->Ack = 0;
    GoodixAppendChecksumU8Le(&Command->Buffer[2], Command->Length - 2);

    for (retry = 0; retry < GOODIX_CMD_RETRY; retry++) {
        status = GoodixWrite(DeviceContext, DeviceContext->CommandAddress, Command->Buffer, sizeof(*Command));
        if (!NT_SUCCESS(status)) {
            break;
        }

        for (poll = 0; poll < GOODIX_CMD_RETRY; poll++) {
            status = GoodixRead(DeviceContext, DeviceContext->CommandAddress, ack.Buffer, sizeof(ack));
            if (!NT_SUCCESS(status)) {
                break;
            }

            if (ack.Ack == GOODIX_CMD_ACK_OK) {
                GoodixDelayMilliseconds(40);
                status = STATUS_SUCCESS;
                goto Exit;
            }

            if (ack.Ack == GOODIX_CMD_ACK_BUSY || ack.Ack == 0x00) {
                GoodixDelayMilliseconds(1);
                continue;
            }

            if (ack.Ack == GOODIX_CMD_ACK_BUFFER_OVERFLOW) {
                GoodixDelayMilliseconds(10);
            } else if (ack.Ack == GOODIX_CMD_ACK_CHECKSUM_ERROR) {
                status = STATUS_CRC_ERROR;
            } else {
                status = STATUS_INVALID_DEVICE_STATE;
            }
            break;
        }
    }

Exit:
    WdfWaitLockRelease(DeviceContext->IoLock);
    return status;
}

static
NTSTATUS
GoodixWaitConfigStatus(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 TargetStatus,
    _In_ ULONG RetryCount
    )
{
    GOODIX_CMD_PACKET ack = { 0 };
    NTSTATUS status = STATUS_IO_TIMEOUT;
    ULONG retry;

    for (retry = 0; retry < RetryCount; retry++) {
        status = GoodixRead(
            DeviceContext,
            DeviceContext->CommandAddress,
            ack.Buffer,
            sizeof(ack));
        if (!NT_SUCCESS(status)) {
            return status;
        }

        if (ack.State == TargetStatus) {
            return STATUS_SUCCESS;
        }

        GoodixDelayMilliseconds(20);
    }

    return STATUS_IO_TIMEOUT;
}

static
NTSTATUS
GoodixSendConfigCommand(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 CommandCode
    )
{
    GOODIX_CMD_PACKET cmd = { 0 };
    NTSTATUS status;

    cmd.Length = GOODIX_CFG_CMD_LEN;
    cmd.Command = CommandCode;

    status = GoodixSendCommand(DeviceContext, &cmd);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    return GoodixWaitConfigStatus(
        DeviceContext,
        GOODIX_CFG_CMD_STATUS_PASS,
        GOODIX_CFG_CMD_WAIT_RETRY);
}

static
NTSTATUS
GoodixFindConfigPackage(
    _In_ UINT8 SensorId,
    _Out_ PGOODIX_CFG_PACKAGE_INFO Package
    )
{
    ULONG pkgNum;
    ULONG pkgIndex;
    ULONG firstWildcardOffset = 0;
    ULONG selectedOffset = 0;
    ULONG binLength = g_GoodixCfgGroup9916rBinLen;
    const UINT8* binData = g_GoodixCfgGroup9916rBin;

    RtlZeroMemory(Package, sizeof(*Package));

    if (!GoodixConfigBinValid(binData, binLength)) {
        return STATUS_CRC_ERROR;
    }

    pkgNum = binData[9];
    if (pkgNum == 0) {
        return STATUS_NOT_FOUND;
    }

    for (pkgIndex = 0; pkgIndex < pkgNum; pkgIndex++) {
        ULONG offsetTable = GOODIX_CFG_BIN_HEAD_LEN + (pkgIndex * 2U);
        ULONG pkgOffset;
        ULONG nextOffset;
        ULONG pkgLength;
        UINT8 packageSensorId;

        if ((offsetTable + 1U) >= binLength) {
            return STATUS_INVALID_BUFFER_SIZE;
        }

        pkgOffset = GoodixReadU16Le(binData + offsetTable);
        if (pkgOffset >= binLength || (pkgOffset + GOODIX_CFG_PACKAGE_DATA_OFFSET) > binLength) {
            return STATUS_INVALID_BUFFER_SIZE;
        }

        if (pkgIndex + 1U == pkgNum) {
            nextOffset = binLength;
        } else {
            ULONG nextTable = GOODIX_CFG_BIN_HEAD_LEN + ((pkgIndex + 1U) * 2U);
            if ((nextTable + 1U) >= binLength) {
                return STATUS_INVALID_BUFFER_SIZE;
            }
            nextOffset = GoodixReadU16Le(binData + nextTable);
        }

        if (nextOffset <= pkgOffset || nextOffset > binLength) {
            return STATUS_INVALID_BUFFER_SIZE;
        }

        pkgLength = nextOffset - pkgOffset;
        packageSensorId = binData[pkgOffset + 20U];

        if (packageSensorId == SensorId) {
            selectedOffset = pkgOffset;
            break;
        }

        if (packageSensorId == GOODIX_CFG_PACKAGE_SENSOR_ANY && firstWildcardOffset == 0) {
            firstWildcardOffset = pkgOffset;
        }
    }

    if (selectedOffset == 0) {
        selectedOffset = firstWildcardOffset;
    }

    if (selectedOffset == 0) {
        return STATUS_NOT_FOUND;
    }

    {
        ULONG pkgIndexLocal;
        ULONG nextOffset;
        ULONG pkgLength;
        UINT8 cfgType;

        pkgIndexLocal = 0;
        while (pkgIndexLocal < pkgNum) {
            ULONG offsetTable = GOODIX_CFG_BIN_HEAD_LEN + (pkgIndexLocal * 2U);
            ULONG pkgOffset = GoodixReadU16Le(binData + offsetTable);
            if (pkgOffset == selectedOffset) {
                break;
            }
            pkgIndexLocal++;
        }

        if (pkgIndexLocal + 1U == pkgNum) {
            nextOffset = binLength;
        } else {
            nextOffset = GoodixReadU16Le(binData + GOODIX_CFG_BIN_HEAD_LEN + ((pkgIndexLocal + 1U) * 2U));
        }

        pkgLength = nextOffset - selectedOffset;
        cfgType = binData[selectedOffset + 19U];
        if (pkgLength <= GOODIX_CFG_PACKAGE_DATA_OFFSET) {
            return STATUS_INVALID_BUFFER_SIZE;
        }

        Package->SensorId = binData[selectedOffset + 20U];
        Package->ConfigType = cfgType;
        Package->ConfigData = binData + selectedOffset + GOODIX_CFG_PACKAGE_DATA_OFFSET;
        Package->ConfigLength = pkgLength - GOODIX_CFG_PACKAGE_DATA_OFFSET;
    }

    return STATUS_SUCCESS;
}


typedef struct
{
    BYTE  reportId;                                 // Report ID = 0x54 (84) 'T'
                                                       // Collection: TouchScreen
    BYTE  DIG_TouchScreenContactCountMaximum;       // Usage 0x000D0055: Contact Count Maximum, Value = 0 to 8
} featureReport54_t;

typedef struct __declspec(align(2))
{
    BYTE  DIG_TouchScreenFingerState;               // Usage 0x000D0042: Tip Switch, Value = 0 to 1
    BYTE  DIG_TouchScreenFingerContactIdentifier;   // Usage 0x000D0051: Contact Identifier, Value = 0 to 1
    BYTE GD_TouchScreenFingerXL;                    // Usage 0x00010030: X, Value = 0 to 32767
    BYTE GD_TouchScreenFingerXH;                    // Usage 0x00010030: X, Value = 0 to 32767
    BYTE GD_TouchScreenFingerYL;                    // Usage 0x00010031: Y, Value = 0 to 32767
    BYTE GD_TouchScreenFingerYH;                    // Usage 0x00010031: Y, Value = 0 to 32767
}inputpoint;

typedef struct __declspec(align(2))
{
    BYTE  reportId;                                 // Report ID = 0x54 (84) 'T'
                                                       // Collection: TouchScreen Finger
    BYTE points[60];

    BYTE  DIG_TouchScreenContactCount;              // Usage 0x000D0054: Contact Count, Value = 0 to 8
} inputReport54_t;

//
// This is the default report descriptor for the virtual Hid device returned
// by the mini driver in response to IOCTL_HID_GET_REPORT_DESCRIPTOR.
//
/*HID_REPORT_DESCRIPTOR       G_DefaultReportDescriptor[] = {
    0x06,0x00, 0xFF,                // USAGE_PAGE (Vender Defined Usage Page)
    0x09,0x01,                      // USAGE (Vendor Usage 0x01)
    0xA1,0x01,                      // COLLECTION (Application)
    0x85,CONTROL_FEATURE_REPORT_ID,    // REPORT_ID (1)
    0x09,0x01,                         // USAGE (Vendor Usage 0x01)
    0x15,0x00,                         // LOGICAL_MINIMUM(0)
    0x26,0xff, 0x00,                   // LOGICAL_MAXIMUM(255)
    0x75,0x08,                         // REPORT_SIZE (0x08)
    0x96,(FEATURE_REPORT_SIZE_CB & 0xff), (FEATURE_REPORT_SIZE_CB >> 8), // REPORT_COUNT
    0xB1,0x00,                         // FEATURE (Data,Ary,Abs)
    0x09,0x01,                         // USAGE (Vendor Usage 0x01)
    0x75,0x08,                         // REPORT_SIZE (0x08)
    0x96,(INPUT_REPORT_SIZE_CB & 0xff), (INPUT_REPORT_SIZE_CB >> 8), // REPORT_COUNT
    0x81,0x00,                         // INPUT (Data,Ary,Abs)
    0x09,0x01,                         // USAGE (Vendor Usage 0x01)
    0x75,0x08,                         // REPORT_SIZE (0x08)
    0x96,(OUTPUT_REPORT_SIZE_CB & 0xff), (OUTPUT_REPORT_SIZE_CB >> 8), // REPORT_COUNT
    0x91,0x00,                         // OUTPUT (Data,Ary,Abs)
    0xC0,                           // END_COLLECTION
};*/

HID_REPORT_DESCRIPTOR       G_DefaultReportDescriptor[] = {
    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x04,     //   (LOCAL)USAGE              0x000D0004 Touch Screen(Application Collection)
    0xA1, 0x01,     //   (MAIN)COLLECTION         0x01 Application(Usage = 0x000D0004: Page = Digitizer Device Page, Usage = Touch Screen, Type = Application Collection)
    0x85, 0x54,     //     (GLOBAL)REPORT_ID          0x54 (84) 'T'

    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x14,           //    (GLOBAL)LOGICAL_MINIMUM(0)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //46 47
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //55 56
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical
    
    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical
    
    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x22,     //     (LOCAL)USAGE              0x000D0022 Finger(Logical Collection)
    0xA1, 0x02,     //     (MAIN)COLLECTION         0x02 Logical(Usage = 0x000D0022: Page = Digitizer Device Page, Usage = Finger, Type = Logical Collection)
    0x09, 0x42,     //       (LOCAL)USAGE              0x000D0042 Tip Switch(Momentary Control)
    0x25, 0x01,     //       (GLOBAL)LOGICAL_MAXIMUM    0x01 (1)
    0x75, 0x01,     //       (GLOBAL)REPORT_SIZE        0x01 (1) Number of bits per field
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x32,     //       (LOCAL)USAGE              0x000D0032 In Range(Momentary Control)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x47,     //       (LOCAL)USAGE              0x000D0047 Confidence(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 1 bit) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x95, 0x05,     //       (GLOBAL)REPORT_COUNT       0x05 (5) Number of fields
    0x81, 0x03,     //       (MAIN)INPUT              0x00000003 (5 fields x 1 bit) 1 = Constant 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x75, 0x08,     //       (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x95, 0x01,     //       (GLOBAL)REPORT_COUNT       0x01 (1) Number of fields
    0x09, 0x51,     //       (LOCAL)USAGE              0x000D0051 Contact Identifier(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x05, 0x01,     //       (GLOBAL)USAGE_PAGE         0x0001 Generic Desktop Page
    0x26, 0x38, 0x04,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (1080)    //99 100
    0x75, 0x10,     //       (GLOBAL)REPORT_SIZE        0x10 (16) Number of bits per field
    0x09, 0x30,     //       (LOCAL)USAGE              0x00010030 X(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x26, 0x24, 0x09,   // (GLOBAL) LOGICAL_MAXIMUM    0x7FFF (2250)    //108 109
    0x09, 0x31,     //       (LOCAL)USAGE              0x00010031 Y(Dynamic Value)
    0x81, 0x02,     //       (MAIN)INPUT              0x00000002 (1 field x 16 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Logical

    0x05, 0x0D,     // (GLOBAL) USAGE_PAGE         0x000D Digitizer Device Page
    0x09, 0x54,     //     (LOCAL)USAGE              0x000D0054 Contact Count(Dynamic Value)
    0x75, 0x08,     //     (GLOBAL)REPORT_SIZE        0x08 (8) Number of bits per field
    0x25, 0x0A,     //     (GLOBAL)LOGICAL_MAXIMUM    0x08 (8)
    0x81, 0x02,     //     (MAIN)INPUT              0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0x09, 0x55,     //     (LOCAL)USAGE              0x000D0055 Contact Count Maximum(Static Value)
    0xB1, 0x02,     //     (MAIN)FEATURE            0x00000002 (1 field x 8 bits) 0 = Data 1 = Variable 0 = Absolute 0 = NoWrap 0 = Linear 0 = PrefState 0 = NoNull 0 = NonVolatile 0 = Bitmap
    0xC0,           // (MAIN)   END_COLLECTION     Application

};

featureReport54_t features = {0x54,10};
//
// This is the default HID descriptor returned by the mini driver
// in response to IOCTL_HID_GET_DEVICE_DESCRIPTOR. The size
// of report descriptor is currently the size of G_DefaultReportDescriptor.
//

HID_DESCRIPTOR              G_DefaultHidDescriptor = {
    0x09,   // length of HID descriptor
    0x21,   // descriptor type == HID  0x21
    0x0100, // hid spec release
    0x00,   // country code == Not Specified
    0x01,   // number of HID class descriptors
    {                                       //DescriptorList[0]
        0x22,                               //report descriptor type 0x22
        sizeof(G_DefaultReportDescriptor)   //total length of report descriptor
    }
};

NTSTATUS
DriverEntry(
    _In_  PDRIVER_OBJECT    DriverObject,
    _In_  PUNICODE_STRING   RegistryPath
    )
/*++

Routine Description:
    DriverEntry initializes the driver and is the first routine called by the
    system after the driver is loaded. DriverEntry specifies the other entry
    points in the function driver, such as EvtDevice and DriverUnload.

Parameters Description:

    DriverObject - represents the instance of the function driver that is loaded
    into memory. DriverEntry must initialize members of DriverObject before it
    returns to the caller. DriverObject is allocated by the system before the
    driver is loaded, and it is released by the system after the system unloads
    the function driver from memory.

    RegistryPath - represents the driver specific path in the Registry.
    The function driver can use the path to store driver related data between
    reboots. The path does not store hardware instance specific data.

Return Value:

    STATUS_SUCCESS, or another status value for which NT_SUCCESS(status) equals
                    TRUE if successful,

    STATUS_UNSUCCESSFUL, or another status for which NT_SUCCESS(status) equals
                    FALSE otherwise.

--*/
{
    WDF_DRIVER_CONFIG       config;
    WDF_OBJECT_ATTRIBUTES driverAttributes;
    NTSTATUS                status;
    WDFDRIVER               driver;
    WPP_INIT_TRACING(DriverObject, RegistryPath);
    TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "George Droid on command. :D");
#ifdef _KERNEL_MODE
    //
    // Opt-in to using non-executable pool memory on Windows 8 and later.
    // https://msdn.microsoft.com/en-us/library/windows/hardware/hh920402(v=vs.85).aspx
    //
    ExInitializeDriverRuntime(DrvRtPoolNxOptIn);
    TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Kernel Mode.");
#endif

    WDF_DRIVER_CONFIG_INIT(&config, EvtDeviceAdd);

    TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Driver Init.");

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(&driverAttributes, DRIVER_CONTEXT);
    TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Attributes Init.");
    driverAttributes.EvtCleanupCallback = EvtDriverCleanup;

    status = WdfDriverCreate(DriverObject,
                            RegistryPath,
                            &driverAttributes,
                            &config,
                            &driver);
    if (!NT_SUCCESS(status)) {

        goto Exit;
    }

    {
        PDRIVER_CONTEXT driverContext;
        WDF_OBJECT_ATTRIBUTES lockAttributes;

        driverContext = GetDriverContext(driver);
        RtlZeroMemory(driverContext, sizeof(*driverContext));

        WDF_OBJECT_ATTRIBUTES_INIT(&lockAttributes);
        lockAttributes.ParentObject = driver;
        status = WdfWaitLockCreate(&lockAttributes, &driverContext->ControlLock);
        if (!NT_SUCCESS(status)) {
            goto Exit;
        }

        status = GoodixCreateControlDevice(driver);
        if (!NT_SUCCESS(status)) {
            goto Exit;
        }
    }
Exit:
    return status;
}
VOID
EvtDriverCleanup(
    _In_ WDFOBJECT Object
)
{
    WPP_CLEANUP(WdfDriverWdmGetDriverObject((WDFDRIVER)Object));
}

static
NTSTATUS
GoodixCreateControlDevice(
    _In_ WDFDRIVER Driver
    )
{
    NTSTATUS status;
    PWDFDEVICE_INIT controlInit;
    WDFDEVICE controlDevice;
    WDF_IO_QUEUE_CONFIG queueConfig;
    WDF_OBJECT_ATTRIBUTES deviceAttributes;
    UNICODE_STRING deviceName;
    UNICODE_STRING symbolicLinkName;
    UNICODE_STRING sddl;
    PDRIVER_CONTEXT driverContext;

    RtlInitUnicodeString(&sddl, L"D:P(A;;GA;;;SY)(A;;GA;;;BA)(A;;GRGW;;;BU)");
    controlInit = WdfControlDeviceInitAllocate(
        Driver,
        &sddl);
    if (controlInit == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    RtlInitUnicodeString(&deviceName, GOODIX_TOUCH_CONTROL_NT_DEVICE_NAME);
    status = WdfDeviceInitAssignName(controlInit, &deviceName);
    if (!NT_SUCCESS(status)) {
        WdfDeviceInitFree(controlInit);
        return status;
    }

    WdfDeviceInitSetDeviceType(controlInit, FILE_DEVICE_UNKNOWN);
    WdfDeviceInitSetExclusive(controlInit, FALSE);

    WDF_OBJECT_ATTRIBUTES_INIT(&deviceAttributes);
    status = WdfDeviceCreate(&controlInit, &deviceAttributes, &controlDevice);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(&queueConfig, WdfIoQueueDispatchSequential);
    queueConfig.EvtIoDeviceControl = GoodixControlEvtIoDeviceControl;

    status = WdfIoQueueCreate(
        controlDevice,
        &queueConfig,
        WDF_NO_OBJECT_ATTRIBUTES,
        &GetDriverContext(Driver)->ControlQueue);
    if (!NT_SUCCESS(status)) {
        WdfObjectDelete(controlDevice);
        return status;
    }

    RtlInitUnicodeString(&symbolicLinkName, GOODIX_TOUCH_CONTROL_DOS_DEVICE_NAME);
    status = WdfDeviceCreateSymbolicLink(controlDevice, &symbolicLinkName);
    if (!NT_SUCCESS(status)) {
        WdfObjectDelete(controlDevice);
        return status;
    }

    driverContext = GetDriverContext(Driver);
    driverContext->ControlDevice = controlDevice;
    WdfControlFinishInitializing(controlDevice);
    return STATUS_SUCCESS;
}

static
VOID
GoodixSetActiveTouchDevice(
    _In_ WDFDRIVER Driver,
    _In_opt_ WDFDEVICE Device
    )
{
    PDRIVER_CONTEXT driverContext;

    driverContext = GetDriverContext(Driver);
    WdfWaitLockAcquire(driverContext->ControlLock, NULL);
    driverContext->ActiveTouchDevice = Device;
    WdfWaitLockRelease(driverContext->ControlLock);
}

static
PDEVICE_CONTEXT
GoodixAcquireActiveDeviceContext(
    _In_ WDFDRIVER Driver,
    _Out_opt_ WDFDEVICE* ReferencedDevice
    )
{
    PDRIVER_CONTEXT driverContext;
    WDFDEVICE device;

    if (ReferencedDevice != NULL) {
        *ReferencedDevice = NULL;
    }

    driverContext = GetDriverContext(Driver);
    WdfWaitLockAcquire(driverContext->ControlLock, NULL);
    device = driverContext->ActiveTouchDevice;
    if (device != NULL) {
        WdfObjectReference(device);
    }
    WdfWaitLockRelease(driverContext->ControlLock);

    if (ReferencedDevice != NULL) {
        *ReferencedDevice = device;
    }

    if (device == NULL) {
        return NULL;
    }

    return GetDeviceContext(device);
}

static
VOID
GoodixReleaseActiveDevice(
    _In_opt_ WDFDEVICE Device
    )
{
    if (Device != NULL) {
        WdfObjectDereference(Device);
    }
}

VOID
GoodixControlEvtIoDeviceControl(
    _In_ WDFQUEUE Queue,
    _In_ WDFREQUEST Request,
    _In_ size_t OutputBufferLength,
    _In_ size_t InputBufferLength,
    _In_ ULONG IoControlCode
    )
{
    WDFDEVICE controlDevice;
    WDFDRIVER driver;
    WDFDEVICE activeDevice;
    PDEVICE_CONTEXT deviceContext;
    NTSTATUS status;

    UNREFERENCED_PARAMETER(OutputBufferLength);
    UNREFERENCED_PARAMETER(InputBufferLength);

    controlDevice = WdfIoQueueGetDevice(Queue);
    driver = WdfDeviceGetDriver(controlDevice);
    activeDevice = NULL;
    deviceContext = GoodixAcquireActiveDeviceContext(driver, &activeDevice);
    if (deviceContext == NULL) {
        WdfRequestComplete(Request, STATUS_DEVICE_NOT_READY);
        return;
    }

    status = GoodixProcessControlRequest(deviceContext, Request, IoControlCode);
    GoodixReleaseActiveDevice(activeDevice);
    WdfRequestComplete(Request, status);
}

NTSTATUS
EvtDeviceAdd(
    _In_  WDFDRIVER         Driver,
    _Inout_ PWDFDEVICE_INIT DeviceInit
    )
/*++
Routine Description:

    EvtDeviceAdd is called by the framework in response to AddDevice
    call from the PnP manager. We create and initialize a device object to
    represent a new instance of the device.

Arguments:

    Driver - Handle to a framework driver object created in DriverEntry

    DeviceInit - Pointer to a framework-allocated WDFDEVICE_INIT structure.

Return Value:

    NTSTATUS

--*/
{
    NTSTATUS                status;
    WDF_OBJECT_ATTRIBUTES   deviceAttributes;
    WDFDEVICE               device;
    PDEVICE_CONTEXT         deviceContext;
    PHID_DEVICE_ATTRIBUTES  hidAttributes;
    UNREFERENCED_PARAMETER  (Driver);

    WDF_PNPPOWER_EVENT_CALLBACKS pnpCallbacks;
    WDF_PNPPOWER_EVENT_CALLBACKS_INIT(&pnpCallbacks);

    pnpCallbacks.EvtDevicePrepareHardware = OnPrepareHardware;
    pnpCallbacks.EvtDeviceReleaseHardware = OnReleaseHardware;
    pnpCallbacks.EvtDeviceD0Entry = OnD0Entry;
    pnpCallbacks.EvtDeviceD0Exit = OnD0Exit;


    //
    // Mark ourselves as a filter, which also relinquishes power policy ownership
    //
    WdfFdoInitSetFilter(DeviceInit);

    WdfDeviceInitSetPnpPowerEventCallbacks(DeviceInit, &pnpCallbacks);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
                            &deviceAttributes,
                            DEVICE_CONTEXT);

    status = WdfDeviceCreate(&DeviceInit,
                            &deviceAttributes,
                            &device);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = WdfDeviceCreateDeviceInterface(
        device,
        &GUID_DEVINTERFACE_GOODIX_TOUCH_CONTROL,
        NULL);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    deviceContext = GetDeviceContext(device);
    deviceContext->Device       = device;
    deviceContext->DeviceData = 0;
    deviceContext->OnClose = FALSE;
    deviceContext->ResetGpioId.QuadPart = 0;
    deviceContext->ResetGpioPresent = FALSE;
    deviceContext->LastTouchID = 0;
    deviceContext->LastLoggedTouchCount = 0xFF;
    deviceContext->ReportRateLevel = GOODIX_REPORT_RATE_240HZ;
    deviceContext->ActiveReportRateLevel = 0xFF;
    deviceContext->SensorId = 0;
    deviceContext->VersionValid = FALSE;
    deviceContext->TouchDataAddress = TOUCH_INFO_ADDR;
    deviceContext->CommandAddress = CMD_ADDR;
    deviceContext->FwBufferAddress = 0;
    deviceContext->FwBufferMaxLength = 0;
    deviceContext->IcInfoValid = FALSE;
    deviceContext->ConfigApplied = FALSE;
    deviceContext->FirmwareUpdated = FALSE;
    deviceContext->SpbController = WDF_NO_HANDLE;
    deviceContext->ResetGpioIoTarget = WDF_NO_HANDLE;
    deviceContext->ControllerRequestLock = WDF_NO_HANDLE;
    deviceContext->ControllerRequestWorkItem = WDF_NO_HANDLE;
    deviceContext->ControllerRequestWorkItemQueued = 0;
    deviceContext->ConsecutiveTransportFailures = 0;
    deviceContext->PendingControllerRequestCode = 0;

    status = WdfWaitLockCreate(WDF_NO_OBJECT_ATTRIBUTES, &deviceContext->IoLock);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = WdfWaitLockCreate(WDF_NO_OBJECT_ATTRIBUTES, &deviceContext->ControllerRequestLock);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    {
        WDF_OBJECT_ATTRIBUTES workItemAttributes;
        WDF_WORKITEM_CONFIG workItemConfig;

        WDF_OBJECT_ATTRIBUTES_INIT(&workItemAttributes);
        workItemAttributes.ParentObject = device;

        WDF_WORKITEM_CONFIG_INIT(
            &workItemConfig,
            GoodixControllerRequestWorkItem);

        status = WdfWorkItemCreate(
            &workItemConfig,
            &workItemAttributes,
            &deviceContext->ControllerRequestWorkItem);
        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    hidAttributes = &deviceContext->HidDeviceAttributes;
    RtlZeroMemory(hidAttributes, sizeof(HID_DEVICE_ATTRIBUTES));
    hidAttributes->Size         = sizeof(HID_DEVICE_ATTRIBUTES);
    hidAttributes->VendorID     = HIDMINI_VID;
    hidAttributes->ProductID    = HIDMINI_PID;
    hidAttributes->VersionNumber = HIDMINI_VERSION;

    status = QueueCreate(device,
                         &deviceContext->DefaultQueue);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    status = ManualQueueCreate(device,
                               &deviceContext->ManualQueue);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    //
    // Use default "HID Descriptor" (hardcoded). We will set the
    // wReportLength memeber of HID descriptor when we read the
    // the report descriptor either from registry or the hard-coded
    // one.
    //
    deviceContext->HidDescriptor = G_DefaultHidDescriptor;

    //
    // We need to read read descriptor from registry
    //
    status = ReadDescriptorFromRegistry(device);
    if (!NT_SUCCESS(status)) {
    }

    G_DefaultReportDescriptor[46] = XMax&0xFF;
    G_DefaultReportDescriptor[47] = (XMax>>8)&0x0F;
    G_DefaultReportDescriptor[55] = YMax & 0xFF;
    G_DefaultReportDescriptor[56] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 1] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 1] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 1] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 1] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 2] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 2] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 2] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 2] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 3] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 3] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 3] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 3] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 4] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 4] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 4] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 4] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 5] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 5] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 5] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 5] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 6] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 6] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 6] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 6] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 7] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 7] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 7] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 7] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 8] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 8] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 8] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 8] = (YMax >> 8) & 0x0F;

    G_DefaultReportDescriptor[46 + 53 * 9] = XMax & 0xFF;
    G_DefaultReportDescriptor[47 + 53 * 9] = (XMax >> 8) & 0x0F;
    G_DefaultReportDescriptor[55 + 53 * 9] = YMax & 0xFF;
    G_DefaultReportDescriptor[56 + 53 * 9] = (YMax >> 8) & 0x0F;

    deviceContext->ReportDescriptor = G_DefaultReportDescriptor;
    status = STATUS_SUCCESS;

    return status;
}

NTSTATUS
    OnPrepareHardware(
        _In_  WDFDEVICE     FxDevice,
        _In_  WDFCMRESLIST  FxResourcesRaw,
        _In_  WDFCMRESLIST  FxResourcesTranslated
    )
    /*++

        Routine Description:

        This routine caches the SPB resource connection ID.

        Arguments:

        FxDevice - a handle to the framework device object
        FxResourcesRaw - list of translated hardware resources that
            the PnP manager has assigned to the device
        FxResourcesTranslated - list of raw hardware resources that
            the PnP manager has assigned to the device

        Return Value:

        Status

    --*/
{
    PDEVICE_CONTEXT pDevice = GetDeviceContext(FxDevice);
    BOOLEAN fSpbResourceFound = FALSE;
    BOOLEAN fInterruptResourceFound = FALSE;
    BOOLEAN fResetGpioFound = FALSE;
    ULONG interruptIndex = 0;
    NTSTATUS status = STATUS_SUCCESS;

    UNREFERENCED_PARAMETER(FxResourcesRaw);

    //
    // Parse the peripheral's resources.
    //

    ULONG resourceCount = WdfCmResourceListGetCount(FxResourcesTranslated);

    for (ULONG i = 0; i < resourceCount; i++)
    {
        PCM_PARTIAL_RESOURCE_DESCRIPTOR pDescriptor;
        UCHAR Class;
        UCHAR Type;

        pDescriptor = WdfCmResourceListGetDescriptor(
            FxResourcesTranslated, i);

        switch (pDescriptor->Type)
        {
        case CmResourceTypeConnection:

            //
            // Look for I2C or SPI resource and save connection ID.
            //

            Class = pDescriptor->u.Connection.Class;
            Type = pDescriptor->u.Connection.Type;

            if ((Class == CM_RESOURCE_CONNECTION_CLASS_SERIAL) &&
                ((Type == CM_RESOURCE_CONNECTION_TYPE_SERIAL_SPI)))
            {
                if (fSpbResourceFound == FALSE)
                {
                    pDevice->PeripheralId.LowPart =
                        pDescriptor->u.Connection.IdLowPart;
                    pDevice->PeripheralId.HighPart =
                        pDescriptor->u.Connection.IdHighPart;

                    fSpbResourceFound = TRUE;
                }
            }
            else if ((Class == CM_RESOURCE_CONNECTION_CLASS_GPIO) &&
                (Type == CM_RESOURCE_CONNECTION_TYPE_GPIO_IO))
            {
                if (fResetGpioFound == FALSE)
                {
                    pDevice->ResetGpioId.LowPart =
                        pDescriptor->u.Connection.IdLowPart;
                    pDevice->ResetGpioId.HighPart =
                        pDescriptor->u.Connection.IdHighPart;
                    pDevice->ResetGpioPresent = TRUE;
                    fResetGpioFound = TRUE;
                }
            }

            break;

        case CmResourceTypeInterrupt:

            if (fInterruptResourceFound == FALSE)
            {
                fInterruptResourceFound = TRUE;
                interruptIndex = i;
            }
            break;

        default:

            //
            // Ignoring all other resource types.
            //

            break;
        }
    }

    //
    // An SPB resource is required.
    //

    if (fSpbResourceFound == FALSE)
    {
        status = STATUS_NOT_FOUND;
    }

    //
    // Create the interrupt if an interrupt
    // resource was found.
    //

    if (NT_SUCCESS(status))
    {
        if (fInterruptResourceFound == TRUE)
        {
            WDF_INTERRUPT_CONFIG interruptConfig;
            
            WDF_INTERRUPT_CONFIG_INIT(
                &interruptConfig,
                OnInterruptIsr,
                NULL);
            interruptConfig.ReportInactiveOnPowerDown = TRUE;
            interruptConfig.PassiveHandling = TRUE;
            interruptConfig.InterruptTranslated = WdfCmResourceListGetDescriptor(
                FxResourcesTranslated,
                interruptIndex);
            interruptConfig.InterruptRaw = WdfCmResourceListGetDescriptor(
                FxResourcesRaw,
                interruptIndex);

            status = WdfInterruptCreate(
                pDevice->Device,
                &interruptConfig,
                WDF_NO_OBJECT_ATTRIBUTES,
                &pDevice->Interrupt);

            if (!NT_SUCCESS(status))
            {
            }

            if (NT_SUCCESS(status))
            {
                WdfInterruptDisable(pDevice->Interrupt);
            }
        }
    }

    return status;
}

NTSTATUS
    OnReleaseHardware(
        _In_  WDFDEVICE     FxDevice,
        _In_  WDFCMRESLIST  FxResourcesTranslated
    )
    /*++

        Routine Description:

        Arguments:

        FxDevice - a handle to the framework device object
        FxResourcesTranslated - list of raw hardware resources that
            the PnP manager has assigned to the device

        Return Value:

        Status

    --*/
{
    PDEVICE_CONTEXT pDevice = GetDeviceContext(FxDevice);
    UNREFERENCED_PARAMETER(FxResourcesTranslated);
    GoodixSetActiveTouchDevice(WdfDeviceGetDriver(FxDevice), NULL);
    GoodixCloseResetGpio(pDevice);
    if (pDevice->Interrupt != NULL)
    {
        WdfObjectDelete(pDevice->Interrupt);
    }
    return STATUS_SUCCESS;
}

NTSTATUS
OnD0Entry(
    _In_  WDFDEVICE               FxDevice,
    _In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

    Routine Description:

    This routine allocates objects needed by the driver.

    Arguments:

    FxDevice - a handle to the framework device object
    FxPreviousState - previous power state

    Return Value:

    Status

--*/
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    PDEVICE_CONTEXT pDevice = GetDeviceContext(FxDevice);
    NTSTATUS status;
    GOODIX_FW_VERSION version = { 0 };

    //
    // Create the SPB target.
    //

    WDF_OBJECT_ATTRIBUTES targetAttributes;
    WDF_OBJECT_ATTRIBUTES_INIT(&targetAttributes);

    status = WdfIoTargetCreate(
        pDevice->Device,
        &targetAttributes,
        &pDevice->SpbController);

    if (!NT_SUCCESS(status))
    {
    }

    status = SpbDeviceOpen(pDevice);
    if (!NT_SUCCESS(status)) {
        if (pDevice->SpbController != WDF_NO_HANDLE)
        {
            WdfObjectDelete(pDevice->SpbController);
            pDevice->SpbController = WDF_NO_HANDLE;
        }
        return status;
    }

    if (pDevice->ResetGpioPresent) {
        status = GoodixOpenResetGpio(pDevice);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "GoodixOpenResetGpio failed status=0x%08X", status);
            status = STATUS_SUCCESS;
        }
    }

    status = GoodixReadVersion(pDevice, &version);
    if (NT_SUCCESS(status)) {
        pDevice->SensorId = version.SensorId;
        pDevice->VersionValid = TRUE;
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Goodix version read ok sensor_id=%u pid=%c%c%c%c%c%c%c%c",
            version.SensorId,
            version.PatchPid[0], version.PatchPid[1], version.PatchPid[2], version.PatchPid[3],
            version.PatchPid[4], version.PatchPid[5], version.PatchPid[6], version.PatchPid[7]);
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE, "Embedded fw pid=%c%c%c%c%c vid=%02X%02X%02X%02X current_vid=%02X%02X%02X%02X",
            g_GoodixEmbeddedFwPid[0], g_GoodixEmbeddedFwPid[1], g_GoodixEmbeddedFwPid[2],
            g_GoodixEmbeddedFwPid[3], g_GoodixEmbeddedFwPid[4],
            g_GoodixEmbeddedFwVid[0], g_GoodixEmbeddedFwVid[1], g_GoodixEmbeddedFwVid[2], g_GoodixEmbeddedFwVid[3],
            version.PatchVid[0], version.PatchVid[1], version.PatchVid[2], version.PatchVid[3]);
    } else {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "GoodixReadVersion failed status=0x%08X", status);
    }

    if (pDevice->VersionValid) {
        status = GoodixMaybeUpdateFirmware(pDevice, &version);
        if (NT_SUCCESS(status)) {
            pDevice->SensorId = version.SensorId;
            pDevice->VersionValid = TRUE;
        } else {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "GoodixMaybeUpdateFirmware failed status=0x%08X", status);
            status = STATUS_SUCCESS;
        }
    }

    status = GoodixReadIcInfo(pDevice);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "GoodixReadIcInfo failed status=0x%08X, keeping defaults", status);
        status = STATUS_SUCCESS;
    }

    status = GoodixApplyEmbeddedConfig(pDevice);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "GoodixApplyEmbeddedConfig failed during init sensor_id=%u status=0x%08X",
            pDevice->SensorId, status);
        status = STATUS_SUCCESS;
    } else if (pDevice->ConfigApplied) {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Goodix embedded config applied during init sensor_id=%u",
            pDevice->SensorId);
    }

    status = GoodixApplyReportRate(pDevice, pDevice->ReportRateLevel);
    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
            "GoodixApplyReportRate failed during init level=%u status=0x%08X",
            pDevice->ReportRateLevel, status);
        status = STATUS_SUCCESS;
    } else {
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Goodix report rate applied during init level=%u",
            pDevice->ReportRateLevel);
    }

    if (NT_SUCCESS(status)) {
        GoodixSetActiveTouchDevice(WdfDeviceGetDriver(FxDevice), FxDevice);
    }

    return status;
}

static
VOID
GoodixHandleControllerRequest(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 RequestCode
    )
{
    NTSTATUS status;

    switch (RequestCode) {
    case 0x01:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Goodix request: config sensor_id=%u", DeviceContext->SensorId);
        status = GoodixApplyEmbeddedConfig(DeviceContext);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "GoodixApplyEmbeddedConfig failed from request sensor_id=%u status=0x%08X",
                DeviceContext->SensorId, status);
            return;
        }

        status = GoodixApplyReportRate(DeviceContext, DeviceContext->ReportRateLevel);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "GoodixApplyReportRate after config failed level=%u status=0x%08X",
                DeviceContext->ReportRateLevel, status);
        }
        break;

    case 0x03:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Goodix request: reset");
        status = GoodixResetDevice(DeviceContext, 30);
        if (!NT_SUCCESS(status)) {
            TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE,
                "GoodixResetDevice from request failed status=0x%08X",
                status);
        }
        break;

    default:
        TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
            "Goodix request: unsupported code=0x%02X", RequestCode);
        break;
    }
}

static
VOID
GoodixQueueControllerRequest(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 RequestCode
    )
{
    UINT8 requestToQueue = RequestCode;

    if (requestToQueue == 0) {
        return;
    }

    WdfWaitLockAcquire(DeviceContext->ControllerRequestLock, NULL);

    if (DeviceContext->PendingControllerRequestCode == GOODIX_DEFERRED_REQUEST_RECOVER
        || requestToQueue == GOODIX_DEFERRED_REQUEST_RECOVER) {
        DeviceContext->PendingControllerRequestCode = GOODIX_DEFERRED_REQUEST_RECOVER;
    } else if (DeviceContext->PendingControllerRequestCode == 0x01 || requestToQueue == 0x01) {
        DeviceContext->PendingControllerRequestCode = 0x01;
    } else if (DeviceContext->PendingControllerRequestCode == 0) {
        DeviceContext->PendingControllerRequestCode = requestToQueue;
    } else {
        DeviceContext->PendingControllerRequestCode = requestToQueue;
    }

    WdfWaitLockRelease(DeviceContext->ControllerRequestLock);

    if (InterlockedCompareExchange(&DeviceContext->ControllerRequestWorkItemQueued, 1, 0) == 0) {
        WdfWorkItemEnqueue(DeviceContext->ControllerRequestWorkItem);
    }
}

VOID
GoodixControllerRequestWorkItem(
    _In_ WDFWORKITEM WorkItem
    )
{
    WDFDEVICE device;
    PDEVICE_CONTEXT deviceContext;
    UINT8 requestCode;
    BOOLEAN interruptToggled = FALSE;

    device = (WDFDEVICE)WdfWorkItemGetParentObject(WorkItem);
    deviceContext = GetDeviceContext(device);

    for (;;) {
        requestCode = 0;

        WdfWaitLockAcquire(deviceContext->ControllerRequestLock, NULL);
        requestCode = deviceContext->PendingControllerRequestCode;
        deviceContext->PendingControllerRequestCode = 0;
        WdfWaitLockRelease(deviceContext->ControllerRequestLock);

        if (requestCode == 0) {
            break;
        }

        if ((requestCode != GOODIX_DEFERRED_REQUEST_RECOVER)
            && (deviceContext->Interrupt != NULL)
            && !interruptToggled) {
            WdfInterruptDisable(deviceContext->Interrupt);
            interruptToggled = TRUE;
        }

        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "Goodix deferred request handling code=0x%02X",
            requestCode);

        if (requestCode == GOODIX_DEFERRED_REQUEST_RECOVER) {
            NTSTATUS status;

            TraceEvents(
                TRACE_LEVEL_WARNING,
                TRACE_DEVICE,
                "Goodix deferred recovery begin failures=%lu",
                deviceContext->ConsecutiveTransportFailures);

            status = GoodixReinitializeAfterReportRateChange(deviceContext);
            if (NT_SUCCESS(status)) {
                deviceContext->ConsecutiveTransportFailures = 0;
                TraceEvents(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_DEVICE,
                    "Goodix deferred recovery completed");
            } else {
                TraceEvents(
                    TRACE_LEVEL_WARNING,
                    TRACE_DEVICE,
                    "Goodix deferred recovery failed status=0x%08X",
                    status);
            }
        } else {
            GoodixHandleControllerRequest(deviceContext, requestCode);
        }
    }

    if (interruptToggled) {
        WdfInterruptEnable(deviceContext->Interrupt);
    }

    InterlockedExchange(&deviceContext->ControllerRequestWorkItemQueued, 0);

    WdfWaitLockAcquire(deviceContext->ControllerRequestLock, NULL);
    requestCode = deviceContext->PendingControllerRequestCode;
    WdfWaitLockRelease(deviceContext->ControllerRequestLock);

    if (requestCode != 0 &&
        InterlockedCompareExchange(&deviceContext->ControllerRequestWorkItemQueued, 1, 0) == 0) {
        WdfWorkItemEnqueue(deviceContext->ControllerRequestWorkItem);
    }
}

NTSTATUS
GoodixReinitializeAfterReportRateChange(
    _In_ PDEVICE_CONTEXT DeviceContext
    )
{
    NTSTATUS status;
    GOODIX_FW_VERSION version = { 0 };
    BOOLEAN interruptToggled = FALSE;

    TraceEvents(
        TRACE_LEVEL_INFORMATION,
        TRACE_DEVICE,
        "Goodix runtime reinit begin requestedLevel=%u",
        DeviceContext->ReportRateLevel);

    if (DeviceContext->Interrupt != NULL) {
        WdfInterruptDisable(DeviceContext->Interrupt);
        interruptToggled = TRUE;
    }

    status = GoodixResetDevice(DeviceContext, 100);
    if (!NT_SUCCESS(status)) {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix runtime reinit reset failed status=0x%08X",
            status);
        goto Exit;
    }

    GoodixDelayMilliseconds(20);

    DeviceContext->VersionValid = FALSE;
    DeviceContext->IcInfoValid = FALSE;
    DeviceContext->ConfigApplied = FALSE;
    DeviceContext->ActiveReportRateLevel = 0xFF;

    status = GoodixReadVersion(DeviceContext, &version);
    if (NT_SUCCESS(status)) {
        DeviceContext->SensorId = version.SensorId;
        DeviceContext->VersionValid = TRUE;
        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "Goodix runtime reinit version sensor_id=%u pid=%c%c%c%c%c%c%c%c",
            version.SensorId,
            version.PatchPid[0], version.PatchPid[1], version.PatchPid[2], version.PatchPid[3],
            version.PatchPid[4], version.PatchPid[5], version.PatchPid[6], version.PatchPid[7]);
    } else {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix runtime reinit read version failed status=0x%08X",
            status);
        goto Exit;
    }

    status = GoodixReadIcInfo(DeviceContext);
    if (!NT_SUCCESS(status)) {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix runtime reinit ic_info failed status=0x%08X",
            status);
        goto Exit;
    }

    status = GoodixApplyEmbeddedConfig(DeviceContext);
    if (!NT_SUCCESS(status)) {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix runtime reinit config failed sensor_id=%u status=0x%08X",
            DeviceContext->SensorId,
            status);
        goto Exit;
    }

    status = GoodixApplyReportRate(DeviceContext, DeviceContext->ReportRateLevel);
    if (!NT_SUCCESS(status)) {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix runtime reinit report rate failed level=%u status=0x%08X",
            DeviceContext->ReportRateLevel,
            status);
        goto Exit;
    }

    TraceEvents(
        TRACE_LEVEL_INFORMATION,
        TRACE_DEVICE,
        "Goodix runtime reinit complete level=%u active=%u",
        DeviceContext->ReportRateLevel,
        DeviceContext->ActiveReportRateLevel);

Exit:
    if (interruptToggled) {
        WdfInterruptEnable(DeviceContext->Interrupt);
    }

    return status;
}

NTSTATUS
GoodixProcessControlRequest(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ WDFREQUEST Request,
    _In_ ULONG IoControlCode
    )
{
    NTSTATUS status;

    switch (IoControlCode)
    {
    case IOCTL_GOODIX_TOUCH_GET_REPORT_RATE:
    {
        PGOODIX_TOUCH_REPORT_RATE_STATE rateState = NULL;
        size_t bufferLength = 0;

        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "Goodix control get report rate requested persistent=%u active=%u",
            DeviceContext->ReportRateLevel,
            (DeviceContext->ActiveReportRateLevel == 0xFF)
                ? DeviceContext->ReportRateLevel
                : DeviceContext->ActiveReportRateLevel);

        status = WdfRequestRetrieveOutputBuffer(
            Request,
            sizeof(*rateState),
            (PVOID*)&rateState,
            &bufferLength);
        if (!NT_SUCCESS(status)) {
            return status;
        }

        UNREFERENCED_PARAMETER(bufferLength);
        rateState->PersistentLevel = DeviceContext->ReportRateLevel;
        rateState->ActiveLevel =
            (DeviceContext->ActiveReportRateLevel == 0xFF)
            ? DeviceContext->ReportRateLevel
            : DeviceContext->ActiveReportRateLevel;
        WdfRequestSetInformation(Request, sizeof(*rateState));
        return STATUS_SUCCESS;
    }

    case IOCTL_GOODIX_TOUCH_SET_REPORT_RATE:
    {
        PGOODIX_REPORT_RATE_CONTROL reportRateControl = NULL;
        size_t bufferLength = 0;
        UINT8 requestedLevel;
        UINT8 previousPersistentLevel;

        status = WdfRequestRetrieveInputBuffer(
            Request,
            sizeof(*reportRateControl),
            (PVOID*)&reportRateControl,
            &bufferLength);
        if (!NT_SUCCESS(status)) {
            return status;
        }

        requestedLevel = GoodixNormalizeReportRateLevel((UINT8)reportRateControl->Level);
        UNREFERENCED_PARAMETER(bufferLength);

        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "Goodix control set report rate requested level=%u currentPersistent=%u currentActive=%u",
            requestedLevel,
            DeviceContext->ReportRateLevel,
            (DeviceContext->ActiveReportRateLevel == 0xFF)
                ? 0xFF
                : DeviceContext->ActiveReportRateLevel);

        previousPersistentLevel = DeviceContext->ReportRateLevel;
        DeviceContext->ReportRateLevel = requestedLevel;
        status = GoodixPersistReportRateLevel(DeviceContext, requestedLevel);
        if (!NT_SUCCESS(status)) {
            TraceEvents(
                TRACE_LEVEL_WARNING,
                TRACE_DEVICE,
                "Goodix control set report rate persist failed level=%u status=0x%08X",
                requestedLevel,
                status);
            DeviceContext->ReportRateLevel = previousPersistentLevel;
            return status;
        }

        status = GoodixReinitializeAfterReportRateChange(DeviceContext);
        if (!NT_SUCCESS(status)) {
            DeviceContext->ReportRateLevel = previousPersistentLevel;
            (void)GoodixPersistReportRateLevel(DeviceContext, previousPersistentLevel);
            TraceEvents(
                TRACE_LEVEL_WARNING,
                TRACE_DEVICE,
                "Goodix control set report rate reinit failed level=%u status=0x%08X",
                requestedLevel,
                status);
            return status;
        }

        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "Goodix control set report rate applied level=%u active=%u",
            requestedLevel,
            DeviceContext->ActiveReportRateLevel);

        WdfRequestSetInformation(Request, sizeof(*reportRateControl));
        return STATUS_SUCCESS;
    }

    default:
        return STATUS_INVALID_DEVICE_REQUEST;
    }
}

NTSTATUS
OnD0Exit(
    _In_  WDFDEVICE               FxDevice,
    _In_  WDF_POWER_DEVICE_STATE  FxPreviousState
)
/*++

    Routine Description:

    This routine destroys objects needed by the driver.

    Arguments:

    FxDevice - a handle to the framework device object
    FxPreviousState - previous power state

    Return Value:

    Status

--*/
{
    UNREFERENCED_PARAMETER(FxPreviousState);

    PDEVICE_CONTEXT pDevice = GetDeviceContext(FxDevice);
    GoodixSetActiveTouchDevice(WdfDeviceGetDriver(FxDevice), NULL);
    GoodixCloseResetGpio(pDevice);
    SpbDeviceClose(pDevice);
    if (pDevice->SpbController != WDF_NO_HANDLE)
    {
        WdfObjectDelete(pDevice->SpbController);
        pDevice->SpbController = WDF_NO_HANDLE;
    }

    return STATUS_SUCCESS;
}


#ifdef _KERNEL_MODE
EVT_WDF_IO_QUEUE_IO_INTERNAL_DEVICE_CONTROL EvtIoDeviceControl;
#else
EVT_WDF_IO_QUEUE_IO_DEVICE_CONTROL          EvtIoDeviceControl;
#endif

NTSTATUS
QueueCreate(
    _In_  WDFDEVICE         Device,
    _Out_ WDFQUEUE          *Queue
    )
/*++
Routine Description:

    This function creates a default, parallel I/O queue to proces IOCTLs
    from hidclass.sys.

Arguments:

    Device - Handle to a framework device object.

    Queue - Output pointer to a framework I/O queue handle, on success.

Return Value:

    NTSTATUS

--*/
{
    NTSTATUS                status;
    WDF_IO_QUEUE_CONFIG     queueConfig;
    WDF_OBJECT_ATTRIBUTES   queueAttributes;
    WDFQUEUE                queue;
    PQUEUE_CONTEXT          queueContext;

    WDF_IO_QUEUE_CONFIG_INIT_DEFAULT_QUEUE(
                            &queueConfig,
                            WdfIoQueueDispatchParallel);

#ifdef _KERNEL_MODE
    queueConfig.EvtIoInternalDeviceControl  = EvtIoDeviceControl;
    queueConfig.EvtIoDeviceControl          = EvtIoDeviceControl;
#else
    //
    // HIDclass uses INTERNAL_IOCTL which is not supported by UMDF. Therefore
    // the hidumdf.sys changes the IOCTL type to DEVICE_CONTROL for next stack
    // and sends it down
    //
    queueConfig.EvtIoDeviceControl          = EvtIoDeviceControl;
#endif

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
                            &queueAttributes,
                            QUEUE_CONTEXT);

    status = WdfIoQueueCreate(
                            Device,
                            &queueConfig,
                            &queueAttributes,
                            &queue);

    if( !NT_SUCCESS(status) ) {
        return status;
    }

    queueContext = GetQueueContext(queue);
    queueContext->Queue         = queue;
    queueContext->DeviceContext = GetDeviceContext(Device);
    queueContext->OutputReport  = 0;

    *Queue = queue;
    
    return status;
}

VOID
EvtIoDeviceControl(
    _In_  WDFQUEUE          Queue,
    _In_  WDFREQUEST        Request,
    _In_  size_t            OutputBufferLength,
    _In_  size_t            InputBufferLength,
    _In_  ULONG             IoControlCode
    )
/*++
Routine Description:

    This event callback function is called when the driver receives an

    (KMDF) IOCTL_HID_Xxx code when handlng IRP_MJ_INTERNAL_DEVICE_CONTROL
    (UMDF) IOCTL_HID_Xxx, IOCTL_UMDF_HID_Xxx when handling IRP_MJ_DEVICE_CONTROL

Arguments:

    Queue - A handle to the queue object that is associated with the I/O request

    Request - A handle to a framework request object.

    OutputBufferLength - The length, in bytes, of the request's output buffer,
            if an output buffer is available.

    InputBufferLength - The length, in bytes, of the request's input buffer, if
            an input buffer is available.

    IoControlCode - The driver or system defined IOCTL associated with the request

Return Value:

    NTSTATUS

--*/
{
    NTSTATUS                status;
    BOOLEAN                 completeRequest = TRUE;
    WDFDEVICE               device = WdfIoQueueGetDevice(Queue);
    PDEVICE_CONTEXT         deviceContext = NULL;
    PQUEUE_CONTEXT          queueContext = GetQueueContext(Queue);
    UNREFERENCED_PARAMETER  (OutputBufferLength);
    UNREFERENCED_PARAMETER  (InputBufferLength);

    deviceContext = GetDeviceContext(device);

    switch (IoControlCode)
    {
    case IOCTL_GOODIX_TOUCH_GET_REPORT_RATE:
    case IOCTL_GOODIX_TOUCH_SET_REPORT_RATE:
        status = GoodixProcessControlRequest(queueContext->DeviceContext, Request, IoControlCode);
        break;

    case IOCTL_HID_GET_DEVICE_DESCRIPTOR:   // METHOD_NEITHER
        //
        // Retrieves the device's HID descriptor.
        //
        _Analysis_assume_(deviceContext->HidDescriptor.bLength != 0);
        status = RequestCopyFromBuffer(Request,
                            &deviceContext->HidDescriptor,
                            deviceContext->HidDescriptor.bLength);
        break;

    case IOCTL_HID_GET_DEVICE_ATTRIBUTES:   // METHOD_NEITHER
        //
        //Retrieves a device's attributes in a HID_DEVICE_ATTRIBUTES structure.
        //
        status = RequestCopyFromBuffer(Request,
                            &queueContext->DeviceContext->HidDeviceAttributes,
                            sizeof(HID_DEVICE_ATTRIBUTES));
        break;

    case IOCTL_HID_GET_REPORT_DESCRIPTOR:   // METHOD_NEITHER
        //
        //Obtains the report descriptor for the HID device.
        //
        status = RequestCopyFromBuffer(Request,
                            deviceContext->ReportDescriptor,
                            deviceContext->HidDescriptor.DescriptorList[0].wReportLength);
        break;

    case IOCTL_HID_READ_REPORT:             // METHOD_NEITHER
        //
        // Returns a report from the device into a class driver-supplied
        // buffer.
        //
        status = ReadReport(queueContext, Request, &completeRequest);
        break;

    case IOCTL_HID_WRITE_REPORT:            // METHOD_NEITHER
        //
        // Transmits a class driver-supplied report to the device.
        //
        status = WriteReport(queueContext, Request);
        break;

#ifdef _KERNEL_MODE

    case IOCTL_HID_GET_FEATURE:             // METHOD_OUT_DIRECT

        status = GetFeature(queueContext, Request);
        break;

    case IOCTL_HID_SET_FEATURE:             // METHOD_IN_DIRECT

        status = SetFeature(queueContext, Request);
        break;

    case IOCTL_HID_GET_INPUT_REPORT:        // METHOD_OUT_DIRECT

        status = GetInputReport(queueContext, Request);
        break;

    case IOCTL_HID_SET_OUTPUT_REPORT:       // METHOD_IN_DIRECT

        status = SetOutputReport(queueContext, Request);
        break;

#else // UMDF specific

    //
    // HID minidriver IOCTL uses HID_XFER_PACKET which contains an embedded pointer.
    //
    //   typedef struct _HID_XFER_PACKET {
    //     PUCHAR reportBuffer;
    //     ULONG  reportBufferLen;
    //     UCHAR  reportId;
    //   } HID_XFER_PACKET, *PHID_XFER_PACKET;
    //
    // UMDF cannot handle embedded pointers when marshalling buffers between processes.
    // Therefore a special driver mshidumdf.sys is introduced to convert such IRPs to
    // new IRPs (with new IOCTL name like IOCTL_UMDF_HID_Xxxx) where:
    //
    //   reportBuffer - passed as one buffer inside the IRP
    //   reportId     - passed as a second buffer inside the IRP
    //
    // The new IRP is then passed to UMDF host and driver for further processing.
    //

    case IOCTL_UMDF_HID_GET_FEATURE:        // METHOD_NEITHER

        status = GetFeature(queueContext, Request);
        break;

    case IOCTL_UMDF_HID_SET_FEATURE:        // METHOD_NEITHER

        status = SetFeature(queueContext, Request);
        break;

    case IOCTL_UMDF_HID_GET_INPUT_REPORT:  // METHOD_NEITHER

        status = GetInputReport(queueContext, Request);
        break;

    case IOCTL_UMDF_HID_SET_OUTPUT_REPORT: // METHOD_NEITHER

        status = SetOutputReport(queueContext, Request);
        break;

#endif // _KERNEL_MODE

    case IOCTL_HID_GET_STRING:                      // METHOD_NEITHER

        status = GetString(Request);
        break;

    case IOCTL_HID_GET_INDEXED_STRING:              // METHOD_OUT_DIRECT

        status = GetIndexedString(Request);
        break;

    case IOCTL_HID_SEND_IDLE_NOTIFICATION_REQUEST:  // METHOD_NEITHER
        //
        // This has the USBSS Idle notification callback. If the lower driver
        // can handle it (e.g. USB stack can handle it) then pass it down
        // otherwise complete it here as not inplemented. For a virtual
        // device, idling is not needed.
        //
        // Not implemented. fall through...
        //
    case IOCTL_HID_ACTIVATE_DEVICE:                 // METHOD_NEITHER
    case IOCTL_HID_DEACTIVATE_DEVICE:               // METHOD_NEITHER
    case IOCTL_GET_PHYSICAL_DESCRIPTOR:             // METHOD_OUT_DIRECT
        //
        // We don't do anything for these IOCTLs but some minidrivers might.
        //
        // Not implemented. fall through...
        //
    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }

    //
    // Complete the request. Information value has already been set by request
    // handlers.
    //
    if (completeRequest) {
        WdfRequestComplete(Request, status);
    }
}

NTSTATUS
RequestCopyFromBuffer(
    _In_  WDFREQUEST        Request,
    _In_  PVOID             SourceBuffer,
    _When_(NumBytesToCopyFrom == 0, __drv_reportError(NumBytesToCopyFrom cannot be zero))
    _In_  size_t            NumBytesToCopyFrom
    )
/*++

Routine Description:

    A helper function to copy specified bytes to the request's output memory

Arguments:

    Request - A handle to a framework request object.

    SourceBuffer - The buffer to copy data from.

    NumBytesToCopyFrom - The length, in bytes, of data to be copied.

Return Value:

    NTSTATUS

--*/
{
    NTSTATUS                status;
    WDFMEMORY               memory;
    size_t                  outputBufferLength;

    status = WdfRequestRetrieveOutputMemory(Request, &memory);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    WdfMemoryGetBuffer(memory, &outputBufferLength);
    if (outputBufferLength < NumBytesToCopyFrom) {
        status = STATUS_INVALID_BUFFER_SIZE;
        return status;
    }

    status = WdfMemoryCopyFromBuffer(memory,
                                    0,
                                    SourceBuffer,
                                    NumBytesToCopyFrom);
    if( !NT_SUCCESS(status) ) {
        
        return status;
    }
    WdfRequestSetInformation(Request, NumBytesToCopyFrom);
    return status;
}

NTSTATUS
ReadReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request,
    _Always_(_Out_)
          BOOLEAN*          CompleteRequest
    )
/*++

Routine Description:

    Handles IOCTL_HID_READ_REPORT for the HID collection. Normally the request
    will be forwarded to a manual queue for further process. In that case, the
    caller should not try to complete the request at this time, as the request
    will later be retrieved back from the manually queue and completed there.
    However, if for some reason the forwarding fails, the caller still need
    to complete the request with proper error code immediately.

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to  Request Packet.

    CompleteRequest - A boolean output value, indicating whether the caller
            should complete the request or not

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;

    //
    // forward the request to manual queue
    //
    status = WdfRequestForwardToIoQueue(
                            Request,
                            QueueContext->DeviceContext->ManualQueue);
    if( !NT_SUCCESS(status) ) {    
        *CompleteRequest = TRUE;
    }
    else {
        *CompleteRequest = FALSE;
    }

    return status;
}

NTSTATUS
WriteReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_WRITE_REPORT all the collection.

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to  Request Packet.

Return Value:

    NT status code.

--*/

{
    NTSTATUS                status;
    HID_XFER_PACKET         packet;
    ULONG                   reportSize;
    PHIDMINI_OUTPUT_REPORT  outputReport;

    status = RequestGetHidXferPacket_ToWriteToDevice(
                            Request,
                            &packet);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    if (packet.reportId != CONTROL_COLLECTION_REPORT_ID) {
        //
        // Return error for unknown collection
        //
        status = STATUS_INVALID_PARAMETER;
        return status;
    }

    //
    // before touching buffer make sure buffer is big enough.
    //
    reportSize = sizeof(HIDMINI_OUTPUT_REPORT);

    if (packet.reportBufferLen < reportSize) {
        status = STATUS_INVALID_BUFFER_SIZE;
        
        return status;
    }

    outputReport = (PHIDMINI_OUTPUT_REPORT)packet.reportBuffer;

    //
    // Store the device data in device extension.
    //
    QueueContext->DeviceContext->DeviceData = outputReport->Data;

    //
    // set status and information
    //
    WdfRequestSetInformation(Request, reportSize);
    return status;
}


HRESULT
GetFeature(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_GET_FEATURE for all the collection.

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to  Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    HID_XFER_PACKET         packet;
    ULONG                   reportSize;
    
    UNREFERENCED_PARAMETER(QueueContext);
    status = RequestGetHidXferPacket_ToReadFromDevice(
                            Request,
                            &packet);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    if (packet.reportId != CONTROL_COLLECTION_REPORT_ID) {
        //
        // If collection ID is not for control collection then handle
        // this request just as you would for a regular collection.
        //
        status = STATUS_INVALID_PARAMETER;
        
        
        return status;
    }

    //
    // Since output buffer is for write only (no read allowed by UMDF in output
    // buffer), any read from output buffer would be reading garbage), so don't
    // let app embed custom control code in output buffer. The minidriver can
    // support multiple features using separate report ID instead of using
    // custom control code. Since this is targeted at report ID 1, we know it
    // is a request for getting attributes.
    //
    // While KMDF does not enforce the rule (disallow read from output buffer),
    // it is good practice to not do so.
    //

    reportSize = sizeof(features);
    if (packet.reportBufferLen < reportSize) {
        status = STATUS_INVALID_BUFFER_SIZE;
        
        
        return status;
    }

    //
    // Since this device has one report ID, hidclass would pass on the report
    // ID in the buffer (it wouldn't if report descriptor did not have any report
    // ID). However, since UMDF allows only writes to an output buffer, we can't
    // "read" the report ID from "output" buffer. There is no need to read the
    // report ID since we get it other way as shown above, however this is
    // something to keep in mind.
    //
    packet.reportBuffer[0] = features.reportId;
    packet.reportBuffer[1] = features.DIG_TouchScreenContactCountMaximum;
    
    //
    // Report how many bytes were copied
    //
    WdfRequestSetInformation(Request, reportSize);
    return status;
}

NTSTATUS
SetFeature(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_SET_FEATURE for all the collection.
    For control collection (custom defined collection) it handles
    the user-defined control codes for sideband communication

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    HID_XFER_PACKET         packet;
    ULONG                   reportSize;
    PHIDMINI_CONTROL_INFO   controlInfo;
    PHID_DEVICE_ATTRIBUTES  hidAttributes = &QueueContext->DeviceContext->HidDeviceAttributes;

    status = RequestGetHidXferPacket_ToWriteToDevice(
                            Request,
                            &packet);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    if (packet.reportId != CONTROL_COLLECTION_REPORT_ID) {
        //
        // If collection ID is not for control collection then handle
        // this request just as you would for a regular collection.
        //
        status = STATUS_INVALID_PARAMETER;
        
        
        return status;
    }

    //
    // before touching control code make sure buffer is big enough.
    //
    reportSize = sizeof(HIDMINI_CONTROL_INFO);

    if (packet.reportBufferLen < reportSize) {
        status = STATUS_INVALID_BUFFER_SIZE;
        
        
        return status;
    }

    controlInfo = (PHIDMINI_CONTROL_INFO)packet.reportBuffer;

    switch(controlInfo->ControlCode)
    {
    case HIDMINI_CONTROL_CODE_SET_ATTRIBUTES:
        //
        // Store the device attributes in device extension
        //
        hidAttributes->ProductID     = controlInfo->u.Attributes.ProductID;
        hidAttributes->VendorID      = controlInfo->u.Attributes.VendorID;
        hidAttributes->VersionNumber = controlInfo->u.Attributes.VersionNumber;

        //
        // set status and information
        //
        WdfRequestSetInformation(Request, reportSize);
        break;

    case HIDMINI_CONTROL_CODE_DUMMY1:
        status = STATUS_NOT_IMPLEMENTED;
        
        break;

    case HIDMINI_CONTROL_CODE_DUMMY2:
        status = STATUS_NOT_IMPLEMENTED;
        
        break;

    case HIDMINI_CONTROL_CODE_SET_REPORT_RATE:
    {
        UINT8 requestedLevel;
        UINT8 previousPersistentLevel;

        requestedLevel = GoodixNormalizeReportRateLevel((UINT8)controlInfo->u.ReportRate.Level);
        previousPersistentLevel = QueueContext->DeviceContext->ReportRateLevel;
        QueueContext->DeviceContext->ReportRateLevel = requestedLevel;

        status = GoodixPersistReportRateLevel(
            QueueContext->DeviceContext,
            requestedLevel);
        if (!NT_SUCCESS(status)) {
            QueueContext->DeviceContext->ReportRateLevel = previousPersistentLevel;
            break;
        }

        status = GoodixReinitializeAfterReportRateChange(QueueContext->DeviceContext);
        if (!NT_SUCCESS(status)) {
            QueueContext->DeviceContext->ReportRateLevel = previousPersistentLevel;
            (void)GoodixPersistReportRateLevel(
                QueueContext->DeviceContext,
                previousPersistentLevel);
            break;
        }

        WdfRequestSetInformation(Request, reportSize);
        break;
    }

    default:
        status = STATUS_NOT_IMPLEMENTED;
        break;
    }
    return status;
}

NTSTATUS
GetInputReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_GET_INPUT_REPORT for all the collection.

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    HID_XFER_PACKET         packet;
    ULONG                   reportSize;
    PHIDMINI_INPUT_REPORT   reportBuffer;

    status = RequestGetHidXferPacket_ToReadFromDevice(
                            Request,
                            &packet);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    if (packet.reportId != CONTROL_COLLECTION_REPORT_ID) {
        //
        // If collection ID is not for control collection then handle
        // this request just as you would for a regular collection.
        //
        status = STATUS_INVALID_PARAMETER;
        
        return status;
    }

    reportSize = sizeof(HIDMINI_INPUT_REPORT);
    if (packet.reportBufferLen < reportSize) {
        status = STATUS_INVALID_BUFFER_SIZE;
        
        return status;
    }

    reportBuffer = (PHIDMINI_INPUT_REPORT)(packet.reportBuffer);

    reportBuffer->ReportId = CONTROL_COLLECTION_REPORT_ID;
    reportBuffer->Data     = QueueContext->OutputReport;

    //
    // Report how many bytes were copied
    //
    WdfRequestSetInformation(Request, reportSize);
    return status;
}


NTSTATUS
SetOutputReport(
    _In_  PQUEUE_CONTEXT    QueueContext,
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_SET_OUTPUT_REPORT for all the collection.

Arguments:

    QueueContext - The object context associated with the queue

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    HID_XFER_PACKET         packet;
    ULONG                   reportSize;
    PHIDMINI_OUTPUT_REPORT  reportBuffer;

    status = RequestGetHidXferPacket_ToWriteToDevice(
                            Request,
                            &packet);
    if( !NT_SUCCESS(status) ) {
        return status;
    }

    if (packet.reportId != CONTROL_COLLECTION_REPORT_ID) {
        //
        // If collection ID is not for control collection then handle
        // this request just as you would for a regular collection.
        //
        status = STATUS_INVALID_PARAMETER;
        
        return status;
    }

    //
    // before touching buffer make sure buffer is big enough.
    //
    reportSize = sizeof(HIDMINI_OUTPUT_REPORT);

    if (packet.reportBufferLen < reportSize) {
        status = STATUS_INVALID_BUFFER_SIZE;
        return status;
    }

    reportBuffer = (PHIDMINI_OUTPUT_REPORT)packet.reportBuffer;

    QueueContext->OutputReport = reportBuffer->Data;

    //
    // Report how many bytes were copied
    //
    WdfRequestSetInformation(Request, reportSize);
    return status;
}


NTSTATUS
GetStringId(
    _In_  WDFREQUEST        Request,
    _Out_ ULONG            *StringId,
    _Out_ ULONG            *LanguageId
    )
/*++

Routine Description:

    Helper routine to decode IOCTL_HID_GET_INDEXED_STRING and IOCTL_HID_GET_STRING.

Arguments:

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    ULONG                   inputValue;

#ifdef _KERNEL_MODE

    WDF_REQUEST_PARAMETERS  requestParameters;

    //
    // IOCTL_HID_GET_STRING:                      // METHOD_NEITHER
    // IOCTL_HID_GET_INDEXED_STRING:              // METHOD_OUT_DIRECT
    //
    // The string id (or string index) is passed in Parameters.DeviceIoControl.
    // Type3InputBuffer. However, Parameters.DeviceIoControl.InputBufferLength
    // was not initialized by hidclass.sys, therefore trying to access the
    // buffer with WdfRequestRetrieveInputMemory will fail
    //
    // Another problem with IOCTL_HID_GET_INDEXED_STRING is that METHOD_OUT_DIRECT
    // expects the input buffer to be Irp->AssociatedIrp.SystemBuffer instead of
    // Type3InputBuffer. That will also fail WdfRequestRetrieveInputMemory.
    //
    // The solution to the above two problems is to get Type3InputBuffer directly
    //
    // Also note that instead of the buffer's content, it is the buffer address
    // that was used to store the string id (or index)
    //

    WDF_REQUEST_PARAMETERS_INIT(&requestParameters);
    WdfRequestGetParameters(Request, &requestParameters);

    inputValue = PtrToUlong(
        requestParameters.Parameters.DeviceIoControl.Type3InputBuffer);

    status = STATUS_SUCCESS;

#else

    WDFMEMORY               inputMemory;
    size_t                  inputBufferLength;
    PVOID                   inputBuffer;

    //
    // mshidumdf.sys updates the IRP and passes the string id (or index) through
    // the input buffer correctly based on the IOCTL buffer type
    //

    status = WdfRequestRetrieveInputMemory(Request, &inputMemory);
    if( !NT_SUCCESS(status) ) {
        KdPrint(("WdfRequestRetrieveInputMemory failed 0x%x\n",status));
        return status;
    }
    inputBuffer = WdfMemoryGetBuffer(inputMemory, &inputBufferLength);

    //
    // make sure buffer is big enough.
    //
    if (inputBufferLength < sizeof(ULONG))
    {
        status = STATUS_INVALID_BUFFER_SIZE;
        KdPrint(("GetStringId: invalid input buffer. size %d, expect %d\n",
                            (int)inputBufferLength, (int)sizeof(ULONG)));
        return status;
    }

    inputValue = (*(PULONG)inputBuffer);

#endif

    //
    // The least significant two bytes of the INT value contain the string id.
    //
    *StringId = (inputValue & 0x0ffff);

    //
    // The most significant two bytes of the INT value contain the language
    // ID (for example, a value of 1033 indicates English).
    //
    *LanguageId = (inputValue >> 16);
    return status;
}


NTSTATUS
GetIndexedString(
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_GET_INDEXED_STRING

Arguments:

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    ULONG                   languageId, stringIndex;

    status = GetStringId(Request, &stringIndex, &languageId);

    // While we don't use the language id, some minidrivers might.
    //
    UNREFERENCED_PARAMETER(languageId);

    if (NT_SUCCESS(status)) {

        if (stringIndex != VHIDMINI_DEVICE_STRING_INDEX)
        {
            status = STATUS_INVALID_PARAMETER;
            
            return status;
        }

        status = RequestCopyFromBuffer(Request, VHIDMINI_DEVICE_STRING, sizeof(VHIDMINI_DEVICE_STRING));
    }
    return status;
}


NTSTATUS
GetString(
    _In_  WDFREQUEST        Request
    )
/*++

Routine Description:

    Handles IOCTL_HID_GET_STRING.

Arguments:

    Request - Pointer to Request Packet.

Return Value:

    NT status code.

--*/
{
    NTSTATUS                status;
    ULONG                   languageId, stringId;
    size_t                  stringSizeCb;
    PWSTR                   string;

    status = GetStringId(Request, &stringId, &languageId);

    // While we don't use the language id, some minidrivers might.
    //
    UNREFERENCED_PARAMETER(languageId);

    if (!NT_SUCCESS(status)) {
        return status;
    }

    switch (stringId){
    case HID_STRING_ID_IMANUFACTURER:
        stringSizeCb = sizeof(VHIDMINI_MANUFACTURER_STRING);
        string = VHIDMINI_MANUFACTURER_STRING;
        break;
    case HID_STRING_ID_IPRODUCT:
        stringSizeCb = sizeof(VHIDMINI_PRODUCT_STRING);
        string = VHIDMINI_PRODUCT_STRING;
        break;
    case HID_STRING_ID_ISERIALNUMBER:
        stringSizeCb = sizeof(VHIDMINI_SERIAL_NUMBER_STRING);
        string = VHIDMINI_SERIAL_NUMBER_STRING;
        break;
    default:
        status = STATUS_INVALID_PARAMETER;
        
        return status;
    }

    status = RequestCopyFromBuffer(Request, string, stringSizeCb);
    return status;
}


NTSTATUS
ManualQueueCreate(
    _In_  WDFDEVICE         Device,
    _Out_ WDFQUEUE          *Queue
    )
/*++
Routine Description:

    This function creates a manual I/O queue to receive IOCTL_HID_READ_REPORT
    forwarded from the device's default queue handler.

    It also creates a periodic timer to check the queue and complete any pending
    request with data from the device. Here timer expiring is used to simulate
    a hardware event that new data is ready.

    The workflow is like this:

    - Hidclass.sys sends an ioctl to the miniport to read input report.

    - The request reaches the driver's default queue. As data may not be avaiable
      yet, the request is forwarded to a second manual queue temporarily.

    - Later when data is ready (as simulated by timer expiring), the driver
      checks for any pending request in the manual queue, and then completes it.

    - Hidclass gets notified for the read request completion and return data to
      the caller.

    On the other hand, for IOCTL_HID_WRITE_REPORT request, the driver simply
    sends the request to the hardware (as simulated by storing the data at
    DeviceContext->DeviceData) and completes the request immediately. There is
    no need to use another queue for write operation.

Arguments:

    Device - Handle to a framework device object.

    Queue - Output pointer to a framework I/O queue handle, on success.

Return Value:

    NTSTATUS

--*/
{
    NTSTATUS                status;
    WDF_IO_QUEUE_CONFIG     queueConfig;
    WDF_OBJECT_ATTRIBUTES   queueAttributes;
    WDFQUEUE                queue;
    PMANUAL_QUEUE_CONTEXT   queueContext;
    
    WDF_IO_QUEUE_CONFIG_INIT(
                            &queueConfig,
                            WdfIoQueueDispatchManual);

    WDF_OBJECT_ATTRIBUTES_INIT_CONTEXT_TYPE(
                            &queueAttributes,
                            MANUAL_QUEUE_CONTEXT);

    status = WdfIoQueueCreate(
                            Device,
                            &queueConfig,
                            &queueAttributes,
                            &queue);

    if( !NT_SUCCESS(status) ) {
        
        
        return status;
    }

    queueContext = GetManualQueueContext(queue);
    queueContext->Queue         = queue;
    queueContext->DeviceContext = GetDeviceContext(Device);

    *Queue = queue;
    return status;
}

void
EvtTimerFunc(
    _In_  WDFTIMER          Timer
    )
/*++
Routine Description:

    This periodic timer callback routine checks the device's manual queue and
    completes any pending request with data from the device.

Arguments:

    Timer - Handle to a timer object that was obtained from WdfTimerCreate.

Return Value:

    VOID

--*/
{
    NTSTATUS                status;
    WDFQUEUE                queue;
    PMANUAL_QUEUE_CONTEXT   queueContext;
    WDFREQUEST              request;
    HIDMINI_INPUT_REPORT    readReport;

    queue = (WDFQUEUE)WdfTimerGetParentObject(Timer);
    queueContext = GetManualQueueContext(queue);

    //
    // see if we have a request in manual queue
    //
    status = WdfIoQueueRetrieveNextRequest(
                            queueContext->Queue,
                            &request);

    if (NT_SUCCESS(status)) {

        readReport.ReportId = CONTROL_FEATURE_REPORT_ID;
        readReport.Data     = queueContext->DeviceContext->DeviceData;

        status = RequestCopyFromBuffer(request,
                            &readReport,
                            sizeof(readReport));

        WdfRequestComplete(request, status);
    }
}

BOOLEAN
OnInterruptIsr(
    _In_  WDFINTERRUPT FxInterrupt,
    _In_  ULONG        MessageID
)
/*++

  Routine Description:

    This routine responds to interrupts generated by the H/W.
    It then waits indefinitely for the user to signal that
    the interrupt has been acknowledged, allowing the ISR to
    return. This ISR is called at PASSIVE_LEVEL.

  Arguments:

    Interrupt - a handle to a framework interrupt object
    MessageID - message number identifying the device's
        hardware interrupt message (if using MSI)

  Return Value:

    TRUE if interrupt recognized.

--*/
{
    BOOLEAN           fInterruptRecognized = TRUE;
    WDFDEVICE         device;
    PDEVICE_CONTEXT   pDevice;
    NTSTATUS          status;
    WDFREQUEST        request;
    inputReport54_t   readReport = { 0 };
    UINT8 infoBuf[3] = { 0 };
    UINT8 touchBuf[10 + MAX_POINT_NUM * BYTES_PER_COORD] = { 0 };
    UINT8 touchEvtClear = 0;
    BOOLEAN clearTouchEvent = FALSE;

    UINT8 touchId = 0;
    UINT16 x = 0, y = 0;
    UNREFERENCED_PARAMETER(MessageID);

    device = WdfInterruptGetDevice(FxInterrupt);

    pDevice = GetDeviceContext(device);
    if (pDevice->OnClose)
        return TRUE;

    RtlZeroMemory(touchBuf, sizeof(touchBuf));
    RtlZeroMemory(infoBuf, sizeof(infoBuf));

    status = GoodixRead(pDevice, pDevice->TouchDataAddress, infoBuf, sizeof(infoBuf));
    if (!NT_SUCCESS(status)) {
        pDevice->ConsecutiveTransportFailures += 1;
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "Goodix interrupt header read failed status=0x%08X failures=%lu",
            status,
            pDevice->ConsecutiveTransportFailures);
        if (pDevice->ConsecutiveTransportFailures >= GOODIX_TRANSPORT_FAILURE_THRESHOLD) {
            GoodixQueueControllerRequest(pDevice, GOODIX_DEFERRED_REQUEST_RECOVER);
        }
        goto exit;
    }

    pDevice->ConsecutiveTransportFailures = 0;
    clearTouchEvent = TRUE;

    // touchBuf[0] EventID
    if ((infoBuf[0] & GOODIX_TOUCH_EVENT) != 0) {
        // Continue into touch processing below.
    } else if ((infoBuf[0] & GOODIX_REQUEST_EVENT) != 0) {
        GoodixQueueControllerRequest(pDevice, infoBuf[2]);
        goto exit;
    } else if (infoBuf[0] == EVT_ID_CONTROLLER_READY) {
        if (!pDevice->ConfigApplied) {
            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
                "Goodix controller ready, queueing deferred config");
            GoodixQueueControllerRequest(pDevice, 0x01);
        }
        goto exit;
    } else if ((infoBuf[0] & (GOODIX_GESTURE_EVENT | GOODIX_HOTKNOT_EVENT)) != 0) {
        goto exit;
    } else {
        goto exit;
    }

    readReport.DIG_TouchScreenContactCount = infoBuf[2];

    BYTE touch_count = infoBuf[2] & 0x0F;
    UINT8 previousTouchCount = pDevice->LastLoggedTouchCount;
    UINT8 firstTouchId = 0;
    UINT16 firstTouchX = 0;
    UINT16 firstTouchY = 0;

    if (touch_count > 10) // The touchscreen doesn't support more than 10 points.
    {
        TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "The touchscreen does not support more than 10 points. Defaulting to 10.");
        touch_count = 10;
    }

    if (touch_count > 0) {
        status = GoodixRead(
            pDevice,
            pDevice->TouchDataAddress + 8,
            touchBuf,
            10 + BYTES_PER_COORD * (touch_count - 1));
        if (!NT_SUCCESS(status)) {
            pDevice->ConsecutiveTransportFailures += 1;
            TraceEvents(
                TRACE_LEVEL_WARNING,
                TRACE_DEVICE,
                "Goodix touch payload read failed status=0x%08X failures=%lu",
                status,
                pDevice->ConsecutiveTransportFailures);
            if (pDevice->ConsecutiveTransportFailures >= GOODIX_TRANSPORT_FAILURE_THRESHOLD) {
                GoodixQueueControllerRequest(pDevice, GOODIX_DEFERRED_REQUEST_RECOVER);
            }
            clearTouchEvent = FALSE;
            goto exit;
        }

        pDevice->ConsecutiveTransportFailures = 0;
    }

    switch(touch_count)
    {
        case 0: {
            // All points leave
            readReport.points[0] = 0x06;
            readReport.points[1] = pDevice->LastTouchID;
            readReport.DIG_TouchScreenContactCount = 1;
            if (previousTouchCount != 0 && previousTouchCount != 0xFF) {
                TraceEvents(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_DEVICE,
                    "Touch released id=%u",
                    pDevice->LastTouchID);
            }
            pDevice->LastLoggedTouchCount = 0;
            break;
        }
        default: {
            for (UINT8 i = 0; i < touch_count; i++)
            {
                touchId = ((touchBuf[0 + i * 8] >> 4) & 0x0F);
                x = ((touchBuf[3 + i * 8] << 8) | touchBuf[2 + i * 8]) / 0x10;
                y = ((touchBuf[5 + i * 8] << 8) | touchBuf[4 + i * 8]) / 0x10;
                if (i == 0) {
                    firstTouchId = touchId;
                    firstTouchX = x;
                    firstTouchY = y;
                }
                pDevice->LastTouchID = touchId;
                readReport.points[i * 6 + 0] = 0x07;  // In Point
                readReport.points[i * 6 + 1] = touchId;
                readReport.points[i * 6 + 2] = x & 0xFF;
                readReport.points[i * 6 + 3] = (x >> 8) & 0x0F;
                readReport.points[i * 6 + 4] = y & 0xFF;
                readReport.points[i * 6 + 5] = (y >> 8) & 0x0F;
            }
            if (previousTouchCount != touch_count) {
                TraceEvents(
                    TRACE_LEVEL_INFORMATION,
                    TRACE_DEVICE,
                    "Touch contacts=%u firstId=%u x=%u y=%u",
                    touch_count,
                    firstTouchId,
                    firstTouchX,
                    firstTouchY);
            }
            pDevice->LastLoggedTouchCount = touch_count;
        }
    }

    status = WdfIoQueueRetrieveNextRequest(
        pDevice->ManualQueue,
        &request);

    if (NT_SUCCESS(status)) {

        readReport.reportId = CONTROL_FEATURE_REPORT_ID;

        status = RequestCopyFromBuffer(request,
            &readReport,
            sizeof(readReport));

        WdfRequestComplete(request, status);
    }

exit:
    if (clearTouchEvent) {
        status = GoodixWrite(pDevice, pDevice->TouchDataAddress, &touchEvtClear, 1);
        if (!NT_SUCCESS(status)) {
            pDevice->ConsecutiveTransportFailures += 1;
            TraceEvents(
                TRACE_LEVEL_WARNING,
                TRACE_DEVICE,
                "Goodix interrupt clear failed status=0x%08X failures=%lu",
                status,
                pDevice->ConsecutiveTransportFailures);
            if (pDevice->ConsecutiveTransportFailures >= GOODIX_TRANSPORT_FAILURE_THRESHOLD) {
                GoodixQueueControllerRequest(pDevice, GOODIX_DEFERRED_REQUEST_RECOVER);
            }
        }
    }
    return fInterruptRecognized;
}

NTSTATUS
GoodixRead(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ UINT32 addr,
    _In_ UINT8* readBuf,
    _In_ UINT32 readLen
)
{
    NTSTATUS status = STATUS_SUCCESS;
    PUINT8 TxBuf = (PUINT8)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        8 + readLen,
        TOUCH_POOL_TAG
    );
    PUINT8 RxBuf = (PUINT8)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        8 + readLen,
        TOUCH_POOL_TAG
    );

    if (TxBuf == NULL || RxBuf == NULL) {
        status = STATUS_INSUFFICIENT_RESOURCES;
        goto Exit;
    }

    TxBuf[0] = GOODIX_SPI_READ;
    TxBuf[1] = (addr >> 24) & 0xFF;
    TxBuf[2] = (addr >> 16) & 0xFF;
    TxBuf[3] = (addr >> 8) & 0xFF;
    TxBuf[4] = addr & 0xFF;
    TxBuf[5] = 0xFF;
    TxBuf[6] = 0xFF;
    TxBuf[7] = 0xFF;

    status = SpbDeviceWriteRead(pDevice, TxBuf, RxBuf, 8 + readLen, 8 + readLen);
    if (NT_SUCCESS(status)) {
        RtlCopyMemory(readBuf, &RxBuf[8], readLen);
    }

Exit:
    ExFreePoolWithTag(TxBuf, TOUCH_POOL_TAG);
    ExFreePoolWithTag(RxBuf, TOUCH_POOL_TAG);
    return status;
}

NTSTATUS
GoodixWrite(
    _In_ PDEVICE_CONTEXT pDevice, 
    _In_ UINT32 addr, 
    _In_ UINT8* writeBuf, 
    _In_ UINT32 writeLen
)
{
    UINT8* SpbBuf = (UINT8*)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        writeLen + 5,
        TOUCH_POOL_TAG
    );
    NTSTATUS status = STATUS_SUCCESS;

    if (SpbBuf == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    SpbBuf[0] = GOODIX_SPI_WRITE;
    SpbBuf[1] = (addr >> 24) & 0xFF;
    SpbBuf[2] = (addr >> 16) & 0xFF;
    SpbBuf[3] = (addr >> 8) & 0xFF;
    SpbBuf[4] = addr & 0xFF;
    RtlCopyMemory(&SpbBuf[5], writeBuf, writeLen);

    status = SpbDeviceWrite(pDevice, SpbBuf, writeLen + 5);

    ExFreePoolWithTag(SpbBuf, TOUCH_POOL_TAG);
    return status;
}

static
NTSTATUS
GoodixOpenResetGpio(
    _In_ PDEVICE_CONTEXT DeviceContext
    )
{
    NTSTATUS status;
    WDF_OBJECT_ATTRIBUTES targetAttributes;
    WDF_IO_TARGET_OPEN_PARAMS openParams;
    DECLARE_UNICODE_STRING_SIZE(devicePath, RESOURCE_HUB_PATH_SIZE);

    if (!DeviceContext->ResetGpioPresent) {
        return STATUS_NOT_SUPPORTED;
    }

    if (DeviceContext->ResetGpioIoTarget != WDF_NO_HANDLE) {
        return STATUS_SUCCESS;
    }

    status = RESOURCE_HUB_CREATE_PATH_FROM_ID(
        &devicePath,
        DeviceContext->ResetGpioId.LowPart,
        DeviceContext->ResetGpioId.HighPart);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    WDF_OBJECT_ATTRIBUTES_INIT(&targetAttributes);
    status = WdfIoTargetCreate(
        DeviceContext->Device,
        &targetAttributes,
        &DeviceContext->ResetGpioIoTarget);
    if (!NT_SUCCESS(status)) {
        DeviceContext->ResetGpioIoTarget = WDF_NO_HANDLE;
        return status;
    }

    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME(
        &openParams,
        &devicePath,
        GENERIC_WRITE);
    openParams.ShareAccess = 0;
    openParams.CreateDisposition = FILE_OPEN;
    openParams.FileAttributes = FILE_ATTRIBUTE_NORMAL;

    status = WdfIoTargetOpen(DeviceContext->ResetGpioIoTarget, &openParams);
    if (!NT_SUCCESS(status)) {
        WdfObjectDelete(DeviceContext->ResetGpioIoTarget);
        DeviceContext->ResetGpioIoTarget = WDF_NO_HANDLE;
    }

    return status;
}

static
VOID
GoodixCloseResetGpio(
    _In_ PDEVICE_CONTEXT DeviceContext
    )
{
    if (DeviceContext->ResetGpioIoTarget != WDF_NO_HANDLE) {
        WdfIoTargetClose(DeviceContext->ResetGpioIoTarget);
        WdfObjectDelete(DeviceContext->ResetGpioIoTarget);
        DeviceContext->ResetGpioIoTarget = WDF_NO_HANDLE;
    }
}

static
NTSTATUS
GoodixWriteResetGpio(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ BOOLEAN Level
    )
{
    UCHAR value = Level ? 1U : 0U;
    WDF_MEMORY_DESCRIPTOR inputDescriptor;
    WDF_MEMORY_DESCRIPTOR outputDescriptor;
    WDF_REQUEST_SEND_OPTIONS requestSendOptions;

    if (DeviceContext->ResetGpioIoTarget == WDF_NO_HANDLE) {
        return STATUS_NOT_SUPPORTED;
    }

    WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&inputDescriptor, &value, sizeof(value));
    WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&outputDescriptor, &value, sizeof(value));
    GoodixInitRequestSendOptions(&requestSendOptions, GOODIX_SPB_XFER_TIMEOUT_MS);

    return WdfIoTargetSendIoctlSynchronously(
        DeviceContext->ResetGpioIoTarget,
        NULL,
        IOCTL_GPIO_WRITE_PINS,
        &inputDescriptor,
        &outputDescriptor,
        &requestSendOptions,
        NULL);
}

static
NTSTATUS
GoodixResetDevice(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ ULONG DelayMs
    )
{
    NTSTATUS status;

    if (DeviceContext->ResetGpioIoTarget == WDF_NO_HANDLE) {
        return STATUS_NOT_SUPPORTED;
    }

    status = GoodixWriteResetGpio(DeviceContext, FALSE);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    GoodixDelayMilliseconds(2);

    status = GoodixWriteResetGpio(DeviceContext, TRUE);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (DelayMs != 0) {
        GoodixDelayMilliseconds((LONG)DelayMs);
    }

    return STATUS_SUCCESS;
}

static
NTSTATUS
GoodixWriteConfirm(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT32 Address,
    _In_reads_bytes_(Length) const UINT8* Buffer,
    _In_ ULONG Length
    )
{
    NTSTATUS status;
    UINT8* readBack;

    readBack = (UINT8*)ExAllocatePool2(POOL_FLAG_NON_PAGED, Length, TOUCH_POOL_TAG);
    if (readBack == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    status = GoodixWriteLarge(DeviceContext, Address, Buffer, Length);
    if (NT_SUCCESS(status)) {
        status = GoodixReadLarge(DeviceContext, Address, readBack, Length);
        if (NT_SUCCESS(status) && RtlCompareMemory(Buffer, readBack, Length) != Length) {
            status = STATUS_CRC_ERROR;
        }
    }

    ExFreePoolWithTag(readBack, TOUCH_POOL_TAG);
    return status;
}

static
NTSTATUS
GoodixParseEmbeddedFirmware(
    _Out_ PGOODIX_PARSED_FW ParsedFirmware
    )
{
    const UINT8* firmware = g_GoodixFw9916rBin;
    ULONG firmwareLength = g_GoodixFw9916rBinLen;
    ULONG expectedSize;
    ULONG checksum = 0;
    ULONG infoOffset;
    ULONG firmwareOffset;
    ULONG i;

    RtlZeroMemory(ParsedFirmware, sizeof(*ParsedFirmware));

    if (firmwareLength < GOODIX_FW_HEADER_SIZE) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    expectedSize = GoodixReadU32Le(&firmware[0]) + GOODIX_FW_FILE_CHECKSUM_OFFSET;
    if (expectedSize != firmwareLength) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    for (i = GOODIX_FW_FILE_CHECKSUM_OFFSET; (i + 1) < firmwareLength; i += 2) {
        checksum += firmware[i] | ((ULONG)firmware[i + 1] << 8);
    }

    if (checksum != GoodixReadU32Le(&firmware[4])) {
        return STATUS_CRC_ERROR;
    }

    RtlCopyMemory(ParsedFirmware->FwPid, &firmware[17], sizeof(ParsedFirmware->FwPid));
    RtlCopyMemory(ParsedFirmware->FwVid, &firmware[25], sizeof(ParsedFirmware->FwVid));
    ParsedFirmware->SubsysNum = firmware[29];
    ParsedFirmware->ChipType = firmware[30];
    ParsedFirmware->ProtocolVer = firmware[31];
    ParsedFirmware->BusType = firmware[32];
    ParsedFirmware->FlashProtect = firmware[33];

    if (ParsedFirmware->ChipType != GOODIX_FW_BRD_CHIP_TYPE ||
        ParsedFirmware->SubsysNum == 0 ||
        ParsedFirmware->SubsysNum > GOODIX_FW_SUBSYS_MAX_NUM) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    firmwareOffset = GOODIX_FW_HEADER_SIZE;
    for (i = 0; i < ParsedFirmware->SubsysNum; i++) {
        UINT32 size;

        infoOffset = GOODIX_FW_SUBSYS_INFO_OFFSET + (i * GOODIX_FW_SUBSYS_INFO_SIZE);
        size = GoodixReadU32Le(&firmware[infoOffset + 1]);
        if ((firmwareOffset + size) > firmwareLength) {
            return STATUS_INVALID_IMAGE_FORMAT;
        }

        ParsedFirmware->Subsys[i].Type = firmware[infoOffset];
        ParsedFirmware->Subsys[i].Size = size;
        ParsedFirmware->Subsys[i].FlashAddress = GoodixReadU32Le(&firmware[infoOffset + 5]);
        ParsedFirmware->Subsys[i].Data = &firmware[firmwareOffset];
        firmwareOffset += size;
    }

    return STATUS_SUCCESS;
}

static
BOOLEAN
GoodixFirmwareNeedsUpdate(
    _In_ const GOODIX_FW_VERSION* Version,
    _In_ const GOODIX_PARSED_FW* ParsedFirmware
    )
{
    static const CHAR noCode[] = "NOCODE";

    if (RtlCompareMemory(Version->RomPid, noCode, sizeof(noCode) - 1) == (sizeof(noCode) - 1) ||
        RtlCompareMemory(Version->PatchPid, noCode, sizeof(noCode) - 1) == (sizeof(noCode) - 1)) {
        return TRUE;
    }

    if (RtlCompareMemory(Version->PatchPid, ParsedFirmware->FwPid, sizeof(ParsedFirmware->FwPid)) != sizeof(ParsedFirmware->FwPid)) {
        return TRUE;
    }

    if (RtlCompareMemory(Version->PatchVid, ParsedFirmware->FwVid, sizeof(ParsedFirmware->FwVid)) != sizeof(ParsedFirmware->FwVid)) {
        return TRUE;
    }

    return FALSE;
}

static
NTSTATUS
GoodixHoldCpu(
    _In_ PDEVICE_CONTEXT DeviceContext
    )
{
    NTSTATUS status = STATUS_UNSUCCESSFUL;
    UINT8 regValue[2] = { 0x01, 0x00 };
    UINT8 tempBuffer[12];
    ULONG retry;

    for (retry = 0; retry < 100; retry++) {
        status = GoodixWrite(DeviceContext, GOODIX_FW_HOLD_CPU_REG_W, regValue, sizeof(regValue));
        if (!NT_SUCCESS(status)) {
            GoodixDelayMilliseconds(1);
            continue;
        }

        status = GoodixRead(DeviceContext, GOODIX_FW_HOLD_CPU_REG_R, &tempBuffer[0], 4);
        status = NT_SUCCESS(status) ? GoodixRead(DeviceContext, GOODIX_FW_HOLD_CPU_REG_R, &tempBuffer[4], 4) : status;
        status = NT_SUCCESS(status) ? GoodixRead(DeviceContext, GOODIX_FW_HOLD_CPU_REG_R, &tempBuffer[8], 4) : status;
        if (NT_SUCCESS(status) &&
            RtlCompareMemory(&tempBuffer[0], &tempBuffer[4], 4) == 4 &&
            RtlCompareMemory(&tempBuffer[4], &tempBuffer[8], 4) == 4) {
            return STATUS_SUCCESS;
        }

        GoodixDelayMilliseconds(1);
    }

    return STATUS_IO_TIMEOUT;
}

static
NTSTATUS
GoodixLoadIsp(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ const GOODIX_PARSED_FW* ParsedFirmware
    )
{
    NTSTATUS status;
    GOODIX_FW_VERSION ispVersion = { 0 };
    UINT8 bootOption[8];

    if (ParsedFirmware->SubsysNum == 0 || ParsedFirmware->Subsys[0].Size == 0) {
        return STATUS_INVALID_IMAGE_FORMAT;
    }

    status = GoodixWriteConfirm(
        DeviceContext,
        GOODIX_FW_ISP_RAM_ADDR_BRD,
        ParsedFirmware->Subsys[0].Data,
        ParsedFirmware->Subsys[0].Size);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    RtlFillMemory(bootOption, sizeof(bootOption), 0x55);
    status = GoodixWriteConfirm(
        DeviceContext,
        GOODIX_FW_CPU_RUN_FROM_REG,
        bootOption,
        sizeof(bootOption));
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixResetDevice(DeviceContext, 100);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixReadVersion(DeviceContext, &ispVersion);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (RtlCompareMemory(&ispVersion.PatchPid[3], "ISP", 3) != 3) {
        return STATUS_REVISION_MISMATCH;
    }

    return STATUS_SUCCESS;
}

static
NTSTATUS
GoodixUpdatePrepare(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ const GOODIX_PARSED_FW* ParsedFirmware
    )
{
    NTSTATUS status;
    UINT32 misctlValue = GOODIX_FW_ENABLE_MISCTL_BRD;
    UINT8 watchdog = 0x00;

    UNREFERENCED_PARAMETER(ParsedFirmware);

    status = GoodixResetDevice(DeviceContext, 5);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixHoldCpu(DeviceContext);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixWrite(DeviceContext, GOODIX_FW_MISCTL_REG_BRD, (UINT8*)&misctlValue, sizeof(misctlValue));
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixWrite(DeviceContext, GOODIX_FW_WATCHDOG_REG_BRD, &watchdog, sizeof(watchdog));
    if (!NT_SUCCESS(status)) {
        return status;
    }

    return GoodixLoadIsp(DeviceContext, ParsedFirmware);
}

static
NTSTATUS
GoodixSendFlashCmd(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 FwType,
    _In_ UINT16 FwLength,
    _In_ UINT32 FwAddress
    )
{
    NTSTATUS status;
    UINT8 cmd[16] = { 0 };
    UINT8 ack[16] = { 0 };
    ULONG i;

    cmd[2] = GOODIX_FLASH_CMD_LEN;
    cmd[3] = GOODIX_FLASH_CMD_TYPE_WRITE;
    cmd[4] = FwType;
    cmd[5] = (UINT8)(FwLength & 0xFF);
    cmd[6] = (UINT8)((FwLength >> 8) & 0xFF);
    cmd[7] = (UINT8)(FwAddress & 0xFF);
    cmd[8] = (UINT8)((FwAddress >> 8) & 0xFF);
    cmd[9] = (UINT8)((FwAddress >> 16) & 0xFF);
    cmd[10] = (UINT8)((FwAddress >> 24) & 0xFF);
    GoodixAppendChecksumU8Le(&cmd[2], 9);

    status = GoodixWrite(DeviceContext, GOODIX_FW_FLASH_CMD_REG_BRD, cmd, sizeof(cmd));
    if (!NT_SUCCESS(status)) {
        return status;
    }

    for (i = 0; i < 5; i++) {
        status = GoodixRead(DeviceContext, GOODIX_FW_FLASH_CMD_REG_BRD, ack, sizeof(ack));
        if (NT_SUCCESS(status) && ack[1] == GOODIX_FLASH_CMD_ACK_CHK_PASS) {
            break;
        }
        GoodixDelayMilliseconds(5);
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (ack[1] != GOODIX_FLASH_CMD_ACK_CHK_PASS) {
        return STATUS_CRC_ERROR;
    }

    GoodixDelayMilliseconds(50);
    for (i = 0; i < 20; i++) {
        status = GoodixRead(DeviceContext, GOODIX_FW_FLASH_CMD_REG_BRD, ack, sizeof(ack));
        if (NT_SUCCESS(status) &&
            ack[1] == GOODIX_FLASH_CMD_ACK_CHK_PASS &&
            ack[0] == GOODIX_FLASH_CMD_STATUS_WRITE_OK) {
            return STATUS_SUCCESS;
        }
        GoodixDelayMilliseconds(10);
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    switch (ack[0]) {
    case GOODIX_FLASH_CMD_STATUS_CHK_FAIL:
    case GOODIX_FLASH_CMD_STATUS_WRITE_ERR:
        return STATUS_RETRY;
    case GOODIX_FLASH_CMD_STATUS_ADDR_ERR:
        return STATUS_INVALID_ADDRESS;
    default:
        return STATUS_IO_DEVICE_ERROR;
    }
}

static
NTSTATUS
GoodixFlashPackage(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ UINT8 FwType,
    _In_reads_bytes_(PackageLength) const UINT8* Package,
    _In_ UINT32 FlashAddress,
    _In_ UINT16 PackageLength
    )
{
    NTSTATUS status;
    ULONG retry;

    for (retry = 0; retry < 2; retry++) {
        status = GoodixWriteLarge(
            DeviceContext,
            GOODIX_FW_ISP_BUFFER_REG_BRD,
            Package,
            PackageLength);
        if (!NT_SUCCESS(status)) {
            return status;
        }

        status = GoodixSendFlashCmd(DeviceContext, FwType, PackageLength, FlashAddress);
        if (status != STATUS_RETRY) {
            return status;
        }
    }

    return STATUS_RETRY;
}

static
NTSTATUS
GoodixFlashSubsystem(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ const GOODIX_FW_SUBSYS* Subsystem
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    UINT8* packet;
    ULONG offset = 0;
    ULONG remaining = Subsystem->Size;

    packet = (UINT8*)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        GOODIX_FW_PACKET_MAX_SIZE + 4,
        TOUCH_POOL_TAG);
    if (packet == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    while (remaining > 0) {
        ULONG chunk = (remaining > GOODIX_FW_PACKET_MAX_SIZE) ? GOODIX_FW_PACKET_MAX_SIZE : remaining;

        RtlCopyMemory(packet, Subsystem->Data + offset, chunk);
        GoodixAppendChecksumU16Le(packet, chunk);

        status = GoodixFlashPackage(
            DeviceContext,
            Subsystem->Type,
            packet,
            Subsystem->FlashAddress + offset,
            (UINT16)(chunk + 4));
        if (!NT_SUCCESS(status)) {
            break;
        }

        offset += chunk;
        remaining -= chunk;
    }

    ExFreePoolWithTag(packet, TOUCH_POOL_TAG);
    return status;
}

static
NTSTATUS
GoodixFlashFirmware(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _In_ const GOODIX_PARSED_FW* ParsedFirmware
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    ULONG i;

    for (i = 1; i < ParsedFirmware->SubsysNum; i++) {
        status = GoodixFlashSubsystem(DeviceContext, &ParsedFirmware->Subsys[i]);
        if (!NT_SUCCESS(status)) {
            return status;
        }
    }

    return STATUS_SUCCESS;
}

static
NTSTATUS
GoodixMaybeUpdateFirmware(
    _In_ PDEVICE_CONTEXT DeviceContext,
    _Inout_ PGOODIX_FW_VERSION Version
    )
{
    GOODIX_PARSED_FW parsedFirmware;
    NTSTATUS status;
    ULONG attempt;

    if (DeviceContext->ResetGpioIoTarget == WDF_NO_HANDLE) {
        return STATUS_NOT_SUPPORTED;
    }

    status = GoodixParseEmbeddedFirmware(&parsedFirmware);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (!GoodixFirmwareNeedsUpdate(Version, &parsedFirmware)) {
        return STATUS_SUCCESS;
    }

    TraceEvents(TRACE_LEVEL_WARNING, TRACE_DEVICE, "Goodix firmware update needed current_pid=%c%c%c%c%c%c%c%c current_vid=%02X%02X%02X%02X target_pid=%c%c%c%c%c%c%c%c target_vid=%02X%02X%02X%02X",
        Version->PatchPid[0], Version->PatchPid[1], Version->PatchPid[2], Version->PatchPid[3],
        Version->PatchPid[4], Version->PatchPid[5], Version->PatchPid[6], Version->PatchPid[7],
        Version->PatchVid[0], Version->PatchVid[1], Version->PatchVid[2], Version->PatchVid[3],
        parsedFirmware.FwPid[0], parsedFirmware.FwPid[1], parsedFirmware.FwPid[2], parsedFirmware.FwPid[3],
        parsedFirmware.FwPid[4], parsedFirmware.FwPid[5], parsedFirmware.FwPid[6], parsedFirmware.FwPid[7],
        parsedFirmware.FwVid[0], parsedFirmware.FwVid[1], parsedFirmware.FwVid[2], parsedFirmware.FwVid[3]);

    for (attempt = 0; attempt < 2; attempt++) {
        status = GoodixUpdatePrepare(DeviceContext, &parsedFirmware);
        if (!NT_SUCCESS(status)) {
            continue;
        }

        status = GoodixFlashFirmware(DeviceContext, &parsedFirmware);
        if (NT_SUCCESS(status)) {
            break;
        }
    }

    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixResetDevice(DeviceContext, 100);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    status = GoodixReadVersion(DeviceContext, Version);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (RtlCompareMemory(Version->PatchPid, parsedFirmware.FwPid, sizeof(parsedFirmware.FwPid)) != sizeof(parsedFirmware.FwPid) ||
        RtlCompareMemory(Version->PatchVid, parsedFirmware.FwVid, sizeof(parsedFirmware.FwVid)) != sizeof(parsedFirmware.FwVid)) {
        return STATUS_REVISION_MISMATCH;
    }

    DeviceContext->FirmwareUpdated = TRUE;
    return STATUS_SUCCESS;
}

NTSTATUS
SpbDeviceOpen(
    _In_  PDEVICE_CONTEXT  pDevice
)
{
    WDF_IO_TARGET_OPEN_PARAMS  openParams;
    NTSTATUS status;
    DECLARE_UNICODE_STRING_SIZE(DevicePath, RESOURCE_HUB_PATH_SIZE);
    RESOURCE_HUB_CREATE_PATH_FROM_ID(
        &DevicePath,
        pDevice->PeripheralId.LowPart,
        pDevice->PeripheralId.HighPart);

    //
    // Open a handle to the SPB controller.
    //

    WDF_IO_TARGET_OPEN_PARAMS_INIT_OPEN_BY_NAME(
        &openParams,
        &DevicePath,
        (GENERIC_READ | GENERIC_WRITE));

    openParams.ShareAccess = 0;
    openParams.CreateDisposition = FILE_OPEN;
    openParams.FileAttributes = FILE_ATTRIBUTE_NORMAL;

    status = WdfIoTargetOpen(
        pDevice->SpbController,
        &openParams);

    if (!NT_SUCCESS(status))
    {
        return status;
    }

    pDevice->OnClose = FALSE;

    //enable interrupt
    if (pDevice->Interrupt != NULL) {
        WdfInterruptEnable(pDevice->Interrupt);
    }
    return STATUS_SUCCESS;
}

VOID
SpbDeviceClose(
    _In_  PDEVICE_CONTEXT  pDevice
)
{
    pDevice->OnClose = TRUE;
    WdfIoTargetClose(pDevice->SpbController);
}

NTSTATUS
SpbDeviceWrite(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ PVOID pInputBuffer,
    _In_ size_t inputBufferLength
)
{
    WDF_MEMORY_DESCRIPTOR  inMemoryDescriptor;
    WDF_REQUEST_SEND_OPTIONS requestSendOptions;
    ULONG_PTR  bytesWritten = (ULONG_PTR)NULL;
    NTSTATUS status;


    WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(&inMemoryDescriptor,
        pInputBuffer,
        (ULONG)inputBufferLength);
    GoodixInitRequestSendOptions(&requestSendOptions, GOODIX_SPB_XFER_TIMEOUT_MS);

    status = WdfIoTargetSendWriteSynchronously(
        pDevice->SpbController,
        NULL,
        &inMemoryDescriptor,
        NULL,
        &requestSendOptions,
        &bytesWritten
    );

    if (!NT_SUCCESS(status))
    {
    }

    return status;
}

NTSTATUS
SpbDeviceWriteRead(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ PVOID pInputBuffer,
    _In_ PVOID pOutputBuffer,
    _In_ size_t inputBufferLength,
    _In_ size_t outputBufferLength
)
{
    WDF_MEMORY_DESCRIPTOR  memoryDescriptor;
    WDF_REQUEST_SEND_OPTIONS requestSendOptions;
    NTSTATUS status;

    SPB_TRANSFER_LIST_AND_ENTRIES(2) seq;
    SPB_TRANSFER_LIST_INIT(&(seq.List), 2);

    {
        ULONG index = 0;
        seq.List.Transfers[index] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(
            SpbTransferDirectionToDevice,
            0,
            pInputBuffer,
            (ULONG)inputBufferLength
        );
        seq.List.Transfers[index + 1] = SPB_TRANSFER_LIST_ENTRY_INIT_SIMPLE(
            SpbTransferDirectionFromDevice,
            0,
            pOutputBuffer,
            (ULONG)outputBufferLength
        );
    }

    WDF_MEMORY_DESCRIPTOR_INIT_BUFFER(
        &memoryDescriptor,
        &seq,
        sizeof(seq)
    );
    GoodixInitRequestSendOptions(&requestSendOptions, GOODIX_SPB_XFER_TIMEOUT_MS);

    status = WdfIoTargetSendIoctlSynchronously(
        pDevice->SpbController,
        NULL,
        IOCTL_SPB_FULL_DUPLEX,
        &memoryDescriptor,
        NULL,
        &requestSendOptions,
        NULL
    );

    if (!NT_SUCCESS(status)) {
        TraceEvents(TRACE_LEVEL_ERROR, TRACE_DEVICE, "Failed to send IOCTL, NTSTATUS=0x%08lX", status);
    }

    return status;
}

NTSTATUS
GoodixReadVersion(
    _In_ PDEVICE_CONTEXT pDevice,
    _Out_ PGOODIX_FW_VERSION Version
    )
{
    NTSTATUS status = STATUS_SUCCESS;
    GOODIX_FW_VERSION version = { 0 };
    UINT8 retry;

    for (retry = 0; retry < 2; retry++) {
        status = GoodixRead(pDevice, GOODIX_FW_VERSION_ADDR, (UINT8*)&version, sizeof(version));
        if (NT_SUCCESS(status) && GoodixChecksumValidU8Le((const UINT8*)&version, sizeof(version))) {
            *Version = version;
            return STATUS_SUCCESS;
        }
        GoodixDelayMilliseconds(5);
    }

    return NT_SUCCESS(status) ? STATUS_CRC_ERROR : status;
}

NTSTATUS
GoodixReadIcInfo(
    _In_ PDEVICE_CONTEXT pDevice
    )
{
    NTSTATUS status = STATUS_UNSUCCESSFUL;
    UINT8* icInfoBuf;
    UINT8 retry;
    USHORT icInfoLength = 0;

    icInfoBuf = (UINT8*)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        GOODIX_IC_INFO_MAX_LEN,
        TOUCH_POOL_TAG);
    if (icInfoBuf == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    for (retry = 0; retry < 20; retry++) {
        status = GoodixRead(pDevice, GOODIX_IC_INFO_ADDR, (UINT8*)&icInfoLength, sizeof(icInfoLength));
        if (!NT_SUCCESS(status)) {
            GoodixDelayMilliseconds(10);
            continue;
        }

        icInfoLength = GoodixReadU16Le((const UINT8*)&icInfoLength);
        if (icInfoLength < 2 || icInfoLength > GOODIX_IC_INFO_MAX_LEN) {
            status = STATUS_INFO_LENGTH_MISMATCH;
            GoodixDelayMilliseconds(10);
            continue;
        }

        status = GoodixRead(pDevice, GOODIX_IC_INFO_ADDR, icInfoBuf, icInfoLength);
        if (!NT_SUCCESS(status)) {
            GoodixDelayMilliseconds(10);
            continue;
        }

        if (!GoodixChecksumValidU8Le(icInfoBuf, icInfoLength)) {
            status = STATUS_CRC_ERROR;
            GoodixDelayMilliseconds(10);
            continue;
        }

        {
            const UINT8* cur = icInfoBuf + 2;
            ULONG remaining = icInfoLength - 2;
            UINT8 activeScanRateNum;
            UINT8 mutualFreqNum;
            UINT8 selfTxFreqNum;
            UINT8 selfRxFreqNum;
            UINT8 stylusFreqNum;
            GOODIX_IC_INFO_MISC misc = { 0 };

            if (remaining < 16U + 10U + 5U) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }

            cur += 16U;
            remaining -= 16U;
            cur += 10U;
            remaining -= 10U;

            activeScanRateNum = cur[4];
            cur += 5U;
            remaining -= 5U;
            if (activeScanRateNum > 8U || remaining < (ULONG)activeScanRateNum * 2U + 1U) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }
            cur += (ULONG)activeScanRateNum * 2U;
            remaining -= (ULONG)activeScanRateNum * 2U;

            mutualFreqNum = *(cur++);
            remaining -= 1U;
            if (mutualFreqNum > 20U || remaining < (ULONG)mutualFreqNum * 2U + 1U) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }
            cur += (ULONG)mutualFreqNum * 2U;
            remaining -= (ULONG)mutualFreqNum * 2U;

            selfTxFreqNum = *(cur++);
            remaining -= 1U;
            if (selfTxFreqNum > 20U || remaining < (ULONG)selfTxFreqNum * 2U + 1U) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }
            cur += (ULONG)selfTxFreqNum * 2U;
            remaining -= (ULONG)selfTxFreqNum * 2U;

            selfRxFreqNum = *(cur++);
            remaining -= 1U;
            if (selfRxFreqNum > 20U || remaining < (ULONG)selfRxFreqNum * 2U + 1U) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }
            cur += (ULONG)selfRxFreqNum * 2U;
            remaining -= (ULONG)selfRxFreqNum * 2U;

            stylusFreqNum = *(cur++);
            remaining -= 1U;
            if (stylusFreqNum > 8U || remaining < (ULONG)stylusFreqNum * 2U + sizeof(misc)) {
                status = STATUS_INVALID_BUFFER_SIZE;
                break;
            }
            cur += (ULONG)stylusFreqNum * 2U;
            remaining -= (ULONG)stylusFreqNum * 2U;

            RtlCopyMemory(&misc, cur, sizeof(misc));
            misc.CmdAddress = GoodixReadU32Le((const UINT8*)&misc.CmdAddress);
            misc.CmdMaxLength = GoodixReadU16Le((const UINT8*)&misc.CmdMaxLength);
            misc.FwBufferAddress = GoodixReadU32Le((const UINT8*)&misc.FwBufferAddress);
            misc.FwBufferMaxLength = GoodixReadU16Le((const UINT8*)&misc.FwBufferMaxLength);
            misc.TouchDataAddress = GoodixReadU32Le((const UINT8*)&misc.TouchDataAddress);

            if (misc.CmdAddress == 0 || misc.TouchDataAddress == 0 || misc.FwBufferAddress == 0) {
                status = STATUS_INVALID_DEVICE_STATE;
                break;
            }

            pDevice->CommandAddress = misc.CmdAddress;
            pDevice->TouchDataAddress = misc.TouchDataAddress;
            pDevice->FwBufferAddress = misc.FwBufferAddress;
            pDevice->FwBufferMaxLength = misc.FwBufferMaxLength;
            pDevice->IcInfoValid = TRUE;

            TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
                "Goodix ic_info ok cmd=0x%08X touch=0x%08X fwbuf=0x%08X fwbuflen=%u",
                pDevice->CommandAddress,
                pDevice->TouchDataAddress,
                pDevice->FwBufferAddress,
                pDevice->FwBufferMaxLength);
            status = STATUS_SUCCESS;
            break;
        }
    }

    ExFreePoolWithTag(icInfoBuf, TOUCH_POOL_TAG);
    return status;
}

NTSTATUS
GoodixApplyEmbeddedConfig(
    _In_ PDEVICE_CONTEXT pDevice
    )
{
    GOODIX_CFG_PACKAGE_INFO packageInfo;
    NTSTATUS status;
    UINT8* verifyBuffer;

    if (pDevice->ConfigApplied) {
        return STATUS_SUCCESS;
    }

    if (!pDevice->IcInfoValid || pDevice->FwBufferAddress == 0 || pDevice->FwBufferMaxLength == 0) {
        return STATUS_INVALID_DEVICE_STATE;
    }

    status = GoodixFindConfigPackage(pDevice->SensorId, &packageInfo);
    if (!NT_SUCCESS(status)) {
        return status;
    }

    if (packageInfo.ConfigType != GOODIX_CFG_TYPE_NORMAL) {
        return STATUS_NOT_SUPPORTED;
    }

    if (packageInfo.ConfigLength > pDevice->FwBufferMaxLength) {
        return STATUS_BUFFER_TOO_SMALL;
    }

    verifyBuffer = (UINT8*)ExAllocatePool2(
        POOL_FLAG_NON_PAGED,
        packageInfo.ConfigLength,
        TOUCH_POOL_TAG);
    if (verifyBuffer == NULL) {
        return STATUS_INSUFFICIENT_RESOURCES;
    }

    TraceEvents(TRACE_LEVEL_INFORMATION, TRACE_DEVICE,
        "Applying embedded cfg sensor_id=%u selected_sensor_id=%u len=%u fw_pid=%c%c%c%c%c",
        pDevice->SensorId,
        packageInfo.SensorId,
        packageInfo.ConfigLength,
        g_GoodixEmbeddedFwPid[0], g_GoodixEmbeddedFwPid[1], g_GoodixEmbeddedFwPid[2],
        g_GoodixEmbeddedFwPid[3], g_GoodixEmbeddedFwPid[4]);

    status = GoodixSendConfigCommand(pDevice, GOODIX_CFG_CMD_START);
    if (!NT_SUCCESS(status)) {
        goto Exit;
    }

    status = GoodixWriteLarge(
        pDevice,
        pDevice->FwBufferAddress,
        packageInfo.ConfigData,
        packageInfo.ConfigLength);
    if (!NT_SUCCESS(status)) {
        goto SendExit;
    }

    GoodixDelayMilliseconds(2);

    status = GoodixReadLarge(
        pDevice,
        pDevice->FwBufferAddress,
        verifyBuffer,
        packageInfo.ConfigLength);
    if (!NT_SUCCESS(status)) {
        goto SendExit;
    }

    if (RtlCompareMemory(verifyBuffer, packageInfo.ConfigData, packageInfo.ConfigLength) != packageInfo.ConfigLength) {
        status = STATUS_DATA_ERROR;
        goto SendExit;
    }

    status = GoodixSendConfigCommand(pDevice, GOODIX_CFG_CMD_WRITE);
    if (NT_SUCCESS(status)) {
        pDevice->ConfigApplied = TRUE;
        GoodixDelayMilliseconds(100);
    }

SendExit:
    {
        NTSTATUS exitStatus = GoodixSendConfigCommand(pDevice, GOODIX_CFG_CMD_EXIT);
        if (NT_SUCCESS(status) && !NT_SUCCESS(exitStatus)) {
            status = exitStatus;
        }
    }

Exit:
    ExFreePoolWithTag(verifyBuffer, TOUCH_POOL_TAG);
    return status;
}

NTSTATUS
GoodixApplyReportRate(
    _In_ PDEVICE_CONTEXT pDevice,
    _In_ UINT8 ReportRateLevel
    )
{
    GOODIX_CMD_PACKET cmd = { 0 };
    NTSTATUS status = STATUS_SUCCESS;
    UINT8 requestedLevel = GoodixNormalizeReportRateLevel(ReportRateLevel);

    if (pDevice->ActiveReportRateLevel == requestedLevel) {
        pDevice->ReportRateLevel = requestedLevel;
        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "GoodixApplyReportRate skip level=%u already active",
            requestedLevel);
        return STATUS_SUCCESS;
    }

    TraceEvents(
        TRACE_LEVEL_INFORMATION,
        TRACE_DEVICE,
        "GoodixApplyReportRate begin requested=%u currentActive=%u",
        requestedLevel,
        (pDevice->ActiveReportRateLevel == 0xFF) ? 0xFF : pDevice->ActiveReportRateLevel);

    switch (requestedLevel) {
    case GOODIX_REPORT_RATE_120HZ:
        if (pDevice->ActiveReportRateLevel == GOODIX_REPORT_RATE_960HZ) {
            status = GoodixApplyReportRate(pDevice, GOODIX_REPORT_RATE_480HZ);
            if (!NT_SUCCESS(status)) {
                return status;
            }
            pDevice->ActiveReportRateLevel = GOODIX_REPORT_RATE_480HZ;
        }
        if (pDevice->ActiveReportRateLevel == GOODIX_REPORT_RATE_480HZ) {
            cmd.Length = 0x06;
            cmd.Command = 0xC0;
            cmd.Data[0] = 0x00;
            cmd.Data[1] = 0x00;
            cmd.Data[2] = 0xC6;
            status = GoodixSendCommand(pDevice, &cmd);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        }
        RtlZeroMemory(&cmd, sizeof(cmd));
        cmd.Length = 0x05;
        cmd.Command = 0x9D;
        cmd.Data[0] = 0x00;
        cmd.Data[1] = 0xA2;
        cmd.Data[2] = 0x00;
        status = GoodixSendCommand(pDevice, &cmd);
        break;

    case GOODIX_REPORT_RATE_240HZ:
        if (pDevice->ActiveReportRateLevel == GOODIX_REPORT_RATE_960HZ) {
            status = GoodixApplyReportRate(pDevice, GOODIX_REPORT_RATE_480HZ);
            if (!NT_SUCCESS(status)) {
                return status;
            }
            pDevice->ActiveReportRateLevel = GOODIX_REPORT_RATE_480HZ;
        }
        if (pDevice->ActiveReportRateLevel == GOODIX_REPORT_RATE_480HZ) {
            cmd.Length = 0x06;
            cmd.Command = 0xC0;
            cmd.Data[0] = 0x00;
            cmd.Data[1] = 0x00;
            cmd.Data[2] = 0xC6;
            status = GoodixSendCommand(pDevice, &cmd);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        }
        RtlZeroMemory(&cmd, sizeof(cmd));
        cmd.Length = 0x05;
        cmd.Command = 0x9D;
        cmd.Data[0] = 0x01;
        cmd.Data[1] = 0xA3;
        cmd.Data[2] = 0x00;
        status = GoodixSendCommand(pDevice, &cmd);
        break;

    case GOODIX_REPORT_RATE_480HZ:
        if (pDevice->ActiveReportRateLevel == GOODIX_REPORT_RATE_960HZ) {
            cmd.Length = 0x06;
            cmd.Command = 0xC1;
            cmd.Data[0] = 0x00;
            cmd.Data[1] = 0x00;
            cmd.Data[2] = 0xC7;
            status = GoodixSendCommand(pDevice, &cmd);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        } else {
            cmd.Length = 0x06;
            cmd.Command = 0xC0;
            cmd.Data[0] = 0x01;
            cmd.Data[1] = 0x00;
            cmd.Data[2] = 0xC7;
            status = GoodixSendCommand(pDevice, &cmd);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        }
        break;

    case GOODIX_REPORT_RATE_960HZ:
        if (pDevice->ActiveReportRateLevel != GOODIX_REPORT_RATE_480HZ) {
            cmd.Length = 0x06;
            cmd.Command = 0xC0;
            cmd.Data[0] = 0x01;
            cmd.Data[1] = 0x00;
            cmd.Data[2] = 0xC7;
            status = GoodixSendCommand(pDevice, &cmd);
            if (!NT_SUCCESS(status)) {
                return status;
            }
        }
        RtlZeroMemory(&cmd, sizeof(cmd));
        cmd.Length = 0x06;
        cmd.Command = 0xC1;
        cmd.Data[0] = 0x01;
        cmd.Data[1] = 0x00;
        cmd.Data[2] = 0xC8;
        status = GoodixSendCommand(pDevice, &cmd);
        break;

    default:
        return STATUS_NOT_SUPPORTED;
    }

    if (NT_SUCCESS(status)) {
        pDevice->ReportRateLevel = requestedLevel;
        pDevice->ActiveReportRateLevel = requestedLevel;
        TraceEvents(
            TRACE_LEVEL_INFORMATION,
            TRACE_DEVICE,
            "GoodixApplyReportRate success level=%u",
            requestedLevel);
    } else {
        TraceEvents(
            TRACE_LEVEL_WARNING,
            TRACE_DEVICE,
            "GoodixApplyReportRate failed level=%u status=0x%08X",
            requestedLevel,
            status);
    }

    return status;
}



NTSTATUS
ReadDescriptorFromRegistry(
    WDFDEVICE Device
)
/*++
Routine Description:
    Read HID report descriptor from registry
Arguments:
    device - pointer to a device object.
Return Value:
    NT status code.
--*/
{
    WDFKEY          hKey = NULL;
    NTSTATUS        status;
    UNICODE_STRING  xRevertName;
    UNICODE_STRING  yRevertName;
    UNICODE_STRING  xYExchangeName;
    UNICODE_STRING  xMinName;
    UNICODE_STRING  xMaxName;
    UNICODE_STRING  yMinName;
    UNICODE_STRING  yMaxName;
    UNICODE_STRING  reportRateName;
    PDEVICE_CONTEXT deviceContext;
    WDF_OBJECT_ATTRIBUTES   attributes;
    ULONG reportRateLevel = GOODIX_REPORT_RATE_240HZ;

    deviceContext = GetDeviceContext(Device);

    status = WdfDeviceOpenRegistryKey(Device,
        PLUGPLAY_REGKEY_DEVICE,
        KEY_READ,
        WDF_NO_OBJECT_ATTRIBUTES,
        &hKey);

    if (NT_SUCCESS(status)) {

        RtlInitUnicodeString(&xRevertName, L"XRevert");
        RtlInitUnicodeString(&yRevertName, L"YRevert");
        RtlInitUnicodeString(&xYExchangeName, L"XYExchange");
        RtlInitUnicodeString(&xMinName, L"XMin");
        RtlInitUnicodeString(&xMaxName, L"XMax");
        RtlInitUnicodeString(&yMinName, L"YMin");
        RtlInitUnicodeString(&yMaxName, L"YMax");
        RtlInitUnicodeString(&reportRateName, L"ReportRateLevel");

        WDF_OBJECT_ATTRIBUTES_INIT(&attributes);
        attributes.ParentObject = Device;

        status = WdfRegistryQueryULong(hKey, &xRevertName, &XRevert);
        status = WdfRegistryQueryULong(hKey, &yRevertName, &YRevert);
        status = WdfRegistryQueryULong(hKey, &xYExchangeName, &XYExchange);
        status = WdfRegistryQueryULong(hKey, &xMinName, &XMin);
        status = WdfRegistryQueryULong(hKey, &xMaxName, &XMax);
        status = WdfRegistryQueryULong(hKey, &yMinName, &YMin);
        status = WdfRegistryQueryULong(hKey, &yMaxName, &YMax);
        if (NT_SUCCESS(WdfRegistryQueryULong(hKey, &reportRateName, &reportRateLevel))) {
            deviceContext->ReportRateLevel = GoodixNormalizeReportRateLevel((UINT8)reportRateLevel);
        }

        WdfRegistryClose(hKey);
    }

    return status;
}

