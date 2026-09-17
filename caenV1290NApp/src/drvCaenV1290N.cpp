#include <devLib.h>
#include <epicsExport.h>
#include <iocsh.h>
#include <string.h>

#include "V1290N.hpp"
#include "drvCaenV1290N.hpp"

static void poll_thread_C(void* pPvt) {
    CaenV1290N* pCaenV1290N = (CaenV1290N*)pPvt;
    pCaenV1290N->poll();
}

static void acquisition_worker_C(void* pPvt) {
    CaenV1290N* pCaenV1290N = (CaenV1290N*)pPvt;
    pCaenV1290N->acquisition_worker();
}

// TODO: make configurable
const double poll_period_sec = 0.5;

const int ASYN_INTERFACE_MASK =
    asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynInt32ArrayMask | asynDrvUserMask;
const int ASYN_INTERRUPT_MASK = asynInt32Mask | asynUInt32DigitalMask | asynFloat64Mask | asynInt32ArrayMask;

CaenV1290N::CaenV1290N(const char* portName, int baseAddress)
    : asynPortDriver(portName, MAX_CHANNELS, ASYN_INTERFACE_MASK, ASYN_INTERRUPT_MASK,
                     ASYN_MULTIDEVICE, 1, 0, 0),
      acquire_event_(epicsEventCreate(epicsEventEmpty)) {

    // // initialize
    volatile void* ptr;
    const size_t EXTENT = 0x10000;
    if (devRegisterAddress("CAEN_V1290N", atVMEA32, baseAddress, EXTENT, &ptr)) {
        printf("ERROR: devRegisterAddress failed. Cannot initialize board.\n");
        return;
    }
    base = (volatile uint8_t*)ptr;

    // Read the Firmware Revision Register
    uint16_t rev;
    readD16(Register::FirmwareRev, rev);
    int major = (rev >> 4) & 0x0F;
    int minor = rev & 0x0F;
    printf("V1290 Firmware Revision: %d.%d (Raw: 0x%04X)\n\n", major, minor, rev);

    createParam("ACQUISITION_MODE", asynParamInt32, &acquisitionModeId_);
    createParam("EDGE_DETECT_MODE", asynParamInt32, &edgeDetectModeId_);
    createParam("ENABLE_PATTERN", asynParamUInt32Digital, &enablePatternId_);
    createParam("STATUS", asynParamUInt32Digital, &statusId_);
    createParam("CONTROL", asynParamUInt32Digital, &controlId_);
    createParam("EVENTS_STORED", asynParamInt32, &eventsStoredId_);
    createParam("WINDOW_WIDTH", asynParamInt32, &windowWidthId_);
    createParam("WINDOW_OFFSET", asynParamInt32, &windowOffsetId_);
    createParam("SOFTWARE_CLEAR", asynParamInt32, &softwareClearId_);
    createParam("SOFTWARE_TRIGGER", asynParamInt32, &softwareTriggerId_);
    createParam("TDC_HEADER_TRAILER", asynParamInt32, &tdcHeaderTrailerId_);
    createParam("TESTREG", asynParamInt32, &testregId_);
    createParam("DUMMY16", asynParamInt32, &dummy16Id_);
    createParam("DUMMY32", asynParamInt32, &dummy32Id_);
    createParam("DEV_PARAM_STR", asynParamInt32, &devParamId_);
    createParam("ACQUIRE", asynParamInt32, &acquireId_);
    createParam("NUM_CAPTURED", asynParamInt32, &numCapturedId_);
    createParam("RAW_DATA", asynParamInt32Array, &rawDataId_);

    setIntegerParam(acquireId_, 0);
    setIntegerParam(numCapturedId_, 0);

    epicsThreadCreate("CaenV1290NPoller", epicsThreadPriorityLow,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)poll_thread_C, this);

    epicsThreadCreate("CaenV1290NAcquisitionWorker", epicsThreadPriorityHigh,
                      epicsThreadGetStackSize(epicsThreadStackMedium), (EPICSTHREADFUNC)acquisition_worker_C,
                      this);
}

bool CaenV1290N::wait_micro_handshake(uint16_t mask, uint16_t timeout) {
    while (true) {
        uint16_t hs;
        readD16(Register::MicroHandshake, hs);
        if ((hs & mask) == 0 && (timeout > 0)) {
            epicsThreadSleep(0.001);
            timeout--;
        } else {
            break;
        }
    }
    return (timeout > 0) ? true : false;
};

bool CaenV1290N::write_micro(uint16_t opcode, uint16_t val) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
        printf("write_micro: wait_micro_handshake returned error");
        return false;
    }
    writeD16(Register::Micro, opcode);

    if (!wait_micro_handshake(Handshake::WriteOk)) {
        printf("write_micro: wait_micro_handshake returned error");
        return false;
    }
    writeD16(Register::Micro, val);
    return true;
}

bool CaenV1290N::write_micro(uint16_t opcode) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
        printf("write_micro: wait_micro_handshake returned error");
        return false;
    }
    writeD16(Register::Micro, opcode);
    return true;
}

bool CaenV1290N::read_micro(uint16_t opcode, uint16_t& value) {
    if (!wait_micro_handshake(Handshake::WriteOk)) {
        printf("read_micro: wait_micro_handshake returned error");
        return false;
    }
    writeD16(Register::Micro, opcode);

    if (!wait_micro_handshake(Handshake::ReadOk)) {
        printf("read_micro: wait_micro_handshake returned error");
        return false;
    }
    readD16(Register::Micro, value);

    return true;
}

asynStatus CaenV1290N::writeUInt32Digital(asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == enablePatternId_) {
        if (!write_micro(Opcode::WriteEnablePattern, value)) {
            asyn_status = asynError;
        }
    } else if (function == controlId_) {
        writeD16(Register::Control, value);
    }

    if (asyn_status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::writeUInt32Digital\n");
    }

    return asyn_status;
}

asynStatus CaenV1290N::readUInt32Digital(asynUser* pasynUser, epicsUInt32* value, epicsUInt32 mask) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    uint16_t v16 = 0;
    if (function == enablePatternId_) {
        if (!read_micro(Opcode::ReadEnablePattern, v16)) {
            asyn_status = asynError;
        }
        *value = v16;
    } else if (function == controlId_) {
        readD16(Register::Control, v16);
        *value = v16;
    } else {
        asyn_status = asynPortDriver::readUInt32Digital(pasynUser, value, mask);
    }

    if (asyn_status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readUInt32Digital\n");
    }

    return asyn_status;
}

asynStatus CaenV1290N::readInt32(asynUser* pasynUser, epicsInt32* value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    uint16_t v16 = 0;
    if (function == edgeDetectModeId_) {
        if (!read_micro(Opcode::ReadEdgeDetectionMode, v16)) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "read_micro error: ReadEdgeDetectionMode\n");
            asyn_status = asynError;
        };
        *value = v16;
    } else if (function == acquisitionModeId_) {
        if (!read_micro(Opcode::ReadAcquisitionMode, v16)) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "read_micro error: ReadAcquisitionMode\n");
            asyn_status = asynError;
        };
        *value = v16;
    } else if (function == tdcHeaderTrailerId_) {
        if (!read_micro(Opcode::TDCHeaderTrailerStatus, v16)) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR, "read_micro error: TDCHeaderTrailerStatus\n");
            asyn_status = asynError;
        };
        *value = v16;
    } else if (function == testregId_) {
        uint32_t v32 = 0;
        readD32(Register::TestReg, v32);
        *value = v32;
    } else {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in the base readInt32, reason = %d\n", function);
        asyn_status = asynPortDriver::readInt32(pasynUser, value);
    }

    if (asyn_status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readInt32\n");
    }

    return asynSuccess;
}

asynStatus CaenV1290N::readInt32Array(asynUser* pasynUser, epicsInt32* value, size_t nElements, size_t* nIn) {
    if (pasynUser->reason != rawDataId_) {
        return asynPortDriver::readInt32Array(pasynUser, value, nElements, nIn);
    }

    int num_captured = 0;
    getIntegerParam(numCapturedId_, &num_captured);
    size_t count = static_cast<size_t>(num_captured);
    if (count > nElements) {
        count = nElements;
    }

    memcpy(value, capture_buffer_, count * sizeof(*value));
    *nIn = count;
    return asynSuccess;
}

asynStatus CaenV1290N::writeInt32(asynUser* pasynUser, epicsInt32 value) {
    const int function = pasynUser->reason;
    asynStatus asyn_status = asynSuccess;

    if (function == edgeDetectModeId_) {
        if (!write_micro(Opcode::SetEdgeDetectionMode, value)) {
            asyn_status = asynError;
        }
    } else if (function == acquisitionModeId_) {
        if (!write_micro(value == 0 ? Opcode::SetContinuous : Opcode::SetTriggerMatch)) {
            asyn_status = asynError;
        }
    } else if (function == windowWidthId_) {
        if (!write_micro(Opcode::SetWindowWidth, value)) {
            asyn_status = asynError;
        }
    } else if (function == windowOffsetId_) {
        if (!write_micro(Opcode::SetWindowOffset, value)) {
            asyn_status = asynError;
        }
    } else if (function == tdcHeaderTrailerId_) {
        if (!write_micro(value == 0 ? Opcode::DisableTDCHeaderTrailer : Opcode::EnableTDCHeaderTrailer)) {
            asyn_status = asynError;
        }
    } else if (function == softwareClearId_) {
        writeD16(Register::SwClear, value);
    } else if (function == softwareTriggerId_) {
        writeD16(Register::SwTrigger, value);
    } else if (function == dummy16Id_) {
        writeD16(Register::Dummy16, value);
    } else if (function == dummy32Id_) {
        writeD32(Register::Dummy32, value);
    } else if (function == testregId_) {
        writeD32(Register::TestReg, value);
    } else if (function == acquireId_) {
        int acquiring = 0;
        getIntegerParam(acquireId_, &acquiring);
        value = value ? 1 : 0;
        setIntegerParam(acquireId_, value);
        if (value && !acquiring) {
            setIntegerParam(numCapturedId_, 0);
            epicsEventSignal(acquire_event_);
        }
        callParamCallbacks();
    }

    else if (function == devParamId_) {
        // attempt to read FIFO output buffer
        uint32_t out_buff = 0;
        readD32(Register::OutBuf, out_buff);
        printf("out buff = 0x%X\n", out_buff);
    }

    if (asyn_status) {
        asynPrint(pasynUser, ASYN_TRACE_ERROR, "Error in CaenV1290N::readInt32\n");
    }

    return asynSuccess;
}

void CaenV1290N::acquisition_worker() {
    const size_t max_batch = 256;

    while (true) {
        epicsEventWait(acquire_event_);
        size_t captured = 0;

        while (captured < CAPTURE_SIZE) {
            lock();

            int acquire = 0;
            getIntegerParam(acquireId_, &acquire);
            if (!acquire) {
                unlock();
                break;
            }

            uint16_t status = 0;
            readD16(Register::Status, status);

            size_t batch_count = 0;
            while ((status & Status::DataReady) && captured < CAPTURE_SIZE && batch_count < max_batch) {
                uint32_t word = 0;
                readD32(Register::OutBuf, word);
                capture_buffer_[captured++] = static_cast<epicsInt32>(word);
                batch_count++;
                readD16(Register::Status, status);
            }

            setIntegerParam(numCapturedId_, static_cast<int>(captured));
            unlock();

            if (!(status & Status::DataReady)) {
                epicsThreadSleep(0.001);
            }
        }

        lock();
        setIntegerParam(acquireId_, 0);
        setIntegerParam(numCapturedId_, static_cast<int>(captured));
        doCallbacksInt32Array(capture_buffer_, captured, rawDataId_, 0);
        callParamCallbacks();
        unlock();
    }
}

void CaenV1290N::poll() {
    while (true) {
        uint16_t val16 = 0;
        uint32_t val32 = 0;

        lock();

        // Read status register
        val16 = 0;
        readD16(Register::Status, val16);
        setUIntDigitalParam(statusId_, val16, 0xFFFF);

        // read dummy registers for testing
        val16 = 0;
        readD16(Register::Dummy16, val16);
        setIntegerParam(dummy16Id_, val16);

        val32 = 0;
        readD32(Register::Dummy32, val32);
        setIntegerParam(dummy32Id_, val32);

        val16 = 0;
        readD16(Register::EventStored, val16);
        setIntegerParam(eventsStoredId_, val16);

        callParamCallbacks();
        unlock();
        epicsThreadSleep(poll_period_sec);
    }
}

extern "C" int initCaenV1290N(const char* portName, int baseAddress) {
    new CaenV1290N(portName, baseAddress);
    return (asynSuccess);
}

static const iocshArg initArg0 = {"Port name", iocshArgString};
static const iocshArg initArg1 = {"Base Address", iocshArgInt};
static const iocshArg* const initArgs[2] = {&initArg0, &initArg1};
static const iocshFuncDef initFuncDef = {"initCAEN_V1290N", 2, initArgs};
static void initCallFunc(const iocshArgBuf* args) { initCaenV1290N(args[0].sval, args[1].ival); }

void drvCaenV1290NRegister(void) { iocshRegister(&initFuncDef, initCallFunc); }

extern "C" {
epicsExportRegistrar(drvCaenV1290NRegister);
}
