#pragma once
#include <asynPortDriver.h>
#include <devLib.h>
#include <epicsEvent.h>
#include <epicsMMIO.h>
#include <stdint.h>

class CaenV1290N : public asynPortDriver {
  public:
    CaenV1290N(const char* portName, int baseAddress);
    virtual void poll();
    virtual void acquisition_worker();
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    virtual asynStatus readInt32(asynUser* pasynUser, epicsInt32* value);
    virtual asynStatus readInt32Array(asynUser* pasynUser, epicsInt32* value,
                                      size_t nElements, size_t* nIn);
    virtual asynStatus readUInt32Digital(asynUser* pasynUser, epicsUInt32* value, epicsUInt32 mask);
    virtual asynStatus writeUInt32Digital(asynUser* pasynUser, epicsUInt32 value, epicsUInt32 mask);

  private:
    static const size_t CAPTURE_SIZE = 4096;

    // this is a "trick" since adding an offset like 0x1000 to a pointer to uint8_t moves 4 bytes
    volatile uint8_t* base;
    epicsEventId acquire_event_;
    epicsInt32 capture_buffer_[CAPTURE_SIZE];

    /// \brief Continually tests microcontroller handshake until true, or timeout.
    ///
    /// \param mask The mask to test handshake register with.
    /// \param timeout Timeout in milliseconds.
    /// \return True on success, false on error or timeout/
    bool wait_micro_handshake(uint16_t mask, uint16_t timeout = 1000);

    /// \brief Writes given opcode, followed by given value to the micro register.
    ///
    /// \param opcode The opcode to write.
    /// \param val The value to write immediately after the opcode.
    /// \return True on success, false on error or timeout.
    bool write_micro(uint16_t opcode, uint16_t val);

    /// \brief Writes given opcode to the micro register.
    ///
    /// \param opcode The opcode to write.
    /// \return True on success, false on error or timeout.
    bool write_micro(uint16_t opcode);

    /// \brief Reads data stored in micro register after writing given opcode
    ///
    /// \param opcode The opcode to write.
    /// \param value Reference to the value to store the data in.
    /// \return True on success, false on error or timeout.
    bool read_micro(uint16_t opcode, uint16_t& value);

    /// \brief Performs a safe D16 VME bus write of the value to the offset
    /// \param offset The offset from the base address to write to
    /// \param value The value to write
    /// \return True on success, false on error
    bool safe_writeD16(uint16_t offset, uint16_t value) {
        return !devWriteProbe(sizeof(uint16_t), base + offset, &value);
    }

    /// \brief Performs a safe D16 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool safe_readD16(uint16_t offset, uint16_t& value) {
        return !devReadProbe(sizeof(uint16_t), base + offset, &value);
    }

    /// \brief Performs a safe D32 VME bus write of the value to the offset
    /// \param offset The offset from the base address to write to
    /// \param value The value to write
    /// \return True on success, false on error
    bool safe_writeD32(uint32_t offset, uint32_t value) {
        return !devWriteProbe(sizeof(uint32_t), base + offset, &value);
    }

    /// \brief Performs a safe D32 VME bus read of the offset location
    /// \param offset The offset from the base address to write to
    /// \param value The value to store the read data
    /// \return True on success, false on error
    bool safe_readD32(uint32_t offset, uint32_t& value) {
        return !devReadProbe(sizeof(uint32_t), base + offset, &value);
    }


    // Unprotected direct memory access reads/writes

    void writeD16(uint16_t offset, uint16_t value) {
        *(volatile uint16_t*)(base + offset) = value;
    }
    void readD16(uint16_t offset, uint16_t& value) {
        value = *(volatile uint16_t*)(base + offset);
    }

    void writeD32(uint32_t offset, uint32_t value) {
        *(volatile uint32_t*)(base + offset) = value;
    }
    void readD32(uint32_t offset, uint32_t& value) {
        value = *(volatile uint32_t*)(base + offset);
    }

  protected:
    int acquisitionModeId_;
    int edgeDetectModeId_;
    int enablePatternId_;
    int windowWidthId_;
    int windowOffsetId_;
    int extraSearchId_;
    int rejectMarginId_;
    int triggerTimeSubId_;
    int triggerConfigId_;
    int softwareClearId_;
    int softwareTriggerId_;
    int tdcHeaderTrailerId_;
    int statusId_;
    int eventsStoredId_;
    int controlId_;
    int testregId_;
    int dummy16Id_;
    int dummy32Id_;
    int devParamId_;
    int acquireId_;
    int numCapturedId_;
    int rawDataId_;
};
