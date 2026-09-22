#pragma once

#include <stdint.h>

#define MAX_CHANNELS 16

namespace Register {
// Configuration & Status Registers
static const uint16_t OutBuf = 0x0000;          // D32, R, Output Buffer
static const uint16_t Control = 0x1000;         // D16, R/W, Control Register
static const uint16_t Status = 0x1002;          // D16, R, Status Register
static const uint16_t IntLevel = 0x100A;        // D16, R/W, uint16_terrupt Level
static const uint16_t IntVector = 0x100C;       // D16, R/W, uint16_terrupt Vector
static const uint16_t GeoAddr = 0x100E;         // D16, R/W, GEO Address
static const uint16_t McstCbltAddr = 0x1010;    // D16, R/W, MCST/CBLT Address
static const uint16_t McstCbltCtrl = 0x1012;    // D16, R/W, MCST/CBLT Control
static const uint16_t ModReset = 0x1014;        // D16, W, Module Reset
static const uint16_t SwClear = 0x1016;         // D16, W, Software Clear
static const uint16_t SwEventReset = 0x1018;    // D16, W, Software Event Reset
static const uint16_t SwTrigger = 0x101A;       // D16, W, Software Trigger
static const uint16_t EventCounter = 0x101C;    // D32, R, Event Counter
static const uint16_t EventStored = 0x1020;     // D16, R, Event Stored
static const uint16_t AlmostFullLevel = 0x1022; // D16, R/W, Almost Full Level
static const uint16_t BltEventNumber = 0x1024;  // D16, R/W, BLT Event Number
static const uint16_t FirmwareRev = 0x1026;     // D16, R, Firmware Revision
static const uint16_t TestReg = 0x1028;         // D32, R/W, Test Register
static const uint16_t OutProgControl = 0x102C;  // D16, R/W, Output Prog. Control
static const uint16_t Micro = 0x102E;           // D16, R/W, Micro Register (Opcodes)
static const uint16_t MicroHandshake = 0x1030;  // D16, R, Micro Handshake
static const uint16_t SelectFlash = 0x1032;     // D16, R/W, Select Flash
static const uint16_t FlashRW = 0x1034;         // D16, R/W, Flash R/W
static const uint16_t SramPage = 0x1036;        // D16, R/W, SRAM Page
static const uint16_t EventFifo = 0x1038;       // D32, R, Event FIFO
static const uint16_t EventFifoStored = 0x103C; // D16, R, Event FIFO Stored
static const uint16_t EventFifoStatus = 0x103E; // D16, R, Event FIFO Status
static const uint16_t Dummy32 = 0x1200;         // D32, R/W, Dummy 32-bit
static const uint16_t Dummy16 = 0x1204;         // D16, R/W, Dummy 16-bit
} // namespace Register

namespace Control {
static const uint16_t BerrEn = (1 << 0);         // Bus Error enable for BLT
static const uint16_t Term = (1 << 1);           // Software termination status
static const uint16_t TermSw = (1 << 2);         // 1=SW Term, 0=HW Term
static const uint16_t EmptyEvent = (1 << 3);     // Enable Header/Trailer for empty events
static const uint16_t Align64 = (1 << 4);        // 64-bit alignment (dummy word)
static const uint16_t CompEnable = (1 << 5);     // INL compensation enable
static const uint16_t TestFifoEnable = (1 << 6); // Output Buffer test mode
static const uint16_t ReadCompEnable = (1 << 7); // Read compensation SRAM
static const uint16_t EventFifoEn = (1 << 8);    // Event FIFO enable
static const uint16_t EtttEnable = (1 << 9);     // Extended Trigger Time Tag enable
static const uint16_t Bus16mbAddrEn = (1 << 10); // 16MB address range for MEB
} // namespace Control

namespace Status {
static const uint16_t DataReady = (1 << 0);    // Data/Event ready in Buffer
static const uint16_t AlmostFull = (1 << 1);   // Almost Full level reached
static const uint16_t Full = (1 << 2);         // Output Buffer full
static const uint16_t TrgMatchMode = (1 << 3); // 1=Trigger Match, 0=Continuous
static const uint16_t HeaderEn = (1 << 4);     // TDC Header/Trailer enabled
static const uint16_t TermOn = (1 << 5);       // Termination status
static const uint16_t Error0 = (1 << 6);       // TDC 0 Error
static const uint16_t Error1 = (1 << 7);       // TDC 1 Error
static const uint16_t Error2 = (1 << 8);       // TDC 2 Error (V1290A only)
static const uint16_t Error3 = (1 << 9);       // TDC 3 Error (V1290A only)
static const uint16_t BerrFlag = (1 << 10);    // VME Bus Error occurred
static const uint16_t Purged = (1 << 11);      // Board purged
static const uint16_t TriggerLost = (1 << 15); // Trigger overlap/lost
} // namespace Status

namespace Handshake {
static const uint16_t WriteOk = (1 << 0); // Ready for Micro write/opcode
static const uint16_t ReadOk = (1 << 1);  // Data ready for Micro read
} // namespace Handshake

namespace Opcode {
static const uint16_t SetTriggerMatch = 0x0000;
static const uint16_t SetContinuous = 0x0100;
static const uint16_t ReadAcquisitionMode = 0x0200;
static const uint16_t SetEdgeDetectionMode = 0x2200;
static const uint16_t ReadEdgeDetectionMode = 0x2300;
static const uint16_t EnableTDCHeaderTrailer = 0x3000;
static const uint16_t DisableTDCHeaderTrailer = 0x3100;
static const uint16_t TDCHeaderTrailerStatus = 0x3200;
static const uint16_t WriteEnablePattern = 0x4400;
static const uint16_t ReadEnablePattern = 0x4500;
static const uint16_t SetWindowWidth = 0x1000;
static const uint16_t SetWindowOffset = 0x1100;
static const uint16_t ReadTriggerConfig = 0x1600;
} // namespace Opcode
