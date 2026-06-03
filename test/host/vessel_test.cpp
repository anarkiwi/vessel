// Host-based unit tests for Vessel.
//
// We compile the *real* firmware by #including vessel.ino into this test
// translation unit, after the host mocks (Arduino.h, USB-MIDI.h) and the
// ARCH_HOST branches of pins.h/platform.h/CRDigitalPin.h have replaced the
// hardware seams with in-memory simulation (host_sim). Tests then drive the
// firmware's own functions and assert on what it parses and emits.

#include "USB-MIDI.h"
#include "host_sim.h"
#include <gtest/gtest.h>

// Definition for the extern declared in the USB-MIDI mock.
HostMidiUSB MidiUSB;

// The Arduino IDE auto-generates function prototypes when it preprocesses a
// .ino; a plain C++ compile does not, so a few functions are referenced before
// their definition. Declare them here exactly as the IDE would.
bool statusMasked(byte status);
bool channelMasked(byte channel);
bool commandMasked(byte command, byte channel);
void readCmdByte();
void readCmdData();
void runCmd();
void writeByte();

// Pull in the entire firmware. This brings setup()/loop() and every parsing
// function into this TU. setup()/loop() are simply never invoked as such.
#include "../../vessel.ino"

namespace {

// Feed bytes as if the C64 wrote them to the user port, then run the byte I/O
// state machine once per byte (the firmware consumes exactly one byte per ISR
// dispatch).
void feedC64(std::initializer_list<uint8_t> bytes) {
  for (uint8_t b : bytes) {
    hostsim::portIn.push_back(b);
  }
  for (size_t i = 0; i < bytes.size(); ++i) {
    isrMode();
  }
}

// Feed bytes on the MIDI DIN input and let the firmware parse them. drainOutBuf
// reads one byte from the UART into FakeSerial per call, then parses it on the
// next call, so a generous number of pumps fully drains the message.
void pumpMidiIn(std::initializer_list<uint8_t> bytes) {
  for (uint8_t b : bytes) {
    hostsim::uartIn.push_back(b);
  }
  for (size_t i = 0; i < bytes.size() * 2 + 4; ++i) {
    drainOutBuf();
  }
}

class VesselTest : public ::testing::Test {
protected:
  void SetUp() override {
    hostsim::reset();
    // PA2 high == C64 in input mode, so setup()'s final spin-wait returns.
    hostsim::pinLevel[C64_PA2] = true;
    setup();
    isrMode = readByte;
  }
};

// ---------------------------------------------------------------------------
// Command parser state machine (C64 -> Vessel).
// ---------------------------------------------------------------------------

TEST_F(VesselTest, ChannelMaskCommandSetsMask) {
  feedC64({vesselCmd, 0x05, 0x12, 0x34});
  EXPECT_EQ(vesselConfig.receiveChannelMask, 0x1234);
}

TEST_F(VesselTest, StatusMaskCommandSetsMask) {
  feedC64({vesselCmd, 0x06, 0xab, 0xcd});
  EXPECT_EQ(vesselConfig.receiveStatusMask, 0xabcd);
}

TEST_F(VesselTest, ConfigFlagsCommandSetsAllFlags) {
  feedC64({vesselCmd, 0x04, 0x0f});
  EXPECT_TRUE(vesselConfig.nmiEnabled);
  EXPECT_TRUE(vesselConfig.transparent);
  EXPECT_TRUE(vesselConfig.nmiStatusOnlyEnabled);
  EXPECT_TRUE(MIDI.getThruState());
}

TEST_F(VesselTest, ConfigFlagsCommandClearsThru) {
  feedC64({vesselCmd, 0x04, 0x02}); // thru on
  EXPECT_TRUE(MIDI.getThruState());
  feedC64({vesselCmd, 0x04, 0x00}); // thru off
  EXPECT_FALSE(MIDI.getThruState());
}

TEST_F(VesselTest, CommandMaskCommandStoresPerChannelMask) {
  // README example: 0x79 -> channel nibble 9 (channel 10), command bits 0x7.
  feedC64({vesselCmd, 0x07, 0x79});
  EXPECT_EQ(receiveCommandMask[9], 0x07);
}

TEST_F(VesselTest, VersionCommandEmitsVersionString) {
  feedC64({vesselCmd, 0x03});
  ASSERT_EQ(vesselConfig.pendingOut, sizeof(versionStr));
  for (size_t i = 0; i < sizeof(versionStr); ++i) {
    EXPECT_EQ(outBuf[i], (byte)versionStr[i]) << "at index " << i;
  }
}

TEST_F(VesselTest, ResetCommandClearsConfig) {
  feedC64({vesselCmd, 0x05, 0xff, 0xff}); // set a channel mask
  ASSERT_EQ(vesselConfig.receiveChannelMask, 0xffff);
  feedC64({vesselCmd, 0x00}); // reset
  EXPECT_EQ(vesselConfig.receiveChannelMask, 0x0000);
  EXPECT_FALSE(vesselConfig.nmiEnabled);
}

TEST_F(VesselTest, UnknownCommandIsIgnoredAndParserRecovers) {
  feedC64({vesselCmd, 0x42}); // 0x42 > maxCmd
  EXPECT_EQ(isrMode, readByte);
  // Parser still works afterwards.
  feedC64({vesselCmd, 0x05, 0x0a, 0x0b});
  EXPECT_EQ(vesselConfig.receiveChannelMask, 0x0a0b);
}

TEST_F(VesselTest, NonCommandBytePassesToInputBuffer) {
  feedC64({0x90}); // not the command marker
  EXPECT_EQ(inBufReadPtr, 1);
  EXPECT_EQ(inBuf[1], 0x90);
}

// ---------------------------------------------------------------------------
// MIDI DIN parsing + filtering (MIDI -> C64).
// ---------------------------------------------------------------------------

TEST_F(VesselTest, NoteOnFilteredOutByDefault) {
  // Default config masks everything; nothing should be forwarded.
  pumpMidiIn({0x99, 0x3c, 0x40}); // NoteOn ch10
  EXPECT_EQ(vesselConfig.pendingOut, 0);
}

TEST_F(VesselTest, NoteOnForwardedWhenChannelAndCommandEnabled) {
  feedC64({vesselCmd, 0x05, 0x02, 0x00}); // channel mask: bit9 -> channel 10
  feedC64({vesselCmd, 0x07, 0x19});       // command mask: note-on on channel 10
  pumpMidiIn({0x99, 0x3c, 0x40});         // NoteOn ch10 note 0x3c vel 0x40
  ASSERT_EQ(vesselConfig.pendingOut, 3);
  EXPECT_EQ(outBuf[0], 0x99);
  EXPECT_EQ(outBuf[1], 0x3c);
  EXPECT_EQ(outBuf[2], 0x40);
}

TEST_F(VesselTest, NoteOnOnWrongChannelIsFiltered) {
  feedC64({vesselCmd, 0x05, 0x02, 0x00}); // enable channel 10 only
  feedC64({vesselCmd, 0x07, 0x19});
  pumpMidiIn({0x90, 0x3c, 0x40}); // NoteOn ch1
  EXPECT_EQ(vesselConfig.pendingOut, 0);
}

TEST_F(VesselTest, RealtimeClockForwardedWhenStatusEnabled) {
  feedC64({vesselCmd, 0x06, 0x01, 0x00}); // status mask: bit8 -> Clock (0xF8)
  pumpMidiIn({0xf8});
  ASSERT_GE(vesselConfig.pendingOut, 1);
  EXPECT_EQ(outBuf[0], 0xf8);
}

// ---------------------------------------------------------------------------
// USB-MIDI packet decoding (vesselusbmidi.h).
// ---------------------------------------------------------------------------

TEST_F(VesselTest, TransparentUsbMidiDecodesThreeByteMessage) {
  // CIN 9 (NoteOn) -> 3 data bytes, written straight through in transparent
  // mode.
  MidiUSB.rx.push_back(midiEventPacket_t{0x09, 0x99, 0x3c, 0x40});
  EXPECT_TRUE(transparentUsbMidiRx());
  ASSERT_EQ(vesselConfig.pendingOut, 3);
  EXPECT_EQ(outBuf[0], 0x99);
  EXPECT_EQ(outBuf[1], 0x3c);
  EXPECT_EQ(outBuf[2], 0x40);
}

TEST_F(VesselTest, TransparentUsbMidiDecodesSingleByteSysExEnd) {
  // CIN 5 -> single-byte SysEx end; cin2Len reports 0 and the firmware
  // special-cases it to emit exactly one byte.
  MidiUSB.rx.push_back(midiEventPacket_t{0x05, 0xf7, 0x00, 0x00});
  EXPECT_TRUE(transparentUsbMidiRx());
  ASSERT_EQ(vesselConfig.pendingOut, 1);
  EXPECT_EQ(outBuf[0], 0xf7);
}

TEST_F(VesselTest, UsbMidiReturnsFalseWhenNoPackets) {
  EXPECT_FALSE(transparentUsbMidiRx());
}

} // namespace
