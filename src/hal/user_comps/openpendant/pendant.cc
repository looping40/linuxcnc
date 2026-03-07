/*
   Copyright (C) 2018 Raoul Rubien (github.com/rubienr)
   Updated for OpenPendant V2 protocol.

   This program is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2 of the License, or (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with the program; if not, write to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
   02111-1307 USA.
 */

#include "pendant.h"

// system includes
#include <iostream>
#include <iomanip>
#include <string.h>

// local includes
#include "./hal.h"
#include "./usb.h"

using std::endl;

namespace OpenPendant {

// Step multiplier table — indexed by step_req from ESP32.
// ESP32 STEP_MULT[11] sends its array index; selectable entries are 1,3,8,10.
// Values in mm; 0 = disabled/not selectable.
static const float STEP_MULT[] = {
    0.0f,    //  0: off
    0.001f,  //  1: 0.001 mm  (ESP32 multiplier=1)
    0.005f,  //  2: (not selectable)
    0.01f,   //  3: 0.01 mm   (ESP32 multiplier=10)
    0.02f,   //  4: (not selectable)
    0.03f,   //  5: (not selectable)
    0.04f,   //  6: (not selectable)
    0.05f,   //  7: (not selectable)
    0.1f,    //  8: 0.1 mm    (ESP32 multiplier=100)
    0.5f,    //  9: (not selectable)
    1.0f,    // 10: 1.0 mm    (ESP32 multiplier=1000)
};
static const size_t STEP_MULT_COUNT = sizeof(STEP_MULT) / sizeof(STEP_MULT[0]);

// ----------------------------------------------------------------------
// Handwheel
// ----------------------------------------------------------------------
Handwheel::Handwheel() :
    mCounters(),
    mWheelCout(&mDevNull)
{
}
// ----------------------------------------------------------------------
Handwheel::~Handwheel()
{
}
// ----------------------------------------------------------------------
void Handwheel::enableVerbose(bool enable)
{
    mWheelCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
void Handwheel::setMode(HandWheelCounters::CounterNameToIndex mode)
{
    mCounters.setActiveCounter(mode);
}
// ----------------------------------------------------------------------
void Handwheel::count(int16_t delta)
{
    if (mIsEnabled)
    {
        mCounters.count(delta);
    }

    std::ios init(NULL);
    init.copyfmt(*mWheelCout);
    *mWheelCout << "wheel total counts " << std::setfill(' ') << std::setw(5)
                << mCounters << endl;
    mWheelCout->copyfmt(init);
}
// ----------------------------------------------------------------------
const HandWheelCounters& Handwheel::counters() const
{
    return mCounters;
}
// ----------------------------------------------------------------------
HandWheelCounters& Handwheel::counters()
{
    return mCounters;
}
// ----------------------------------------------------------------------
void Handwheel::setEnabled(bool enabled)
{
    mIsEnabled = enabled;
}

// ----------------------------------------------------------------------
// Pendant
// ----------------------------------------------------------------------
Pendant::Pendant(Hal& hal, Usb& usb) :
    mHal(hal),
    mUsb(usb),
    mHandWheel(),
    mPendantCout(&mDevNull)
{
}
// ----------------------------------------------------------------------
Pendant::~Pendant()
{
}
// ----------------------------------------------------------------------
void Pendant::enableVerbose(bool enable)
{
    mHandWheel.enableVerbose(enable);
    mPendantCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
const Handwheel& Pendant::handWheel() const
{
    return mHandWheel;
}
// ----------------------------------------------------------------------
Handwheel& Pendant::handWheel()
{
    return mHandWheel;
}
// ----------------------------------------------------------------------
void Pendant::processEvent(const pendant_rx_packet_t& rx)
{
    // ── Wheel mode → axis selection + handwheel counter mode ──────────
    using Idx = HandWheelCounters::CounterNameToIndex;
    Idx counterMode = Idx::UNDEFINED;
    bool isAxisMode = false;

    switch (rx.wheel_mode)
    {
        case WHEEL_MODE_X:
            counterMode = Idx::AXIS_X;
            isAxisMode = true;
            break;
        case WHEEL_MODE_Y:
            counterMode = Idx::AXIS_Y;
            isAxisMode = true;
            break;
        case WHEEL_MODE_Z:
            counterMode = Idx::AXIS_Z;
            isAxisMode = true;
            break;
        case WHEEL_MODE_A:
            counterMode = Idx::AXIS_A;
            isAxisMode = true;
            break;
        case WHEEL_MODE_ADJ_FEED:
        case WHEEL_MODE_ADJ_SPINDLE:
            counterMode = Idx::LEAD;
            break;
        case WHEEL_MODE_OFF:
        default:
            break;
    }

    mCurrentWheelMode = rx.wheel_mode;
    mHandWheel.setMode(counterMode);
    mHandWheel.setEnabled(counterMode != Idx::UNDEFINED);

    // Axis active pins
    mHal.setAxisXActive(rx.wheel_mode == WHEEL_MODE_X);
    mHal.setAxisYActive(rx.wheel_mode == WHEEL_MODE_Y);
    mHal.setAxisZActive(rx.wheel_mode == WHEEL_MODE_Z);
    mHal.setAxisAActive(rx.wheel_mode == WHEEL_MODE_A);
    if (!isAxisMode)
    {
        mHal.setNoAxisActive(true);
    }

    // ── Jog wheel: delta from absolute position ──────────────────────
    int16_t delta = (int16_t)(rx.wheel_abs - mLastWheelAbs);
    mLastWheelAbs = rx.wheel_abs;

    if (delta != 0)
    {
        if (rx.wheel_mode == WHEEL_MODE_ADJ_FEED)
        {
            mFeedOvrCounts += delta;
            mHal.setFeedOverrideCounts(mFeedOvrCounts);
        }
        else if (rx.wheel_mode == WHEEL_MODE_ADJ_SPINDLE)
        {
            mSpindleOvrCounts += delta;
            mHal.setSpindleOverrideCounts(mSpindleOvrCounts);
        }
        else
        {
            mHandWheel.count(delta);
            mHal.setJogCounts(mHandWheel.counters());
        }
    }

    // ── Step size from step_req ───────────────────────────────────────
    mCurrentStepIdx = rx.step_req;
    if (rx.step_req < STEP_MULT_COUNT)
    {
        mHal.setStepSize(STEP_MULT[rx.step_req]);
    }

    // ── Buttons: bitmask → HAL setters (rising-edge detection) ───────
    uint32_t pressed  = rx.btn_state & ~mLastBtnState;  // newly pressed
    uint32_t released = mLastBtnState & ~rx.btn_state;  // newly released
    mLastBtnState = rx.btn_state;

    // Start/Pause — toggle on rising edge
    if (pressed & BTN_START_PAUSE)   mHal.setStart(true);
    if (released & BTN_START_PAUSE)  mHal.setStart(false);

    // Stop
    if (pressed & BTN_STOP)          mHal.setStop(true);
    if (released & BTN_STOP)         mHal.setStop(false);

    // Reset / E-Stop
    if (pressed & BTN_RESET)         mHal.setReset(true);
    if (released & BTN_RESET)        mHal.setReset(false);

    // Home all
    if (pressed & BTN_HOME)          mHal.setMachineHomingAll(true);
    if (released & BTN_HOME)         mHal.setMachineHomingAll(false);

    // Safe-Z
    if (pressed & BTN_SAFE_Z)        mHal.setSafeZ(true);
    if (released & BTN_SAFE_Z)       mHal.setSafeZ(false);

    // Probe Z
    if (pressed & BTN_PROBE_Z)       mHal.setProbeZ(true);
    if (released & BTN_PROBE_Z)      mHal.setProbeZ(false);

    // Goto zero (workpiece home)
    if (pressed & BTN_GOTO_ZERO)     mHal.setWorkpieceHome(true);
    if (released & BTN_GOTO_ZERO)    mHal.setWorkpieceHome(false);

    // Spindle toggle
    if (pressed & BTN_SPINDLE)       mHal.toggleSpindleOnOff(true);
    if (released & BTN_SPINDLE)      mHal.toggleSpindleOnOff(false);

    // Macros
    if (pressed & BTN_MACRO_1)       mHal.setMacro1(true);
    if (released & BTN_MACRO_1)      mHal.setMacro1(false);

    if (pressed & BTN_MACRO_2)       mHal.setMacro2(true);
    if (released & BTN_MACRO_2)      mHal.setMacro2(false);

    if (pressed & BTN_MACRO_3)       mHal.setMacro3(true);
    if (released & BTN_MACRO_3)      mHal.setMacro3(false);

    // Half / Zero (axis-related, mapped to macros for flexibility)
    if (pressed & BTN_HALF)          mHal.setMacro4(true);
    if (released & BTN_HALF)         mHal.setMacro4(false);

    if (pressed & BTN_ZERO)          mHal.setWorkpieceZero(true);
    if (released & BTN_ZERO)         mHal.setWorkpieceZero(false);

    // Rewind
    if (pressed & BTN_REWIND)        mHal.setMacro6(true);
    if (released & BTN_REWIND)       mHal.setMacro6(false);

    // E-Stop (separate from Reset)
    if (pressed & BTN_ESTOP)         mHal.setReset(true);
    if (released & BTN_ESTOP)        mHal.setReset(false);

    *mPendantCout << "pndnt rx btn=0x" << std::hex << rx.btn_state
                  << " mode=0x" << (int)rx.wheel_mode
                  << " wheel=" << std::dec << rx.wheel_abs
                  << " step=" << (int)rx.step_req
                  << " delta=" << delta << endl;
}
// ----------------------------------------------------------------------
void Pendant::updateDisplayData()
{
    pendant_tx_packet_t& pkt = mUsb.getOutputPackageData();
    pkt.report_id   = 2;

    // DRO positions (workspace / relative)
    pkt.dro_x = static_cast<float>(mHal.getAxisXPosition(false));
    pkt.dro_y = static_cast<float>(mHal.getAxisYPosition(false));
    pkt.dro_z = static_cast<float>(mHal.getAxisZPosition(false));
    pkt.dro_a = static_cast<float>(mHal.getAxisAPosition(false));

    // Machine positions (absolute)
    pkt.mach_x = static_cast<float>(mHal.getAxisXPosition(true));
    pkt.mach_y = static_cast<float>(mHal.getAxisYPosition(true));
    pkt.mach_z = static_cast<float>(mHal.getAxisZPosition(true));
    pkt.mach_a = static_cast<float>(mHal.getAxisAPosition(true));

    // Feed rate: override × max velocity → mm/min
    pkt.feed_rate = static_cast<int32_t>(
        mHal.getFeedOverrideValue() * mHal.getFeedOverrideMaxVel() * 60.0);
    // Feed rate: actual feed in mm/min (0 when idle; from motion.feed-mm-per-minute)
    pkt.feed_rate = static_cast<int32_t>(mHal.getFeedRateMmPerMinute());

    // Spindle RPM
    pkt.spindle_rpm = static_cast<int32_t>(mHal.getspindleSpeedCmd());

    // Overrides (0–100+ %)
    pkt.feed_ovr    = static_cast<uint16_t>(mHal.getFeedOverrideValue() * 100.0);
    pkt.spindle_ovr = static_cast<uint16_t>(mHal.getSpindleOverrideValue() * 100.0);

    // Step size echo
    pkt.step_active = mCurrentStepIdx;

    // Wheel mode echo — confirm accepted mode back to pendant
    pkt.wheelMode_active = mCurrentWheelMode;

    // Status bits
    uint32_t bits = 0;
    if (mHal.getIsMachineOn())  bits |= HAL_BIT_MACHINE_ON;
    // isProgramRunning / isProgramPaused are read via public getters
    // but Hal doesn't expose them directly — use the HalMemory In pins
    // These will be wired once hal.h exposes the necessary getters.
    pkt.hal_status_bits = bits;

    // Tool name — reserved for future use
}
// ----------------------------------------------------------------------
void Pendant::clearDisplayData()
{
    pendant_tx_packet_t& pkt = mUsb.getOutputPackageData();
    memset(&pkt, 0, sizeof(pkt));
    pkt.report_id = 2;
}

} // namespace OpenPendant
