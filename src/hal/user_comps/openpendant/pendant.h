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

#ifndef __OPENPENDANT_PENDANT_H
#define __OPENPENDANT_PENDANT_H

// local includes
#include "pendant-types.h"
#include "pendant_protocol.h"

// system includes
#include <stdint.h>
#include <ostream>

namespace OpenPendant {

// forward declarations
class Hal;
class Usb;

// ----------------------------------------------------------------------
//! Accumulates jog wheel counts per axis.
class Handwheel
{
public:
    Handwheel();
    ~Handwheel();

    void enableVerbose(bool enable);
    void setMode(HandWheelCounters::CounterNameToIndex mode);
    void count(int16_t delta);
    const HandWheelCounters& counters() const;
    HandWheelCounters& counters();
    void setEnabled(bool enabled);

private:
    HandWheelCounters mCounters;
    bool              mIsEnabled{false};
    std::ostream      mDevNull{nullptr};
    std::ostream    * mWheelCout;
};

// ----------------------------------------------------------------------
//! V2 pendant logic: processes pendant_rx_packet_t input, fills
//! pendant_tx_packet_t output from HAL pins.
class Pendant
{
public:
    Pendant(Hal& hal, Usb& usb);
    ~Pendant();

    //! Process a received INPUT report from the ESP32 pendant.
    void processEvent(const pendant_rx_packet_t& rx);

    //! Fill the OUTPUT report with current HAL pin values.
    void updateDisplayData();

    //! Zero out the OUTPUT report (on shutdown).
    void clearDisplayData();

    void enableVerbose(bool enable);

    const Handwheel& handWheel() const;
    Handwheel& handWheel();

private:
    Hal       & mHal;
    Usb       & mUsb;
    Handwheel   mHandWheel;

    int16_t  mLastWheelAbs{0};
    uint32_t mLastBtnState{0};
    uint8_t  mCurrentStepIdx{0};
    uint8_t  mCurrentWheelMode{WHEEL_MODE_OFF};

    std::ostream  mDevNull{nullptr};
    std::ostream* mPendantCout;
};

} // namespace OpenPendant
#endif
