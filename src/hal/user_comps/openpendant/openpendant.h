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

#ifndef __OPEN_PENDANT_H
#define __OPEN_PENDANT_H

// system includes
#include <stdint.h>

// local includes
#include "./hal.h"
#include "./usb.h"
#include "./pendant.h"


namespace OpenPendant {

// ----------------------------------------------------------------------
//! The OpenPendant V2 user space component for LinuxCNC.
class OpenPendantComponent :
    public OnUsbInputPackageListener
{
public:
    OpenPendantComponent();
    virtual ~OpenPendantComponent();
    void process();
    void teardownUsb();
    void setUsbContext(libusb_context* context);
    libusb_device_handle* getUsbDeviceHandle();
    libusb_context* getUsbContext();
    const char* getName() const;
    const char* getHalName() const;
    //! callback received by Usb when an INPUT report arrives
    void onInputDataReceived(const pendant_rx_packet_t& inPackage) override;
    void initWhb();
    void initHal();
    void teardownHal();
    bool enableReceiveAsyncTransfer();
    void updateDisplay();
    void linuxcncSimulate();
    void requestTermination(int signal = -42);
    bool isRunning() const;
    int run();
    bool isSimulationModeEnabled() const;
    void setSimulationMode(bool enableSimulationMode);
    void enableVerbosePendant(bool enable);
    void enableVerboseRx(bool enable);
    void enableVerboseTx(bool enable);
    void enableVerboseHal(bool enable);
    void enableVerboseInit(bool enable);
    void setWaitWithTimeout(uint8_t waitSecs = 3);

private:
    const char* mName;
    Hal                   mHal;
    Usb                   mUsb;
    bool                  mIsRunning{false};
    bool                  mIsSimulationMode{false};
    std::ostream          mDevNull{nullptr};
    std::ostream        * mTxCout;
    std::ostream        * mRxCout;
    std::ostream        * mHalInitCout;
    std::ostream        * mInitCout;
    OnUsbInputPackageListener& packageReceivedEventReceiver;
    Pendant mPendant;
};
}
#endif
