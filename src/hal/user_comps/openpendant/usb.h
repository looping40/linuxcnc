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

#ifndef __OPENPENDANT_USB_H
#define __OPENPENDANT_USB_H

// system includes
#include <stdint.h>
#include <ostream>

// shared protocol definition
#include "pendant_protocol.h"

// forward declarations
struct libusb_device_handle;
struct libusb_context;
struct libusb_transfer;

namespace OpenPendant {

// forward declarations
class Hal;
class Usb;

// ----------------------------------------------------------------------
//! Buffer for receiving INPUT reports (ESP32 → LinuxCNC).
union UsbInPackageBuffer
{
public:
    pendant_rx_packet_t asFields;
    uint8_t             asBuffer[sizeof(pendant_rx_packet_t)];
    UsbInPackageBuffer();
} __attribute__((packed));

// ----------------------------------------------------------------------
//! Callback interface for structured input data.
class OnUsbInputPackageListener
{
public:
    virtual void onInputDataReceived(const pendant_rx_packet_t& inPackage) = 0;
    virtual ~OnUsbInputPackageListener();
};

// ----------------------------------------------------------------------
//! Callback interface for raw USB transfer data.
class UsbRawInputListener
{
public:
    virtual void onUsbDataReceived(struct libusb_transfer* transfer) = 0;
    virtual ~UsbRawInputListener();
};

// ----------------------------------------------------------------------
//! USB communication with the ESP32 pendant.
//! Sends pendant_tx_packet_t via interrupt OUT transfer.
//! Receives pendant_rx_packet_t via async bulk IN transfer.
class Usb : public UsbRawInputListener
{
public:
    Usb(const char* name, OnUsbInputPackageListener& onDataReceivedCallback, Hal& hal);
    ~Usb();

    uint16_t getUsbVendorId() const;
    uint16_t getUsbProductId() const;
    bool isDeviceOpen() const;

    libusb_context** getContextReference();
    libusb_context* getContext();
    void setContext(libusb_context* context);
    libusb_device_handle* getDeviceHandle();
    void setDeviceHandle(libusb_device_handle* deviceHandle);

    bool isWaitForPendantBeforeHalEnabled() const;
    bool getDoReconnect() const;
    void setDoReconnect(bool doReconnect);

    void onUsbDataReceived(struct libusb_transfer* transfer) override;

    void setSimulationMode(bool isSimulationMode);
    void setIsRunning(bool enableRunning);
    void requestTermination();

    bool setupAsyncTransfer();
    void sendDisplayData();

    void enableVerboseTx(bool enable);
    void enableVerboseRx(bool enable);
    void enableVerboseInit(bool enable);

    bool init();
    void setWaitWithTimeout(uint8_t waitSecs);

    pendant_tx_packet_t& getOutputPackageData();

private:
    const uint16_t usbVendorId{0x1111};
    const uint16_t usbProductId{0x2222};

    libusb_context      * context{nullptr};
    libusb_device_handle* deviceHandle{nullptr};
    bool                mDoReconnect{false};
    bool                isWaitWithTimeout{false};
    bool                mIsSimulationMode{false};
    bool                mIsRunning{false};

    UsbInPackageBuffer    inputPackageBuffer;
    pendant_tx_packet_t   outputPackageData{};

    OnUsbInputPackageListener& mDataHandler;
    void (* const mRawDataCallback)(struct libusb_transfer*);
    Hal                   & mHal;
    struct libusb_transfer* inTransfer{nullptr};
    struct libusb_transfer* outTransfer{nullptr};

    std::ostream devNull{nullptr};
    std::ostream* verboseTxOut{nullptr};
    std::ostream* verboseRxOut{nullptr};
    std::ostream* verboseInitOut{nullptr};
    const char  * mName{nullptr};
    uint8_t mWaitSecs{0};
};

} // namespace OpenPendant
#endif
