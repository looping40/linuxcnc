/*
   Copyright (C) 2017 Raoul Rubien (github.com/rubienr).
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

#include "./usb.h"

// system includes
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <libusb.h>
#include <unistd.h>
#include <string.h>

// local includes
#include "./hal.h"
#include "./openpendant.h"

using std::endl;

namespace OpenPendant {

// ----------------------------------------------------------------------
//! Libusb async transfer callback — routes raw data to the Usb object.
void usbInputResponseCallback(struct libusb_transfer* transfer)
{
    assert(transfer->user_data != nullptr);
    UsbRawInputListener* receiver = reinterpret_cast<UsbRawInputListener*>(transfer->user_data);
    receiver->onUsbDataReceived(transfer);
}

// ----------------------------------------------------------------------
UsbInPackageBuffer::UsbInPackageBuffer() :
    asBuffer{0}
{
    static_assert(sizeof(asFields) == sizeof(asBuffer),
                  "UsbInPackageBuffer union size mismatch");
}

// ----------------------------------------------------------------------
Usb::Usb(const char* name, OnUsbInputPackageListener& onDataReceivedCallback, Hal& hal) :
    inputPackageBuffer(),
    mDataHandler(onDataReceivedCallback),
    mRawDataCallback(usbInputResponseCallback),
    mHal(hal),
    inTransfer(libusb_alloc_transfer(0)),
    outTransfer(libusb_alloc_transfer(0)),
    verboseTxOut(&devNull),
    verboseRxOut(&devNull),
    verboseInitOut(&devNull),
    mName(name)
{
    memset(&outputPackageData, 0, sizeof(outputPackageData));
    outputPackageData.report_id = 2;
}

// ----------------------------------------------------------------------
Usb::~Usb()
{
}

// ----------------------------------------------------------------------
uint16_t Usb::getUsbVendorId() const
{
    return usbVendorId;
}

// ----------------------------------------------------------------------
uint16_t Usb::getUsbProductId() const
{
    return usbProductId;
}

// ----------------------------------------------------------------------
bool Usb::isDeviceOpen() const
{
    return deviceHandle != nullptr;
}

// ----------------------------------------------------------------------
libusb_context** Usb::getContextReference()
{
    return &context;
}

// ----------------------------------------------------------------------
libusb_context* Usb::getContext()
{
    return context;
}

// ----------------------------------------------------------------------
void Usb::setContext(libusb_context* context)
{
    this->context = context;
}

// ----------------------------------------------------------------------
libusb_device_handle* Usb::getDeviceHandle()
{
    return deviceHandle;
}

// ----------------------------------------------------------------------
void Usb::setDeviceHandle(libusb_device_handle* deviceHandle)
{
    this->deviceHandle = deviceHandle;
}

// ----------------------------------------------------------------------
bool Usb::isWaitForPendantBeforeHalEnabled() const
{
    return isWaitWithTimeout;
}

// ----------------------------------------------------------------------
bool Usb::getDoReconnect() const
{
    return mDoReconnect;
}

// ----------------------------------------------------------------------
void Usb::setDoReconnect(bool doReconnect)
{
    this->mDoReconnect = doReconnect;
}

// ----------------------------------------------------------------------
void Usb::sendDisplayData()
{
    *verboseTxOut << "out   sending " << sizeof(outputPackageData) << "B via interrupt OUT"
                  << endl;

    // Send via interrupt OUT endpoint.
    // outputPackageData starts with report_id=2, which TinyUSB extracts from buffer[0]
    // to dispatch to the correct _onOutput handler.
    int transferred = 0;
    int r = libusb_interrupt_transfer(deviceHandle,
                                      (0x1 | LIBUSB_ENDPOINT_OUT),  // EP1 OUT
                                      reinterpret_cast<uint8_t*>(&outputPackageData),
                                      sizeof(outputPackageData),
                                      &transferred,
                                      100);
    if (r < 0)
    {
        std::cerr << "transmission failed (" << libusb_error_name(r)
                  << "), try to reconnect ..." << endl;
        setDoReconnect(true);
    }
}

// ----------------------------------------------------------------------
void Usb::onUsbDataReceived(struct libusb_transfer* transfer)
{
    assert(mHal.isInitialized());

    int expectedSize = static_cast<int>(sizeof(pendant_rx_packet_t));

    switch (transfer->status)
    {
        case (LIBUSB_TRANSFER_COMPLETED):
            if (transfer->actual_length >= expectedSize)
            {
                memcpy(&inputPackageBuffer.asBuffer, transfer->buffer, sizeof(pendant_rx_packet_t));
                mDataHandler.onInputDataReceived(inputPackageBuffer.asFields);
            }
            else
            {
                std::cerr << "received unexpected package size: actual="
                          << transfer->actual_length << ", expected=" << expectedSize << endl;
            }

            if (mIsRunning)
            {
                setupAsyncTransfer();
            }
            break;

        case (LIBUSB_TRANSFER_TIMED_OUT):
            if (mIsRunning)
            {
                setupAsyncTransfer();
            }
            break;

        case (LIBUSB_TRANSFER_CANCELLED):
            break;

        case (LIBUSB_TRANSFER_STALL):
        case (LIBUSB_TRANSFER_NO_DEVICE):
        case (LIBUSB_TRANSFER_OVERFLOW):
        case (LIBUSB_TRANSFER_ERROR):
            std::cerr << "transfer error: " << transfer->status
                      << ", requesting reconnect" << endl;
            setDoReconnect(true);
            break;

        default:
            std::cerr << "unknown transfer status: " << transfer->status << endl;
            requestTermination();
            break;
    }
}

// ----------------------------------------------------------------------
void Usb::setSimulationMode(bool isSimulationMode)
{
    mIsSimulationMode = isSimulationMode;
}

// ----------------------------------------------------------------------
void Usb::setIsRunning(bool enableRunning)
{
    mIsRunning = enableRunning;
}

// ----------------------------------------------------------------------
void Usb::requestTermination()
{
    mIsRunning = false;
}

// ----------------------------------------------------------------------
void Usb::cancelAsyncTransfer()
{
    if (inTransfer != nullptr)
    {
        libusb_cancel_transfer(inTransfer);
    }
}

// ----------------------------------------------------------------------
void Usb::reallocTransfers()
{
    if (inTransfer != nullptr)
    {
        libusb_free_transfer(inTransfer);
    }
    if (outTransfer != nullptr)
    {
        libusb_free_transfer(outTransfer);
    }
    inTransfer = libusb_alloc_transfer(0);
    outTransfer = libusb_alloc_transfer(0);
}

// ----------------------------------------------------------------------
bool Usb::setupAsyncTransfer()
{
    assert(inTransfer != nullptr);
    libusb_fill_interrupt_transfer(inTransfer, deviceHandle,
                              (0x1 | LIBUSB_ENDPOINT_IN),
                              inputPackageBuffer.asBuffer,
                              sizeof(inputPackageBuffer.asBuffer),
                              mRawDataCallback,
                              static_cast<void*>(this),
                              750);
    int r = libusb_submit_transfer(inTransfer);
    if (r != 0)
    {
        std::cerr << "setupAsyncTransfer: libusb_submit_transfer failed: "
                  << libusb_error_name(r) << endl;
        return false;
    }
    return true;
}

// ----------------------------------------------------------------------
void Usb::enableVerboseTx(bool enable)
{
    verboseTxOut = enable ? &std::cout : &devNull;
}

// ----------------------------------------------------------------------
void Usb::enableVerboseRx(bool enable)
{
    verboseRxOut = enable ? &std::cout : &devNull;
}

// ----------------------------------------------------------------------
void Usb::enableVerboseInit(bool enable)
{
    verboseInitOut = enable ? &std::cout : &devNull;
}

// ----------------------------------------------------------------------
bool Usb::init()
{
    if (getDoReconnect())
    {
        int pauseSecs = 3;
        *verboseInitOut << "init  pausing " << pauseSecs << "s, waiting for device to be gone ...";
        while ((pauseSecs--) >= 0)
        {
            *verboseInitOut << "." << std::flush;
            sleep(1);
        }
        setDoReconnect(false);
        *verboseInitOut << " done" << endl;
    }

    *verboseInitOut << "init  usb context ...";
    int r = libusb_init(&context);
    if (r != 0)
    {
        std::cerr << endl << "failed to initialize usb context" << endl;
        return false;
    }
    *verboseInitOut << " ok" << endl;

    std::ios init(NULL);
    init.copyfmt(*verboseInitOut);
    if (isWaitWithTimeout)
    {
        *verboseInitOut << "init  waiting maximum " << static_cast<unsigned short>(mWaitSecs) << "s for device "
                        << mName << " vendorId=0x" << std::hex << std::setfill('0') << std::setw(4) << usbVendorId
                        << " productId=0x" << std::setw(4) << usbProductId << " ...";
    }
    else
    {
        *verboseInitOut << "init  not waiting for device " << mName
                        << " vendorId=0x" << std::hex << std::setfill('0') << std::setw(4) << usbVendorId
                        << " productId=0x" << std::setw(4) << usbProductId << std::dec
                        << ", will continue in " << static_cast<unsigned short>(mWaitSecs) << "s ...";
    }
    verboseInitOut->copyfmt(init);

    do
    {
        libusb_device** devicesReference;
        ssize_t devicesCount = libusb_get_device_list(context, &devicesReference);
        if (devicesCount < 0)
        {
            std::cerr << endl << "failed to get device list" << endl;
            return false;
        }

        deviceHandle = libusb_open_device_with_vid_pid(context, usbVendorId, usbProductId);
        libusb_free_device_list(devicesReference, 1);
        *verboseInitOut << "." << std::flush;
        if (!isDeviceOpen())
        {
            *verboseInitOut << "." << std::flush;
            if (isWaitWithTimeout)
            {
                *verboseInitOut << "." << std::flush;
                if ((mWaitSecs--) <= 0)
                {
                    std::cerr << endl << "timeout exceeded, exiting" << endl;
                    return false;
                }
            }
            sleep(1);
        }
    } while (!isDeviceOpen() && mIsRunning);
    *verboseInitOut << " ok" << endl
                    << "init  " << mName << " device found" << endl;

    if (isDeviceOpen())
    {
        *verboseInitOut << "init  detaching active kernel driver ...";
        if (libusb_kernel_driver_active(deviceHandle, 0) == 1)
        {
            int r = libusb_detach_kernel_driver(deviceHandle, 0);
            assert(0 == r);
            *verboseInitOut << " ok" << endl;
        }
        else
        {
            *verboseInitOut << " already detached" << endl;
        }
        *verboseInitOut << "init  claiming interface ...";
        int r = libusb_claim_interface(deviceHandle, 0);
        if (r != 0)
        {
            std::cerr << endl << "failed to claim interface" << endl;
            return false;
        }
        *verboseInitOut << " ok" << endl;
    }
    return true;
}

// ----------------------------------------------------------------------
void Usb::setWaitWithTimeout(uint8_t waitSecs)
{
    mWaitSecs         = waitSecs;
    isWaitWithTimeout = (mWaitSecs > 0);
}

// ----------------------------------------------------------------------
pendant_tx_packet_t& Usb::getOutputPackageData()
{
    return outputPackageData;
}

// ----------------------------------------------------------------------
UsbRawInputListener::~UsbRawInputListener()
{
}

// ----------------------------------------------------------------------
OnUsbInputPackageListener::~OnUsbInputPackageListener()
{
}

} // namespace OpenPendant
