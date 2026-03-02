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

#include "./openpendant.h"

// system includes
#include <assert.h>
#include <iostream>
#include <iomanip>
#include <libusb.h>

using std::endl;

namespace OpenPendant {
// ----------------------------------------------------------------------
void OpenPendantComponent::initHal()
{
    mHal.init();
}
// ----------------------------------------------------------------------
void OpenPendantComponent::teardownHal()
{
    hal_exit(mHal.getHalComponentId());
}
// ----------------------------------------------------------------------
const char* OpenPendantComponent::getName() const
{
    return mName;
}
// ----------------------------------------------------------------------
const char* OpenPendantComponent::getHalName() const
{
    return mHal.getHalComponentName();
}
// ----------------------------------------------------------------------
void OpenPendantComponent::onInputDataReceived(const pendant_rx_packet_t& inPackage)
{
    *mRxCout << "in    rx btn=0x" << std::hex << inPackage.btn_state
             << " mode=0x" << (int)inPackage.wheel_mode
             << " wheel=" << std::dec << inPackage.wheel_abs
             << " step=" << (int)inPackage.step_req << endl;

    mPendant.processEvent(inPackage);
}
// ----------------------------------------------------------------------
void OpenPendantComponent::initWhb()
{
    mIsRunning = true;
    mUsb.setIsRunning(true);
}
// ----------------------------------------------------------------------
void OpenPendantComponent::requestTermination(int signal)
{
    if (signal >= 0)
    {
        *mInitCout << "termination requested upon signal number " << signal << " ..." << endl;
    }
    else
    {
        *mInitCout << "termination requested ... " << endl;
    }
    mUsb.requestTermination();
    mIsRunning = false;
}
// ----------------------------------------------------------------------
bool OpenPendantComponent::isRunning() const
{
    return mIsRunning;
}
// ----------------------------------------------------------------------
OpenPendantComponent::OpenPendantComponent() :
    mName("OpenPendant"),
    mHal(),
    mUsb(mName, *this, mHal),
    mTxCout(&mDevNull),
    mRxCout(&mDevNull),
    mHalInitCout(&mDevNull),
    mInitCout(&mDevNull),
    packageReceivedEventReceiver(*this),
    mPendant(mHal, mUsb)
{
    (void)packageReceivedEventReceiver;
    setSimulationMode(true);
    enableVerbosePendant(false);
    enableVerboseRx(false);
    enableVerboseTx(false);
    enableVerboseInit(false);
    enableVerboseHal(false);
}
// ----------------------------------------------------------------------
OpenPendantComponent::~OpenPendantComponent()
{
}
// ----------------------------------------------------------------------
void OpenPendantComponent::updateDisplay()
{
    if (mIsRunning)
    {
        mPendant.updateDisplayData();
    }
    else
    {
        mPendant.clearDisplayData();
    }
    mUsb.sendDisplayData();
}
// ----------------------------------------------------------------------
int OpenPendantComponent::run()
{
    if (mHal.isSimulationModeEnabled())
    {
        *mInitCout << "init  starting in simulation mode" << endl;
    }

    bool isHalReady = false;
    initWhb();
    initHal();

    if (!mUsb.isWaitForPendantBeforeHalEnabled() && !mHal.isSimulationModeEnabled())
    {
        hal_ready(mHal.getHalComponentId());
        isHalReady = true;
    }

    while (isRunning())
    {
        mHal.setIsPendantConnected(false);

        initWhb();
        if (!mUsb.init())
        {
            return EXIT_FAILURE;
        }

        mHal.setIsPendantConnected(true);

        if (!isHalReady && !mHal.isSimulationModeEnabled())
        {
            hal_ready(mHal.getHalComponentId());
            isHalReady = true;
        }

        if (mUsb.isDeviceOpen())
        {
            *mInitCout << "init  enabling reception ...";
            if (!enableReceiveAsyncTransfer())
            {
                std::cerr << endl << "failed to enable reception" << endl;
                return EXIT_FAILURE;
            }
            *mInitCout << " ok" << endl;
        }
        process();
        teardownUsb();
    }
    teardownHal();

    return EXIT_SUCCESS;
}
// ----------------------------------------------------------------------
void OpenPendantComponent::linuxcncSimulate()
{
}
// ----------------------------------------------------------------------
bool OpenPendantComponent::enableReceiveAsyncTransfer()
{
    return mUsb.setupAsyncTransfer();
}
// ----------------------------------------------------------------------
void OpenPendantComponent::setSimulationMode(bool enableSimulationMode)
{
    mIsSimulationMode = enableSimulationMode;
    mHal.setSimulationMode(mIsSimulationMode);
    mUsb.setSimulationMode(mIsSimulationMode);
}
// ----------------------------------------------------------------------
void OpenPendantComponent::setUsbContext(libusb_context* context)
{
    mUsb.setContext(context);
}
// ----------------------------------------------------------------------
libusb_device_handle* OpenPendantComponent::getUsbDeviceHandle()
{
    return mUsb.getDeviceHandle();
}
// ----------------------------------------------------------------------
libusb_context* OpenPendantComponent::getUsbContext()
{
    return mUsb.getContext();
}
// ----------------------------------------------------------------------
void OpenPendantComponent::process()
{
    if (mUsb.isDeviceOpen())
    {
        while (isRunning() && !mUsb.getDoReconnect())
        {
            struct timeval timeout;
            timeout.tv_sec  = 0;
            timeout.tv_usec = 200 * 1000;

            int r = libusb_handle_events_timeout_completed(getUsbContext(), &timeout, nullptr);
            assert((r == LIBUSB_SUCCESS) || (r == LIBUSB_ERROR_NO_DEVICE) || (r == LIBUSB_ERROR_BUSY) ||
                   (r == LIBUSB_ERROR_TIMEOUT) || (r == LIBUSB_ERROR_INTERRUPTED));
            if (mHal.isSimulationModeEnabled())
            {
                linuxcncSimulate();
            }
            updateDisplay();
        }
        updateDisplay();

        mHal.setIsPendantConnected(false);
        *mInitCout << "connection lost, cleaning up" << endl;
        struct timeval tv;
        tv.tv_sec  = 1;
        tv.tv_usec = 0;
        int r = libusb_handle_events_timeout_completed(getUsbContext(), &tv, nullptr);
        assert(0 == r);
        r = libusb_release_interface(getUsbDeviceHandle(), 0);
        assert((0 == r) || (r == LIBUSB_ERROR_NO_DEVICE));
        libusb_close(getUsbDeviceHandle());
        mUsb.setDeviceHandle(nullptr);
    }
}
// ----------------------------------------------------------------------
void OpenPendantComponent::teardownUsb()
{
    libusb_exit(getUsbContext());
    mUsb.setContext(nullptr);
}
// ----------------------------------------------------------------------
void OpenPendantComponent::enableVerbosePendant(bool enable)
{
    mPendant.enableVerbose(enable);
}
// ----------------------------------------------------------------------
void OpenPendantComponent::enableVerboseRx(bool enable)
{
    mUsb.enableVerboseRx(enable);
    mRxCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
void OpenPendantComponent::enableVerboseTx(bool enable)
{
    mUsb.enableVerboseTx(enable);
    mTxCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
void OpenPendantComponent::enableVerboseHal(bool enable)
{
    mHal.enableVerbose(enable);
    mHalInitCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
void OpenPendantComponent::enableVerboseInit(bool enable)
{
    mUsb.enableVerboseInit(enable);
    mInitCout = enable ? &std::cout : &mDevNull;
}
// ----------------------------------------------------------------------
void OpenPendantComponent::setWaitWithTimeout(uint8_t waitSecs)
{
    mUsb.setWaitWithTimeout(waitSecs);
}
// ----------------------------------------------------------------------
bool OpenPendantComponent::isSimulationModeEnabled() const
{
    return mIsSimulationMode;
}
}
