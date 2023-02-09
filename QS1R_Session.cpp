/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Josh Blum
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyQS1R.hpp"
#include <SoapySDR/Logger.hpp>
#include <mutex>
#include <cstddef>

static std::mutex sessionMutex;
static size_t sessionCount = 0;
static libusb_context * qs1r_context = NULL;

SoapyQS1RSession::SoapyQS1RSession(void)
{
    std::lock_guard<std::mutex> lock(sessionMutex);

    if (sessionCount == 0)
    {
        // libusb
        int usb_ret = libusb_init(&qs1r_context);
        if ( usb_ret != 0 ) {
            SoapySDR::logf(SOAPY_SDR_ERROR, "qs1r_init() failed -- %s", libusb_strerr(usb_ret));
        }
    }
    sessionCount++;
}

SoapyQS1RSession::~SoapyQS1RSession(void)
{
    std::lock_guard<std::mutex> lock(sessionMutex);

    sessionCount--;
    if (sessionCount == 0)
    {
        libusb_exit( qs1r_context ) ;
    }
}
