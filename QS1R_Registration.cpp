/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Paul H Alfille -- after SoapyQS1R example
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

static std::vector<SoapySDR::Kwargs> findQS1R(const SoapySDR::Kwargs &args)
{
    SoapyQS1RSession Sess;

    std::vector<SoapySDR::Kwargs> results;

    libusb_device ** devlist ;
    ssize_t usb_count = libusb_get_device_list( SoapyQS1RSession::qs1r_context, &devlist ) ;

    int index = 0 ;

    for ( auto i =0 ; i < usb_count ; ++i ) {
        struct libusb_device_descriptor desc ;
        libusb_get_device_descriptor( devlist[i], &desc ) ; // always succeeds in modern libusb
        if (desc.idVendor == QS1R_VID && desc.idProduct == QS1R_PID) {
            libusb_device_handle * dev ;
            if ( libusb_open( devlist[i], &dev ) == 0 ) {

                // Set description fields
                SoapySDR::Kwargs devInfo;

                unsigned char adevice[256] ;
                libusb_get_string_descriptor_ascii( dev, desc.iProduct, adevice, 256 ) ;
                devInfo["product"] = (char *) adevice ;

                unsigned char aserial[256] ;
                libusb_get_string_descriptor_ascii( dev, desc.iSerialNumber, aserial, 256 ) ;
                devInfo["serial"] = (char *) aserial;

                unsigned char amanf[256] ;
                libusb_get_string_descriptor_ascii( dev, desc.iManufacturer, amanf, 256 ) ;
                devInfo["manufacturer"] = (char *) amanf;

                char aindex[256] ;
                sprintf( aindex, "%d", index ) ;
                devInfo["index"] = (char *) aindex;
                ++index ;

                char alabel[256] ;
                sprintf( alabel, "QS1R %s on <%d:%d>", aserial, libusb_get_bus_number(devlist[i]), libusb_get_port_number(devlist[i]) );
                devInfo["label"] = alabel ;

                libusb_close( dev ) ;

                results.push_back(devInfo) ;
            }
        }
    }
    libusb_free_device_list( devlist, 1 ) ;

    return results;
}

static SoapySDR::Device *makeQS1R(const SoapySDR::Kwargs &args)
{
    return new SoapyQS1R(args);
}

static SoapySDR::Registry registerQS1R("qs1r", &findQS1R, &makeQS1R, SOAPY_SDR_ABI_VERSION);
