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
#include <stdio.h>

int QS1R_initialize_device( libusb_device_handle * dev ) {
	if ( libusb_set_configuration( dev, 1 )            != 0 ) return -1 ;
	if ( libusb_claim_interface( dev, 0   )            != 0 ) return -1 ;
	if ( libusb_set_interface_alt_setting( dev, 0, 0 ) != 0 ) return -1 ;
	if ( libusb_clear_halt( dev, QS1R_EP2 )            != 0 ) return -1 ;
	if ( libusb_clear_halt( dev, QS1R_EP4 )            != 0 ) return -1 ;
	if ( libusb_clear_halt( dev, QS1R_EP6 )            != 0 ) return -1 ;
	if ( libusb_clear_halt( dev, QS1R_EP8 )            != 0 ) return -1 ;
	return 0 ;
}

int QS1R_ram_write( libusb_device_handle * dev, int ram_address, unsigned char * buffer, int length ) {
	// write the the FX2 CPU's ram
	int pkt_size = MAX_EP0_PACKET_SIZE ;
	int this_address = ram_address;
	unsigned char * this_buffer = buffer ;

	for ( int remaining = length ; remaining > 0 ; remaining -= pkt_size ) {
		if (pkt_size > remaining ) {
			pkt_size = remaining ;
		} 
		int sent = libusb_control_transfer( dev,
			VRT_VENDOR_OUT,
			FX2_WRITE_RAM_REQ,
			this_address,
			0,
			this_buffer,
			pkt_size,
			USB_TIMEOUT_CONTROL );
		if ( sent != pkt_size ) {
			return -1 ;
		}
		this_address += sent ;
		this_buffer += sent ;
	}
	return 0 ;
}

int QS1R_CPU_RESET( libusb_device_handle * dev, int state ) {
	// State 1=reset, 0=normal
	unsigned char s = state ;
	return QS1R_ram_write( dev, FX2_RAM_RESET, &s, 1 ) ;
}

int QS1R_load_firmware( libusb_device_handle * dev, const char * filename ) {
	FILE * firm = fopen( filename, "r" ) ;
	if ( firm == 0 ) {
		SoapySDR::logf(SOAPY_SDR_ERROR, "Cannot open firmware file -- %s", filename);
		return -1 ;
	}

	if ( QS1R_CPU_RESET( dev, 1 ) != 0 ) return -1 ;
	int ret = 0 ;
	while ( !feof(firm) ) {
		char s[1024] ;
		fgets( s, 1024, firm ) ;
		char c ;
		int flength;
		int faddr;
		int type;
		sscanf( s, "%c%02x%04x%02x", &c, &flength, %faddr, &type ) ;
		if ( c != ':' ) {
			ret = -1 ;
			break ;
		}
		switch ( type ) {
			case 0:
				break ;
			case 1:
				//ignore
				break ;
			case 2:
			default:
				// extended address -- not supported
				ret = -1;
				break ;
		}

	}
	fclose( firm ) ;

	if ( QS1R_CPU_RESET( dev, 0 ) != 0 ) return -1 ;
	return ret ;
}

