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

// All routines are true if successful, else false on error

bool ram_write( libusb_device_handle * dev, int ram_address, unsigned char * buffer, int length ) ;
bool cpu_reset( libusb_device_handle * dev, int state ) ;
bool firmware_write( libusb_device_handle * dev, const char * filename ) ;
bool firmware_line( libusb_device_handle * dev, char * line ) ;
bool firmware_line_type0( libusb_device_handle * dev, char * line, int flength, int faddr ) ;
bool FPGA_write( libusb_device_handle * dev, const char * filename ) ;
bool FPGA_control( libusb_device_handle * dev, int state ) ;
bool FPGA_packet( libusb_device_handle * dev, FILE * rbf ) ;
bool bulk_write_EP( libusb_device_handle * dev, int ep, unsigned char * buffer, int length ) ;

bool QS1R_initialize_device( libusb_device_handle * dev ) {
	return (libusb_set_configuration( dev, 1 )==0)
	    && (libusb_claim_interface( dev, 0 )==0)
	    && (libusb_set_interface_alt_setting( dev, 0, 0 )==0)
	    && (libusb_clear_halt( dev, QS1R_EP2 & 0x0F )==0)
	    && (libusb_clear_halt( dev, QS1R_EP4 & 0x0F )==0)
	    && (libusb_clear_halt( dev, QS1R_EP6 & 0x0F )==0)
	    && (libusb_clear_halt( dev, QS1R_EP8 & 0x0F )==0)
		&& firmware_write( dev, "firmware/qs1r_firmware_03032011.hex" )
		&& FPGA_write( dev, "firmware/QS1R_WINRAD_04112011.rbf" ) ;
}

bool ram_write( libusb_device_handle * dev, int ram_address, unsigned char * buffer, int length ) {
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
			return false ;
		}
		this_address += sent ;
		this_buffer += sent ;
	}
	return true ;
}

bool cpu_reset( libusb_device_handle * dev, int state ) {
	// State 1=reset, 0=normal
	unsigned char s = state ;
	return ram_write( dev, FX2_RAM_RESET, &s, 1 ) ;
}

bool firmware_write( libusb_device_handle * dev, const char * filename ) {
	FILE * firm = fopen( filename, "r" ) ;
	if ( firm == NULL ) {
		SoapySDR::logf(SOAPY_SDR_ERROR, "Cannot open firmware file -- %s", filename);
		return false ;
	}

	bool ret = cpu_reset( dev, 1 ) ;
	
	if ( ret ) {
		while ( !feof(firm) ) {
			char line[1024] ;
			fgets( line, 1024, firm ) ;
			if ( ! firmware_line( dev, line ) ) {
				ret = false ;
				break ;
			}
		}
	}

	ret = cpu_reset( dev, 0 ) && ret ; // always do

	fclose( firm ) ;
	return ret ;
}

bool firmware_line( libusb_device_handle * dev, char * line ) {
	char c;
	int flength ;
	int faddr ;
	int ftype ;

	sscanf( s, "%c%02x%04x%02x", &c, &flength, %faddr, &type ) ;
	if ( c != ':' ) {
		return false ; // no initial ':'
	}
	switch ( ftype ) {
		case 0: // only type used
			return firmware_line_type0( dev, line+9, flength, faddr ) ;
		case 1:
			// ignore
			return true ;
		case 2:
			// unsupported extended address
			return false ;
		default:
			// unknown
			return true ;
	}
}

bool firmware_line_type0( libusb_device_handle * dev, char * line, int flength, int faddr ) {
	// note: line has header removed (9 chars)
	unsigned char data[256] ;
	unsigned char running_sum = flength + (faddr & 0xff) + (faddr>>8) + 0 ; // first 4 bytes (header)

	for ( int i = 0 ; i <= flength ; ++i ) { // 1 extra for checksum
		unsigned int b ;
		sscanf( "%02x", line+2*i, &b ) ;
		data[i] = b ;
		running_sum += data[i] ;
	}

	if ( running_sum != 0x00 ) {
		// bad checksum
		return false ;
	}

	return ram_write( dev, faddr, data, flength ) ;
}

bool FPGA_write( libusb_device_handle * dev, const char * filename ) {
	FILE * rbf = fopen( filename, "rb" ) ;
	if ( rbf == NULL ) {
		SoapySDR::logf(SOAPY_SDR_ERROR, "Cannot open FPGA file -- %s", filename);
		return false ;
	}

	bool ret = FPGA_control( dev, FL_BEGIN ) && FPGA_packet( dev, rbf ) ;

	ret = FPGA_control( dev, FL_END ) && ret ; // run anyways

	fclose( rbf ) ;
	return ret ;
}

bool FPGA_packet( libusb_device_handle * dev, FILE * rbf ) {
	unsigned char buffer[ MAX_EP4_PACKET_SIZE ];
	while ( (size_t length = fread( buffer, 1, sizeof(buffer), rbf )) > 0 ) {
		if ( ! bulk_write_EP( dev, QS1R_EP2, buffer, length ) ) {
			// Error writing FPGA
			return false ;
		}
	}
	return true ;
}

bool FPGA_control( libusb_device_handle * dev, int state ) {
	return libusb_control_transfer( dev, VRT_VENDOR_OUT,
             VRQ_FPGA_LOAD, 0, state, 0, 0,
             USB_TIMEOUT_CONTROL ) == 0 ;
}

bool bulk_write_EP( libusb_device_handle * dev, int ep, unsigned char * buffer, int length )
{
	int transfered ;
	if ( libusb_bulk_transfer( dev, ep, buffer, length, &transfered, USB_TIMEOUT_BULK ) != 0 ) {
		// error
		libusb_clear_halt( dev, ep & 0x0F ) ;
		return false ;
	}
	return transfered==length ;
}

// Strangely Covinton's code uses a special code for this. We'll try standard multibus read
bool QS1R_firmware_sn( libusb_device_handle * dev, char * sn, int sn_leng)
{
	uint32_t value ;
	if ( QS1R_read_multibus( dev, DDC_VERSION_REG, &value ) ) {
		snprintf( sn, sn_leng, "%u", value ) ;
		return true ;
	}
	return false ;
}

bool QS1R_read_multibus( libusb_device_handle * dev, int index, uint32_t * value)
{
	unsigned char * buf[4] ;
	if ( libusb_control_transfer( dev, VRT_VENDOR_IN, VRQ_MULTI_READ,
           index, 0, buf, 4, USB_TIMEOUT_CONTROL ) != 4 ) {
			   return false ;
   }
	*value = (uint32_t) buf[3] << 24 | (uint32_t) buf[2] << 16 | (uint32_t) buf[1] << 8 | (uint32_t) buf[0] ) ;
	return true ;
}

bool QS1R_write_multibus( libusb_device_handle * dev, int index, uint32_t value)
{
	unsigned char * buf[4] ;
	buf[0] = value & 0xFF ;
	buf[1] = (value>>8) & 0xFF ;
	buf[2] = (value>>16) & 0xFF ;
	buf[3] = (value>>24) & 0xFF ;
	if ( libusb_control_transfer( dev, VRT_VENDOR_OUT, VRQ_MULTI_WRITE,
           index, 0, buf, 4, USB_TIMEOUT_CONTROL ) != 4 ) {
			   return false ;
   }
	return true ;
}

bool QS1R_putbit( libusb_device_handle * dev, int index, int bit, int value) {
	uint32_t reg ;
	if ( QS1R_read_multibus( dev, index, &reg ) {
		if ( value == 0 ) {
			clearB( reg, bit ) ;
		} else {
			setB( reg, bit ) ;
		}
		return QS1R_write_multibus( dev, index, reg ) ;
	}
	return false ;
}

bool QS1R_getbit( libusb_device_handle * dev, int index, int bit, int * value) {
	uint32_t reg ;
	if ( QS1R_read_multibus( dev, index, &reg ) {
		*value = getB( reg, bit ) ;
		return true ;
	}
	return false ;
}
