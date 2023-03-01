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


bool SoapyQS1R::configure_device( void ) {
    return (libusb_set_configuration( _dev, 1 )==0)         &&(printf("config\n")>0)
        && (libusb_claim_interface( _dev, 0 )==0)           &&(printf("claim\n")>0)
        && (libusb_set_interface_alt_setting( _dev, 0, 0 )==0)&&(printf("alt\n")>0) ;
    }

bool SoapyQS1R::ram_write( int ram_address, unsigned char * buffer, int length ) {
    // write the the FX2 CPU's ram
    int this_size = MAX_EP0_PACKET_SIZE ;
    int this_address = ram_address;
    unsigned char * this_buffer = buffer ;

    for ( int remaining = length ; remaining > 0 ; remaining -= this_size ) {
        if (this_size > remaining ) {
            this_size = remaining ;
        } 
        int sent = libusb_control_transfer( _dev,
            VRT_VENDOR_OUT,
            FX2_WRITE_RAM_REQ,
            this_address,
            0,
            this_buffer,
            this_size,
            USB_TIMEOUT_CONTROL );
        //printf("RAM WRITE buf=%02x... tried-%d, success=%d\n",this_buffer[0],this_size,sent);
        if ( sent != this_size ) {
            return false ;
        }
        this_address += sent ;
        this_buffer += sent ;
    }
    return true ;
}

bool SoapyQS1R::cpu_reset( unsigned char state ) {
    // State 1=reset, 0=normal
    return ram_write( FX2_RAM_RESET, &state, 1 ) &&(printf("reset=%d\n",(int)state)>0);
}

bool SoapyQS1R::firmware_write( const char * filename ) {
    FILE * firm = fopen( filename, "r" ) ;
    uint32_t sn ;
    firmware_read_sn( &sn ) && printf("sn:%d\n",sn);
    if ( firm == NULL ) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "Cannot open firmware file -- %s", filename);
        return false ;
    }

    bool ret = cpu_reset( 1 ) ;
    
    if ( ret ) {
        while ( !feof(firm) ) {
            char line[1024] ;
            fgets( line, 1024, firm ) ;
            //printf("%s\n",line);
            if ( ! firmware_line( line ) ) {
                ret = false ;
                break ;
            }
        }
    }

    ret = cpu_reset( 0 ) && ret ; // always do

    fclose( firm ) ;
    return ret ;
}

bool SoapyQS1R::firmware_line( char * line ) {
    char c;
    unsigned int flength ;
    unsigned int faddr ;
    unsigned int ftype ;

    sscanf( line, "%c%02x%04x%02x", &c, &flength, &faddr, &ftype ) ;
    //printf("%c %d %d %d\n",c,flength,faddr, ftype);
    if ( c != ':' ) {
        return false ; // no initial ':'
    }
    switch ( ftype ) {
        case 0: // only type used
            return firmware_line_type0( line+9, flength, faddr ) ;
        case 1:
            // ignore
            return true ;
        default:
            // unsupported extended address
            return false ;
    }
}

bool SoapyQS1R::firmware_line_type0( char * line, int flength, int faddr ) {
    // note: line has header removed (9 chars)
    unsigned char data[256] ;
    unsigned char running_sum = flength + (faddr & 0xff) + (faddr>>8) + 0 ; // first 4 bytes (header)
    
    //printf("Now line <%s>\n");

    for ( int i = 0 ; i <= flength ; ++i ) { // 1 extra for checksum
        unsigned int b ;
        sscanf( line+2*i, "%02x", &b ) ;
        data[i] = b ;
        running_sum += data[i] ;
        //printf("Char %02x Int %d Running %02x\n",b,(int)data[i],running_sum);
    }

    if ( running_sum != 0x00 ) {
        // bad checksum
        return false ;
    }

    return ram_write( faddr, data, flength ) ;
}

bool SoapyQS1R::FPGA_write( const char * filename ) {
    FILE * rbf = fopen( filename, "rb" ) ;
    printf("FPGA\n");
    if ( rbf == NULL ) {
        SoapySDR::logf(SOAPY_SDR_ERROR, "Cannot open FPGA file -- %s", filename);
        return false ;
    }
    printf("opened file\n");
    bool ret = FPGA_control( FL_BEGIN ) && (printf("BEGIN\n")>0) && FPGA_packet( rbf ) && (printf("PACKET\n")>0) ;

    ret = FPGA_control( FL_END ) && (printf("END\n")>0) && ret ; // run anyways

    fclose( rbf ) ;
    return ret ;
}

bool SoapyQS1R::FPGA_packet( FILE * rbf ) {
    unsigned char buffer[ MAX_EP4_PACKET_SIZE ];
    size_t length ;
    while ( (length = fread( buffer, 1, sizeof(buffer), rbf )) > 0 ) {
        //printf("Read %lu\n",length);
        if ( ! bulk_write_EP( QS1R_EP4, buffer, length ) ) {
            // Error writing FPGA
            fprintf( stderr, "FPGA error writing line %s\n", buffer ) ;
            return false ;
        }
    }
    return true ;
}

bool SoapyQS1R::FPGA_control( int state ) {
    unsigned char dummy[1] ;
    printf("Try Control\n");
    int ret = libusb_control_transfer( _dev, VRT_VENDOR_OUT,
        VRQ_FPGA_LOAD, 0, state, dummy, 0,
        USB_TIMEOUT_CONTROL );
    printf("Control %d, %s\n",ret,libusb_error_name(ret)) ;
    return true ;
}

bool SoapyQS1R::bulk_write_EP( int ep, unsigned char * buffer, int length )
{
    int transfered ;
    int ret = libusb_bulk_transfer( _dev, ep, buffer, length, &transfered, USB_TIMEOUT_BULK ) ;
    if ( ret == 0 ) {
        // good?
        return transfered==length ;
    }
    // error
    fprintf(stderr,"Bulk write error %s\n",libusb_error_name(ret) );
    libusb_clear_halt( _dev, ep & 0x0F ) ;
    return false ;
}

bool SoapyQS1R::firmware_read_sn( uint32_t * value )
{
    unsigned char buf[4] ;
    int ret = libusb_control_transfer( _dev, VRT_VENDOR_IN, VRQ_SN_READ,
       0, 0, buf, 4, USB_TIMEOUT_CONTROL ) ;
    if ( ret != 4 ) {
        fprintf(stderr,"Cannot read SN %d=%s\n",ret,libusb_error_name(ret));
        return false ;
    }
    * value = (uint32_t) buf[3] << 24 | (uint32_t) buf[2] << 16 | (uint32_t) buf[1] << 8 | (uint32_t) buf[0] ;
    return true ;
}

bool SoapyQS1R::read_multibus( int index, uint32_t * value) const
{
    unsigned char buf[4] ;
    if ( libusb_control_transfer( _dev, VRT_VENDOR_IN, VRQ_MULTI_READ,
       index, 0, buf, 4, USB_TIMEOUT_CONTROL ) != 4 ) {
       return false ;
    }
    *value = (uint32_t) buf[3] << 24 | (uint32_t) buf[2] << 16 | (uint32_t) buf[1] << 8 | (uint32_t) buf[0] ;
    return true ;
}

bool SoapyQS1R::FPGA_read_sn( uint32_t * value) const
{
    return read_multibus( DDC_VERSION_REG, value ) ;
}

bool SoapyQS1R::write_multibus( int index, uint32_t value)
{
    unsigned char buf[4] ;
    buf[0] = value & 0xFF ;
    buf[1] = (value>>8) & 0xFF ;
    buf[2] = (value>>16) & 0xFF ;
    buf[3] = (value>>24) & 0xFF ;
    if ( libusb_control_transfer( _dev, VRT_VENDOR_OUT, VRQ_MULTI_WRITE,
       index, 0, buf, 4, USB_TIMEOUT_CONTROL ) != 4 ) {
       return false ;
    }
    return true ;
}

bool SoapyQS1R::DDC_putbit( int index, int bit, int value) {
    uint32_t reg ;
    if ( read_multibus( index, &reg )) {
        if ( value == 0 ) {
            clearB( reg, bit ) ;
        } else {
            setB( reg, bit ) ;
        }
        return write_multibus( index, reg ) ;
    }
    return false ;
}

bool SoapyQS1R::DDC_getbit( int index, int bit, int * value) const {
    uint32_t reg ;
    if ( read_multibus( index, &reg )) {
        *value = getB( reg, bit ) ;
        return true ;
    }
    return false ;
}

//-----------------------

/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <errno.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#ifndef _WIN32
#include <unistd.h>
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

#include <libusb.h>

/*
 * All libusb callback functions should be marked with the LIBUSB_CALL macro
 * to ensure that they are compiled with the same calling convention as libusb.
 *
 * If the macro isn't available in older libusb versions, we simply define it.
 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

#define DEFAULT_BUF_NUMBER  15
#define DEFAULT_BUF_LENGTH  (16 * 32 * 512)

#define CTRL_IN     (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_IN)
#define CTRL_OUT    (LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_ENDPOINT_OUT)
#define CTRL_TIMEOUT    300
#define BULK_TIMEOUT    0

#define EEPROM_ADDR 0xa0

enum usb_reg {
    USB_SYSCTL      = 0x2000,
    USB_CTRL        = 0x2010,
    USB_STAT        = 0x2014,
    USB_EPA_CFG     = 0x2144,
    USB_EPA_CTL     = 0x2148,
    USB_EPA_MAXPKT      = 0x2158,
    USB_EPA_MAXPKT_2    = 0x215a,
    USB_EPA_FIFO_CFG    = 0x2160,
};

int SoapyQS1R::qs1r_read_array(uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
    int r;
    uint16_t index = (block << 8);

    r = libusb_control_transfer(_dev, CTRL_IN, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);
#endif
    return r;
}

int SoapyQS1R::qs1r_write_array(uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
    int r;
    uint16_t index = (block << 8) | 0x10;

    r = libusb_control_transfer(_dev, CTRL_OUT, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);
#endif
    return r;
}

int SoapyQS1R::qs1r_write_reg(uint8_t block, uint16_t addr, uint16_t val, uint8_t len)
{
    int r;
    unsigned char data[2];

    uint16_t index = (block << 8) | 0x10;

    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;

    data[1] = val & 0xff;

    r = libusb_control_transfer(_dev, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    return r;
}

uint16_t SoapyQS1R::qs1r_demod_read_reg(uint8_t page, uint16_t addr, uint8_t len)
{
    int r;
    unsigned char data[2];

    uint16_t index = page;
    uint16_t reg;
    addr = (addr << 8) | 0x20;

    r = libusb_control_transfer(_dev, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    reg = (data[1] << 8) | data[0];

    return reg;
}

int SoapyQS1R::qs1r_demod_write_reg(uint8_t page, uint16_t addr, uint16_t val, uint8_t len)
{
    int r;
    unsigned char data[2];
    uint16_t index = 0x10 | page;
    addr = (addr << 8) | 0x20;

    if (len == 1)
        data[0] = val & 0xff;
    else
        data[0] = val >> 8;

    data[1] = val & 0xff;

    r = libusb_control_transfer(_dev, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

    if (r < 0)
        fprintf(stderr, "%s failed with %d\n", __FUNCTION__, r);

    qs1r_demod_read_reg(0x0a, 0x01, 1);

    return (r == len) ? 0 : -1;
}

/*
int SoapyQS1R::qs1r_close(void)
{
    // block until all async operations have been completed (if any)
    while (QS1R_INACTIVE != _async_status) {
#ifdef _WIN32
        Sleep(1);
#else
        usleep(1000);
#endif
    }

}
*/

void LIBUSB_CALL SoapyQS1R::_libusb_callback(struct libusb_transfer *xfer)
{
    SoapyQS1R * sq = (SoapyQS1R *) xfer->user_data;
    
    if (LIBUSB_TRANSFER_COMPLETED == xfer->status) {
        if (sq->_callback)
            sq->_callback(xfer->buffer, xfer->actual_length, sq);

        libusb_submit_transfer(xfer); /* resubmit transfer */
        sq->_xfer_errors = 0;
    } else if (LIBUSB_TRANSFER_CANCELLED != xfer->status) {
#ifndef _WIN32
        if (LIBUSB_TRANSFER_ERROR == xfer->status)
            sq->_xfer_errors++;

        if (sq->_xfer_errors >= sq->_xfer_buf_num ||
            LIBUSB_TRANSFER_NO_DEVICE == xfer->status) {
#endif
            sq->_dev_lost = true;
            sq->qs1r_cancel_async();
            fprintf(stderr, "cb transfer status: %d, "
                "canceling...\n", xfer->status);
#ifndef _WIN32
        }
#endif
    }
}

int SoapyQS1R::qs1r_wait_async(qs1r_read_async_cb_t cb)
{
    return qs1r_read_async(cb, 0, 0);
}

int SoapyQS1R::_qs1r_alloc_async_buffers(void)
{
    unsigned int i;

    if (!_dev)
        return -1;

    _xfer.resize(_xfer_buf_num);

    for(i = 0; i < _xfer.size(); ++i) {
        _xfer[i] = libusb_alloc_transfer(0);
    }

    if (! _xfer_buf.empty() )
        return -2;

    _xfer_buf.assign(_xfer_buf_num,0);

#if defined(ENABLE_ZEROCOPY) && defined (__linux__) && LIBUSB_API_VERSION >= 0x01000105
    fprintf(stderr, "Allocating %d zero-copy buffers\n", _xfer_buf_num);

    _use_zerocopy = true;
    for (i = 0; i < _xfer_buf.size(); ++i) {
        _xfer_buf[i] = libusb_dev_mem_alloc(_dev, _xfer_buf_len);

        if (_xfer_buf[i]) {
            /* Check if Kernel usbfs mmap() bug is present: if the
             * mapping is correct, the buffers point to memory that
             * was memset to 0 by the Kernel, otherwise, they point
             * to random memory. We check if the buffers are zeroed
             * and otherwise fall back to buffers in userspace.
             */
            if (_xfer_buf[i][0] || memcmp(_xfer_buf[i],
                              _xfer_buf[i] + 1,
                              _xfer_buf_len - 1)) {
                fprintf(stderr, "Detected Kernel usbfs mmap() "
                        "bug, falling back to buffers "
                        "in userspace\n");
                _use_zerocopy = false;
                break;
            }
        } else {
            fprintf(stderr, "Failed to allocate zero-copy "
                    "buffer for transfer %d\nFalling "
                    "back to buffers in userspace\n", i);
            _use_zerocopy = false;
            break;
        }
    }

    /* zero-copy buffer allocation failed (partially or completely)
     * we need to free the buffers again if already allocated */
    if (!_use_zerocopy) {
        for (i = 0; i < _xfer_buf.size(); ++i) {
            if (_xfer_buf[i])
                libusb_dev_mem_free(_dev,
                            _xfer_buf[i],
                            _xfer_buf_len);
        }
    }
#endif

    /* no zero-copy available, allocate buffers in userspace */
    if (!_use_zerocopy) {
        for (i = 0; i < _xfer_buf.size(); ++i) {
            _xfer_buf[i] = (unsigned char *) malloc(_xfer_buf_len);

            if (!_xfer_buf[i])
                return -ENOMEM;
        }
    }

    return 0;
}

int SoapyQS1R::_qs1r_free_async_buffers(void)
{
    unsigned int i;

    if (!_dev)
        return -1;

    for(i = 0; i < _xfer.size(); ++i) {
        if (_xfer[i]) {
            libusb_free_transfer(_xfer[i]);
        }
    }
    _xfer.clear() ;

    for (i = 0; i < _xfer.size(); ++i) {
        if (_xfer_buf[i]) {
            if (_use_zerocopy) {
#if defined (__linux__) && LIBUSB_API_VERSION >= 0x01000105
                libusb_dev_mem_free(_dev,
                            _xfer_buf[i],
                            _xfer_buf_len);
#endif
            } else {
                free(_xfer_buf[i]);
            }
        }
    }
    _xfer_buf.clear();

    return 0;
}

int SoapyQS1R::qs1r_read_async(qs1r_read_async_cb_t cb, uint32_t buf_num, uint32_t buf_len)
{
    unsigned int i;
    int r = 0;
    struct timeval tv = { 1, 0 };
    struct timeval zerotv = { 0, 0 };
    enum qs1r_async_status next_status = QS1R_INACTIVE;

    if (!_dev)
        return -1;

    if (QS1R_INACTIVE != _async_status)
        return -2;

    _async_status = QS1R_RUNNING;
    _async_cancel = 0;

    _callback = cb;

    if (buf_num > 0) {
        _xfer_buf_num = buf_num;
    } else {
        _xfer_buf_num = DEFAULT_BUF_NUMBER;
    }

    if (buf_len > 0 && buf_len % 512 == 0) { /* len must be multiple of 512 */
        _xfer_buf_len = buf_len;
    } else {
        _xfer_buf_len = DEFAULT_BUF_LENGTH;
    }

    _qs1r_alloc_async_buffers();

    for(i = 0; i < _xfer.size(); ++i) {
        libusb_fill_bulk_transfer(_xfer[i],
                      _dev,
                      0x81,
                      _xfer_buf[i],
                      _xfer_buf_len,
                      _libusb_callback,
                      this,
                      BULK_TIMEOUT);

        r = libusb_submit_transfer(_xfer[i]);
        if (r < 0) {
            fprintf(stderr, "Failed to submit transfer %i\n"
                    "Please increase your allowed " 
                    "usbfs buffer size with the "
                    "following command:\n"
                    "echo 0 > /sys/module/usbcore"
                    "/parameters/usbfs_memory_mb\n", i);
            _async_status = QS1R_CANCELING;
            break;
        }
    }

    while (QS1R_INACTIVE != _async_status) {
        r = libusb_handle_events_timeout_completed(SoapyQS1RSession::qs1r_context, &tv,
                               &_async_cancel);
        if (r < 0) {
            /*fprintf(stderr, "handle_events returned: %d\n", r);*/
            if (r == LIBUSB_ERROR_INTERRUPTED) /* stray signal */
                continue;
            break;
        }

        if (QS1R_CANCELING == _async_status) {
            next_status = QS1R_INACTIVE;

            for(i = 0; i < _xfer.size(); ++i) {
                if (!_xfer[i])
                    continue;

                if (LIBUSB_TRANSFER_CANCELLED !=
                        _xfer[i]->status) {
                    r = libusb_cancel_transfer(_xfer[i]);
                    /* handle events after canceling
                     * to allow transfer status to
                     * propagate */
#ifdef _WIN32
                    Sleep(1);
#endif
                    libusb_handle_events_timeout_completed(SoapyQS1RSession::qs1r_context,
                                           &zerotv, NULL);
                    if (r < 0)
                        continue;

                    next_status = QS1R_CANCELING;
                }
            }

            if (_dev_lost || QS1R_INACTIVE == next_status) {
                /* handle any events that still need to
                 * be handled before exiting after we
                 * just cancelled all transfers */
                libusb_handle_events_timeout_completed(SoapyQS1RSession::qs1r_context,
                                       &zerotv, NULL);
                break;
            }
        }
    }

    _qs1r_free_async_buffers();

    _async_status = next_status;

    return r;
}

int SoapyQS1R::qs1r_cancel_async(void)
{
    if (!_dev)
        return -1;

    /* if streaming, try to cancel gracefully */
    if (QS1R_RUNNING == _async_status) {
        _async_status = QS1R_CANCELING;
        _async_cancel = 1;
        return 0;
    }

    /* if called while in pending state, change the state forcefully */
#if 0
    if (QS1R_INACTIVE != _async_status) {
        _async_status = QS1R_INACTIVE;
        return 0;
    }
#endif
    return -2;
}

