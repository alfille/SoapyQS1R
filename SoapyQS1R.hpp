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

#pragma once
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <set>
extern "C" {
    #include <libusb.h>
    #include <math.h>
    #include <string.h>
}
#include <thread>

// Bit macros
// 32 bit for QS1R registers
#define long1 ( (uint32_t) 1 )
#define setB( var, b ) (var) |= (long1 << (b))
#define clearB( var, b ) (var) &= ~(long1 << (b))
#define getB( var, b ) (((var) & (long1 << (b))) ? 1:0)

enum QS1R_Format {
    QS1R_FORMAT_FLOAT32 =0,
    QS1R_FORMAT_INT16   =1,
    QS1R_FORMAT_INT8    =2,
    QS1R_FORMAT_FLOAT64 =3,
};

#define DDC_VERSION_REG 0x00

#define DDC_CONTROL_REG0 0x01
typedef enum {
    DAC_BYPASS, //1=bypass DAC audio
    DAC_EXT_MUTE_ENABLE, //1=mute external input
    WB_BYPASS, // 1=wide-band bypass (data on EP8)
    DAC_CLOCK_SELECT, // 0=24kSPS 1=48kSPS
    MASTER_RESET = 31,
} DDC_CONTROL0 ;

#define DDC_CONTROL_REG1 0x02
typedef enum {
    ADC_PGA_GAIN, // 0=low, 1=high
    ADC_RANDOMIZER_ENABLE,
    ADC_DITHER_ENABLE,
} DDC_CONTROL1 ;

#define DDC_SAMPLE_RATE_REG 0x03

#define GPIO_CONTROL_REG 0x04
// bits 0-14
// 1=input, 0=output

#define GPIO_STATE_REG 0x05
// bits 0-14

#define RFB_CONTROL_REG 0x06
// bits 0-11
// 1=input, 0=output

#define RFB_STATE_REG 0x07
// bits 0-11

#define EXT_CONTROL_REG 0x08
// bits 0-19
// 1=input, 0=output

#define EXT_STATE_REG 0x09
// bits 0-19

#define DDC_FREQ_REG 0x0A

#define REG32 ((double) 4294967296)
#define ENCFREQ ((double) 125000000)

//#define DDC_FREQ( freq_hz ) ( (freq_hz)* 4294967296 / 125000000 )
// Note -- added frequency correction and rounding correction
// use signed constants since correction can be negative
#define DDC_FREQ( freq_hz ) lround( REG32 * (freq_hz) / (ENCFREQ + _freq_corr) ) 

#define FX2_RAM_RESET           0xE600
#define FX2_WRITE_RAM_REQ       0xA0 

/* Vendor Request Types */
#define VRT_VENDOR_IN           0xC0 //LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_IN
#define VRT_VENDOR_OUT          0x40 // LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE | LIBUSB_ENDPOINT_OUT

/* Vendor In Commands */
#define VRQ_I2C_READ            0x81    // wValueL: i2c address; length: how much to read
#define VRQ_SPI_READ            0x82    // wValue: optional header bytes
// wIndexH: enables
// wIndexL: format
// len: how much to read

#define VRQ_SN_READ             0x83

#define VRQ_EEPROM_TYPE_READ    0x84
#define VRQ_I2C_SPEED_READ      0x85
#define VRQ_MULTI_READ          0x86
#define VRQ_DEBUG_READ          0x87

/* Vendor Out Commands */
#define VRQ_FPGA_LOAD           0x02
#define FL_BEGIN                0x00
#define FL_XFER                 1
#define FL_END                  2

#define VRQ_FPGA_SET_RESET      0x04    // wValueL: {0,1}
#define VRQ_MULTI_WRITE         0x05
#define VRQ_REQ_I2C_WRITE       0x08    // wValueL: i2c address; data: data
#define VRQ_REQ_SPI_WRITE       0x09    // wValue: optional header bytes
// wIndexH: enables
// wIndexL: format
// len: how much to write

#define VRQ_I2C_SPEED_SET       0x0B    // wValueL: {0,1}
#define VRQ_CPU_SPEED_SET       0x0C    // wValueL: {0, 1, 2}
#define VRQ_EP_RESET            0x0D

#define USB_TIMEOUT_CONTROL     500
#define USB_TIMEOUT_BULK        11500

#define MAX_EP0_PACKET_SIZE     64
#define MAX_EP4_PACKET_SIZE     1024

// EndPoint 2 output
#define QS1R_EP2 0x02

// EndPoint 4 output
#define QS1R_EP4 0x04

// EndPoint 6 input
#define QS1R_EP6 0x86

// EndPoint 8 input
#define QS1R_EP8 0x88

#define QS1R_PID 0x0008
#define QS1R_VID 0xFFFE

#define FX2LP_PID 0x8613
#define FX2LP_VID 0x04B4

/*!
 * The session object manages qs1r_init/exit
 * with a process-wide reference count.
 */
class SoapyQS1RSession
{
public:
    SoapyQS1RSession(void);
    ~SoapyQS1RSession(void);

    static libusb_context * qs1r_context ;
    static size_t sessionCount ;
    static std::mutex sessionMutex;
};

class SoapyQS1R : public SoapySDR::Device
{
public:
    SoapyQS1R( const SoapySDR::Kwargs & args );

    ~SoapyQS1R( void );


    /*******************************************************************
     * Identification API
     ******************************************************************/

    std::string getDriverKey( void ) const;


    std::string getHardwareKey( void ) const;


    SoapySDR::Kwargs getHardwareInfo( void ) const;


    /*******************************************************************
     * Channels API
     ******************************************************************/

    size_t getNumChannels( const int ) const;


    bool getFullDuplex( const int direction, const size_t channel ) const;


    /*******************************************************************
     * Stream API
     ******************************************************************/

    std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

    std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;
    double getReferenceClockRate( void ) const ;
    SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

    SoapySDR::Stream *setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels = std::vector<size_t>(),
        const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


    void closeStream( SoapySDR::Stream *stream );


    size_t getStreamMTU( SoapySDR::Stream *stream ) const;


    int activateStream(
            SoapySDR::Stream *stream,
            const int flags = 0,
            const long long timeNs = 0,
            const size_t numElems = 0 );


    int deactivateStream(
            SoapySDR::Stream *stream,
            const int flags = 0,
            const long long timeNs = 0 );


    int readStream(
            SoapySDR::Stream *stream,
            void * const *buffs,
            const size_t numElems,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000 );

    int writeStream(
            SoapySDR::Stream *stream,
            const void * const *buffs,
            const size_t numElems,
            int &flags,
            const long long timeNs = 0,
            const long timeoutUs = 100000);

    int readStreamStatus(
            SoapySDR::Stream *stream,
            size_t &chanMask,
            int &flags,
            long long &timeNs,
            const long timeoutUs
    );

    int acquireReadBuffer(
            SoapySDR::Stream *stream,
            size_t &handle,
            const void **buffs,
            int &flags,
            long long &timeNs,
            const long timeoutUs = 100000);

    void releaseReadBuffer(
            SoapySDR::Stream *stream,
            const size_t handle);

    int acquireWriteBuffer(
            SoapySDR::Stream *stream,
            size_t &handle,
            void **buffs,
            const long timeoutUs = 100000);

    void releaseWriteBuffer(
            SoapySDR::Stream *stream,
            const size_t handle,
            const size_t numElems,
            int &flags,
            const long long timeNs = 0);

    size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

    int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

    /*******************************************************************
     * Settings API
     ******************************************************************/

    SoapySDR::ArgInfoList getSettingInfo(void) const;


    void writeSetting(const std::string &key, const std::string &value);


    std::string readSetting(const std::string &key) const;


    /*******************************************************************
     * Antenna API
     ******************************************************************/

    std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;


    void setAntenna( const int direction, const size_t channel, const std::string &name );


    std::string getAntenna( const int direction, const size_t channel ) const;


    /*******************************************************************
     * Frontend corrections API
     ******************************************************************/


    /*******************************************************************
     * Gain API
     ******************************************************************/


    /*******************************************************************
     * Frequency API
     ******************************************************************/

    void setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


    double getFrequency( const int direction, const size_t channel, const std::string &name ) const;


    SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;


    std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;


    SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;


    /*******************************************************************
     * Sample Rate API
     ******************************************************************/

    void setSampleRate( const int direction, const size_t channel, const double rate );


    double getSampleRate( const int direction, const size_t channel ) const;


    std::vector<double> listSampleRates( const int direction, const size_t channel ) const;


    void setBandwidth( const int direction, const size_t channel, const double bw );


    double getBandwidth( const int direction, const size_t channel ) const;


    std::vector<double> listBandwidths( const int direction, const size_t channel ) const;

    /*******************************************************************
     * Qs1R callback
     ******************************************************************/
//    int hackrf_tx_callback( int8_t *buffer, int32_t length );


//    int hackrf_rx_callback( int8_t *buffer, int32_t length );
    std::thread _rx_async_thread;
    void rx_callback(unsigned char *buf, uint32_t len);



private:

    double _sample_rate ;
    double _bandwidth ;
    int _samples_per_read ;

    libusb_device_handle * _dev ;


    std::string _serial;

    double _RX_FREQ;
    double _freq_corr; // frequency correction
    std::string _antenna ;

    /// Mutex protecting all use of the hackrf device _dev and other instance variables.
    /// Most of the hackrf API is thread-safe because it only calls libusb, however
    /// the activateStream() method in this library can close and re-open the device,
    /// so all use of _dev must be protected
    mutable std::mutex  _device_mutex;
    std::mutex  _buf_mutex;
    std::condition_variable _buf_cond;

    SoapyQS1RSession _sess;

    bool qs1r_by_serial( const char * serial ) ;
    bool qs1r_by_index( const char * index ) ;
    bool openDevice( const SoapySDR::Kwargs &args ) ;
    bool ram_write( int ram_address, unsigned char * buffer, int length ) ;
    bool cpu_reset( unsigned char state ) ;
    bool bulk_write_EP( int ep, unsigned char * buffer, int length ) ;
    bool firmware_read_sn( uint32_t * value ) ;
    bool FPGA_read_sn( uint32_t * value) const ;
    bool firmware_write( const char * filename ) ;
    bool firmware_line( char * line ) ;
    bool firmware_line_type0( char * line, int flength, int faddr ) ;
    bool FPGA_write( const char * filename ) ;
    bool FPGA_control( int state ) ;
    bool FPGA_packet( FILE * rbf ) ;
    bool configure_device( void ) ;
    bool read_multibus( int addr, uint32_t * value) const ;
    bool write_multibus( int addr, uint32_t value) ;
    bool DDC_putbit( int index, int bit, int value) ;
    bool DDC_getbit( int index, int bit, int * value) const ;

    typedef void(*qs1r_read_async_cb_t)(unsigned char *buf, uint32_t len, void *ctx);
    int qs1r_cancel_async(void);
    int qs1r_read_async(qs1r_read_async_cb_t cb, uint32_t buf_num, uint32_t buf_len) ;
    int _qs1r_free_async_buffers(void);
    int _qs1r_alloc_async_buffers(void);
    int qs1r_wait_async(qs1r_read_async_cb_t cb);
    static void LIBUSB_CALL _libusb_callback(struct libusb_transfer *xfer);
    //int qs1r_close(void);
    uint16_t qs1r_demod_read_reg(uint8_t page, uint16_t addr, uint8_t len);
    int qs1r_demod_write_reg(uint8_t page, uint16_t addr, uint16_t val, uint8_t len);
    int qs1r_write_reg(uint8_t block, uint16_t addr, uint16_t val, uint8_t len);
    int qs1r_write_array(uint8_t block, uint16_t addr, uint8_t *array, uint8_t len);
    int qs1r_read_array(uint8_t block, uint16_t addr, uint8_t *array, uint8_t len);
    bool _use_zerocopy ;
    int _async_cancel ; // should be bool, but libusb_handle_events_timeout_completed function wants an int
    enum qs1r_async_status {
        QS1R_INACTIVE = 0,
        QS1R_CANCELING,
        QS1R_RUNNING
    } _async_status ;
    bool _dev_lost ;
    bool _iqSwap;
    uint32_t _xfer_errors;
    uint32_t _xfer_buf_num;
    uint32_t _xfer_buf_len;
    std::vector<unsigned char *> _xfer_buf;
    std::vector<struct libusb_transfer *> _xfer;
    qs1r_read_async_cb_t _callback;
    size_t numBuffers;
    size_t bufferLength;
    size_t _asyncBuffs;
    std::atomic<long long> _ticks;
    std::atomic<size_t> _buf_count;
    std::atomic<bool> _overflowEvent;
    struct Buffer
    {
        unsigned long long tick;
        std::vector<signed char> data;
    };
    std::vector<Buffer> _buffs;
    size_t  _buf_head;
    size_t  _buf_tail;
    signed char *_currentBuff;
    size_t _currentHandle;
    std::atomic<bool> _resetBuffer;
    size_t _bufferedElems;
    long long _bufTicks;

    typedef enum RXFormat
    {
        RX_FORMAT_FLOAT32, RX_FORMAT_INT16, RX_FORMAT_INT8
    } RXFormat;
    RXFormat _rxFormat;
    std::vector<std::complex<float> > _lut_32f;
    std::vector<std::complex<float> > _lut_swap_32f;
    std::vector<std::complex<int16_t> > _lut_16i;
    std::vector<std::complex<int16_t> > _lut_swap_16i;

    void rx_async_operation(void);

};
