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
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <set>
#include <libusb.h>

#define BUF_LEN			262144
#define BUF_NUM			15
#define BYTES_PER_SAMPLE	4
#define QS1R_RX_VGA_MAX_DB 62
#define QS1R_RX_LNA_MAX_DB 40
#define QS1R_AMP_MAX_DB 14

enum QS1R_Format {
	QS1R_FORMAT_FLOAT32	=0,
	QS1R_FORMAT_INT16	=1,
	QS1R_FORMAT_INT8	=2,
	QS1R_FORMAT_FLOAT64 =3,
};

typedef enum {
	HACKRF_TRANSCEIVER_MODE_OFF = 0,
	HACKRF_TRANSCEIVER_MODE_RX = 1,
	HACKRF_TRANSCEIVER_MODE_TX = 2,
} HackRF_transceiver_mode_t;

std::set<std::string> &HackRF_getClaimedSerials(void);

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
typedef enum {
	BW20k = 25000,
	BW40k = 50000,
	BW100k = 125000,
	BW200k = 250000,
	BW500k = 625000,
	BW1_0m = 1250000,
	BW1_2m = 1562500,
	BW2_0m = 2500000,
} DDC_SAMPLE_RATE ;

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
//#define DDC_FREQ( freq_hz ) ( (freq_hz)* 4294967296 / 125000000 )
// 2^32 * freq / 125M (clock freq)
// can reduce numerator and denominator by 64
#define DDC_FREQ( freq_hz ) ( (freq_hz)* 67108864 / 1953125 ) 

#define FX2_RAM_RESET			0xE600
#define FX2_WRITE_RAM_REQ		0xA0 

/* Vendor Request Types */
#define VRT_VENDOR_IN			0xC0
#define VRT_VENDOR_OUT			0x40

/* Vendor In Commands */
#define	VRQ_I2C_READ			0x81	// wValueL: i2c address; length: how much to read
#define	VRQ_SPI_READ			0x82	// wValue: optional header bytes
// wIndexH:	enables
// wIndexL:	format
// len: how much to read

#define VRQ_SN_READ     		0x83

#define VRQ_EEPROM_TYPE_READ	0x84
#define VRQ_I2C_SPEED_READ		0x85
#define VRQ_MULTI_READ			0x86
#define VRQ_DEBUG_READ			0x87

/* Vendor Out Commands */
#define VRQ_FPGA_LOAD			0x02
#define FL_BEGIN				0
#define FL_XFER					1
#define FL_END					2

#define VRQ_FPGA_SET_RESET		0x04	// wValueL: {0,1}
#define VRQ_MULTI_WRITE			0x05
#define VRQ_REQ_I2C_WRITE  		0x08	// wValueL: i2c address; data: data
#define VRQ_REQ_SPI_WRITE 		0x09	// wValue: optional header bytes
// wIndexH:	enables
// wIndexL:	format
// len: how much to write

#define VRQ_I2C_SPEED_SET  		0x0B  	// wValueL: {0,1}
#define VRQ_CPU_SPEED_SET		0x0C 	// wValueL: {0, 1, 2}
#define VRQ_EP_RESET			0x0D

#define USB_TIMEOUT_CONTROL		500
#define USB_TIMEOUT_BULK		1000

#define MAX_EP0_PACKET_SIZE		64
#define MAX_EP4_PACKET_SIZE		1024

// USB
extern libusb_context * qs1r_context ;

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

	bool hasDCOffsetMode( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Gain API
	 ******************************************************************/

	std::vector<std::string> listGains( const int direction, const size_t channel ) const;


	void setGainMode( const int direction, const size_t channel, const bool automatic );


	bool getGainMode( const int direction, const size_t channel ) const;


	void setGain( const int direction, const size_t channel, const double value );


	void setGain( const int direction, const size_t channel, const std::string &name, const double value );


	double getGain( const int direction, const size_t channel, const std::string &name ) const;


	SoapySDR::Range getGainRange( const int direction, const size_t channel, const std::string &name ) const;


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
 	 * HackRF callback
 	 ******************************************************************/
	int hackrf_tx_callback( int8_t *buffer, int32_t length );


	int hackrf_rx_callback( int8_t *buffer, int32_t length );




private:

	SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x1;
	SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x2;

	struct Stream {
		Stream(): opened(false), buf_num(BUF_NUM), buf_len(BUF_LEN), buf(nullptr),
				  buf_head(0), buf_tail(0), buf_count(0),
				  remainderHandle(-1), remainderSamps(0), remainderOffset(0), remainderBuff(nullptr),
				  format(QS1R_FORMAT_INT8) {}

		bool opened;
		uint32_t	buf_num;
		uint32_t	buf_len;
		int8_t		**buf;
		uint32_t	buf_head;
		uint32_t	buf_tail;
		uint32_t	buf_count;

		int32_t remainderHandle;
		size_t remainderSamps;
		size_t remainderOffset;
		int8_t* remainderBuff;
		uint32_t format;

		~Stream() { clear_buffers(); }
		void clear_buffers();
		void allocate_buffers();
	};

	struct RXStream: Stream {
		uint32_t vga_gain;
		uint32_t lna_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;

		bool overflow;
	};

	struct TXStream: Stream {
		uint32_t vga_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;
		bool bias;

		bool underflow;

		bool burst_end;
		int32_t burst_samps;
	} ;

	RXStream _rx_stream;
	TXStream _tx_stream;

	bool _auto_bandwidth;

	hackrf_device * _dev;
	std::string _serial;

	uint64_t _current_frequency;

	double _current_samplerate;

	uint32_t _current_bandwidth;

	uint8_t _current_amp;

	/// Mutex protecting all use of the hackrf device _dev and other instance variables.
	/// Most of the hackrf API is thread-safe because it only calls libusb, however
	/// the activateStream() method in this library can close and re-open the device,
	/// so all use of _dev must be protected
	mutable std::mutex	_device_mutex;
	std::mutex	_buf_mutex;
	std::condition_variable _buf_cond;

	HackRF_transceiver_mode_t _current_mode;

	SoapyQS1RSession _sess;
};
