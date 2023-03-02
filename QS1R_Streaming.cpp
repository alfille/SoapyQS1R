/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2015 Charles J. Cliffe
 * Copyright (c) 2015-2017 Josh Blum

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
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <algorithm> //min
#include <climits> //SHRT_MAX
#include <cstring> // memcpy

#define DEFAULT_BUFFER_LENGTH (16 * 32 * 512)
#define DEFAULT_NUM_BUFFERS 15
#define BYTES_PER_SAMPLE (2*sizeof(uint32_t))

std::vector<std::string> SoapyQS1R::getStreamFormats(const int direction, const size_t channel) const {
    if (direction == SOAPY_SDR_RX) {
        std::vector<std::string> formats;

        formats.push_back(SOAPY_SDR_CS8);
        formats.push_back(SOAPY_SDR_CS16);
        formats.push_back(SOAPY_SDR_CF32);

        return formats;
    } else {
        return SoapySDR::Device::getStreamFormats( direction, channel) ;
    }
}

std::string SoapyQS1R::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const {
    //check that direction is SOAPY_SDR_RX
     if (direction == SOAPY_SDR_RX) {
        fullScale = 1<<31;
        return SOAPY_SDR_CS32;
     } else {
         return SoapySDR::Device::getNativeStreamFormat( direction, channel, fullScale ) ;
    }
}

double SoapyQS1R::getReferenceClockRate( void ) const {
    return 125e6;
}

SoapySDR::ArgInfoList SoapyQS1R::getStreamArgsInfo(const int direction, const size_t channel) const {
    //check that direction is SOAPY_SDR_RX
     if (direction == SOAPY_SDR_RX) {
        SoapySDR::ArgInfoList streamArgs;

        SoapySDR::ArgInfo bufflenArg;
        bufflenArg.key = "bufflen";
        bufflenArg.value = std::to_string(DEFAULT_BUFFER_LENGTH);
        bufflenArg.name = "Buffer Size";
        bufflenArg.description = "Number of bytes per buffer, multiples of 512 only.";
        bufflenArg.units = "bytes";
        bufflenArg.type = SoapySDR::ArgInfo::INT;

        streamArgs.push_back(bufflenArg);

        SoapySDR::ArgInfo buffersArg;
        buffersArg.key = "buffers";
        buffersArg.value = std::to_string(DEFAULT_NUM_BUFFERS);
        buffersArg.name = "Ring buffers";
        buffersArg.description = "Number of buffers in the ring.";
        buffersArg.units = "buffers";
        buffersArg.type = SoapySDR::ArgInfo::INT;

        streamArgs.push_back(buffersArg);

        SoapySDR::ArgInfo asyncbuffsArg;
        asyncbuffsArg.key = "asyncBuffs";
        asyncbuffsArg.value = "0";
        asyncbuffsArg.name = "Async buffers";
        asyncbuffsArg.description = "Number of async usb buffers (advanced).";
        asyncbuffsArg.units = "buffers";
        asyncbuffsArg.type = SoapySDR::ArgInfo::INT;

        streamArgs.push_back(asyncbuffsArg);

        return streamArgs;
    } else {
         return SoapySDR::Device::getStreamArgsInfo( direction, channel ) ;
    }
}

/*******************************************************************
 * Async thread work
 ******************************************************************/

static void _rx_callback(unsigned char *buf, uint32_t len, void *ctx)
{
    //printf("_rx_callback\n");
    SoapyQS1R *self = (SoapyQS1R *)ctx;
    self->rx_callback(buf, len);
}

void SoapyQS1R::rx_async_operation(void)
{
    //printf("rx_async_operation\n");
    qs1r_read_async(&_rx_callback, _asyncBuffs, bufferLength);
    //printf("rx_async_operation done!\n");
}

void SoapyQS1R::rx_callback(unsigned char *buf, uint32_t len)
{
    //printf("_rx_callback %d _buf_head=%d, numBuffers=%d\n", len, _buf_head, _buf_tail);

    // atomically add len to ticks but return the previous value
    unsigned long long tick = _ticks.fetch_add(len);

    //overflow condition: the caller is not reading fast enough
    if (_buf_count == numBuffers)
    {
        _overflowEvent = true;
        return;
    }

    //copy into the buffer queue
    auto &buff = _buffs[_buf_tail];
    buff.tick = tick;
    buff.data.resize(len);
    std::memcpy(buff.data.data(), buf, len);

    //increment the tail pointer
    _buf_tail = (_buf_tail + 1) % numBuffers;

    //increment buffers available under lock
    //to avoid race in acquireReadBuffer wait
    {
    std::lock_guard<std::mutex> lock(_buf_mutex);
    _buf_count++;

    }

    //notify readStream()
    _buf_cond.notify_one();
}

/*******************************************************************
 * Stream API
 ******************************************************************/

SoapySDR::Stream *SoapyQS1R::setupStream(
        const int direction,
        const std::string &format,
        const std::vector<size_t> &channels,
        const SoapySDR::Kwargs &args)
{
    if (direction == SOAPY_SDR_RX)
    {
        //check the channel configuration
        if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
        {
            throw std::runtime_error("setupStream invalid channel selection");
        }

        //check the format
        if (format == SOAPY_SDR_CF32)
        {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CF32.");
            _rxFormat = RX_FORMAT_FLOAT32;
        }
        else if (format == SOAPY_SDR_CS16)
        {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CS16.");
            _rxFormat = RX_FORMAT_INT16;
        }
        else if (format == SOAPY_SDR_CS8) {
            SoapySDR_log(SOAPY_SDR_INFO, "Using format CS8.");
            _rxFormat = RX_FORMAT_INT8;
        }
        else
        {
            throw std::runtime_error(
                    "setupStream invalid format '" + format
                            + "' -- Only CS8, CS16 and CF32 are supported by SoapyRTLSDR module.");
        }

        if (_rxFormat != RX_FORMAT_INT8 && !_lut_32f.size())
        {
            SoapySDR_logf(SOAPY_SDR_DEBUG, "Generating RTL-SDR lookup tables");
            // create lookup tables
            for (unsigned int i = 0; i <= 0xffff; i++)
            {
    # if (__BYTE_ORDER == __LITTLE_ENDIAN)
                float re = ((i & 0xff) - 127.4f) * (1.0f / 128.0f);
                float im = ((i >> 8) - 127.4f) * (1.0f / 128.0f);
    #else
                float re = ((i >> 8) - 127.4f) * (1.0f / 128.0f);
                float im = ((i & 0xff) - 127.4f) * (1.0f / 128.0f);
    #endif

                std::complex<float> v32f, vs32f;

                v32f.real(re);
                v32f.imag(im);
                _lut_32f.push_back(v32f);

                vs32f.real(v32f.imag());
                vs32f.imag(v32f.real());
                _lut_swap_32f.push_back(vs32f);

                std::complex<int16_t> v16i, vs16i;

                v16i.real(int16_t((float(SHRT_MAX) * re)));
                v16i.imag(int16_t((float(SHRT_MAX) * im)));
                _lut_16i.push_back(v16i);

                vs16i.real(vs16i.imag());
                vs16i.imag(vs16i.real());
                _lut_swap_16i.push_back(vs16i);
            }
        }

        bufferLength = DEFAULT_BUFFER_LENGTH;
        if (args.count("bufflen") != 0)
        {
            try
            {
                int bufferLength_in = std::stoi(args.at("bufflen"));
                if (bufferLength_in > 0)
                {
                    bufferLength = bufferLength_in;
                }
            }
            catch (const std::invalid_argument &){}
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using buffer length %d", bufferLength);

        numBuffers = DEFAULT_NUM_BUFFERS;
        if (args.count("buffers") != 0)
        {
            try
            {
                int numBuffers_in = std::stoi(args.at("buffers"));
                if (numBuffers_in > 0)
                {
                    numBuffers = numBuffers_in;
                }
            }
            catch (const std::invalid_argument &){}
        }
        SoapySDR_logf(SOAPY_SDR_DEBUG, "RTL-SDR Using %d buffers", numBuffers);

        _asyncBuffs = 0;
        if (args.count("asyncBuffs") != 0)
        {
            try
            {
                int asyncBuffs_in = std::stoi(args.at("asyncBuffs"));
                if (asyncBuffs_in > 0)
                {
                    _asyncBuffs = asyncBuffs_in;
                }
            }
            catch (const std::invalid_argument &){}
        }

        //clear async fifo counts
        _buf_tail = 0;
        _buf_count = 0;
        _buf_head = 0;

        //allocate buffers
        _buffs.resize(numBuffers);
        for (auto &buff : _buffs) buff.data.reserve(bufferLength);
        for (auto &buff : _buffs) buff.data.resize(bufferLength);

        return (SoapySDR::Stream *) this;
    } else {
        return SoapySDR::Device::setupStream( direction, format, channels,args );
    }
}

void SoapyQS1R::closeStream(SoapySDR::Stream *stream)
{
    this->deactivateStream(stream, 0, 0);
    _buffs.clear();
}

size_t SoapyQS1R::getStreamMTU(SoapySDR::Stream *stream) const
{
    return bufferLength / BYTES_PER_SAMPLE;
}

int SoapyQS1R::activateStream(
        SoapySDR::Stream *stream,
        const int flags,
        const long long timeNs,
        const size_t numElems)
{
    if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
    _resetBuffer = true;
    _bufferedElems = 0;

    //start the async thread
    if (not _rx_async_thread.joinable())
    {
        //rtlsdr_reset_buffer(dev);
        _rx_async_thread = std::thread(&SoapyQS1R::rx_async_operation, this);
    }

    return 0;
}

int SoapyQS1R::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
    if (flags != 0) {
        return SOAPY_SDR_NOT_SUPPORTED;
    } else if (_rx_async_thread.joinable()) {
        qs1r_cancel_async();
        _rx_async_thread.join();
    }
    return 0;
}

int SoapyQS1R::readStream(
        SoapySDR::Stream *stream,
        void * const *buffs,
        const size_t numElems,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{
    //drop remainder buffer on reset
    if (_resetBuffer and _bufferedElems != 0)
    {
        _bufferedElems = 0;
        this->releaseReadBuffer(stream, _currentHandle);
    }

    //this is the user's buffer for channel 0
    void *buff0 = buffs[0];

    //are elements left in the buffer? if not, do a new read.
    if (_bufferedElems == 0)
    {
        int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
        if (ret < 0) return ret;
        _bufferedElems = ret;
    }

    //otherwise just update return time to the current tick count
    else
    {
        flags |= SOAPY_SDR_HAS_TIME;
        timeNs = SoapySDR::ticksToTimeNs(_bufTicks, _sample_rate);
    }

    size_t returnedElems = std::min(_bufferedElems, numElems);

    //convert into user's buff0
    if (_rxFormat == RX_FORMAT_FLOAT32)
    {
        float *ftarget = (float *) buff0;
        std::complex<float> tmp;
        if (_iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp = _lut_swap_32f[*((uint16_t*) &_currentBuff[2 * i])];
                ftarget[i * 2] = tmp.real();
                ftarget[i * 2 + 1] = tmp.imag();
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp = _lut_32f[*((uint16_t*) &_currentBuff[2 * i])];
                ftarget[i * 2] = tmp.real();
                ftarget[i * 2 + 1] = tmp.imag();
            }
        }
    }
    else if (_rxFormat == RX_FORMAT_INT16)
    {
        int16_t *itarget = (int16_t *) buff0;
        std::complex<int16_t> tmp;
        if (_iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp = _lut_swap_16i[*((uint16_t*) &_currentBuff[2 * i])];
                itarget[i * 2] = tmp.real();
                itarget[i * 2 + 1] = tmp.imag();
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                tmp = _lut_16i[*((uint16_t*) &_currentBuff[2 * i])];
                itarget[i * 2] = tmp.real();
                itarget[i * 2 + 1] = tmp.imag();
            }
        }
    }
    else if (_rxFormat == RX_FORMAT_INT8)
    {
        int8_t *itarget = (int8_t *) buff0;
        if (_iqSwap)
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                itarget[i * 2] = _currentBuff[i * 2 + 1]-128;
                itarget[i * 2 + 1] = _currentBuff[i * 2]-128;
            }
        }
        else
        {
            for (size_t i = 0; i < returnedElems; i++)
            {
                itarget[i * 2] = _currentBuff[i * 2]-128;
                itarget[i * 2 + 1] = _currentBuff[i * 2 + 1]-128;
            }
        }
    }

    //bump variables for next call into readStream
    _bufferedElems -= returnedElems;
    _currentBuff += returnedElems*BYTES_PER_SAMPLE;
    _bufTicks += returnedElems; //for the next call to readStream if there is a remainder

    //return number of elements written to buff0
    if (_bufferedElems != 0) flags |= SOAPY_SDR_MORE_FRAGMENTS;
    else this->releaseReadBuffer(stream, _currentHandle);
    return returnedElems;
}

int SoapyQS1R::writeStream(
        SoapySDR::Stream *stream,
        const void * const *buffs,
        const size_t numElems,
        int &flags,
        const long long timeNs,
        const long timeoutUs) {
    return SoapySDR::Device::writeStream( stream, buffs, numElems, flags, timeNs, timeoutUs ) ;
}

int SoapyQS1R::readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs) {
    return SoapySDR::Device::readStreamStatus( stream, chanMask, flags, timeNs, timeoutUs);
}



/*******************************************************************
 * Direct buffer access API
 ******************************************************************/

size_t SoapyQS1R::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
    return _buffs.size();
}

int SoapyQS1R::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
    buffs[0] = (void *)_buffs[handle].data.data();
    return 0;
}

int SoapyQS1R::acquireReadBuffer(
    SoapySDR::Stream *stream,
    size_t &handle,
    const void **buffs,
    int &flags,
    long long &timeNs,
    const long timeoutUs)
{
    //reset is issued by various settings
    //to drain old data out of the queue
    if (_resetBuffer)
    {
        //drain all buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        _resetBuffer = false;
        _overflowEvent = false;
    }

    //handle overflow from the rx callback thread
    if (_overflowEvent)
    {
        //drain the old buffers from the fifo
        _buf_head = (_buf_head + _buf_count.exchange(0)) % numBuffers;
        _overflowEvent = false;
        SoapySDR::log(SOAPY_SDR_SSI, "O");
        return SOAPY_SDR_OVERFLOW;
    }

    //wait for a buffer to become available
    if (_buf_count == 0)
    {
        std::unique_lock <std::mutex> lock(_buf_mutex);
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs), [this]{return _buf_count != 0;});
        if (_buf_count == 0) return SOAPY_SDR_TIMEOUT;
    }

    //extract handle and buffer
    handle = _buf_head;
    _buf_head = (_buf_head + 1) % numBuffers;
    _bufTicks = _buffs[handle].tick;
    timeNs = SoapySDR::ticksToTimeNs(_buffs[handle].tick, _sample_rate);
    buffs[0] = (void *)_buffs[handle].data.data();
    flags = SOAPY_SDR_HAS_TIME;

    //return number available
    return _buffs[handle].data.size() / BYTES_PER_SAMPLE;
}

void SoapyQS1R::releaseReadBuffer(
    SoapySDR::Stream *stream,
    const size_t handle)
{
    //TODO this wont handle out of order releases
    _buf_count--;
}

int SoapyQS1R::acquireWriteBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        void **buffs,
        const long timeoutUs) {
    return SoapySDR::Device::acquireWriteBuffer(stream,handle,buffs,timeoutUs) ;
}

void SoapyQS1R::releaseWriteBuffer(
        SoapySDR::Stream *stream,
        const size_t handle,
        const size_t numElems,
        int &flags,
        const long long timeNs) {
    SoapySDR::Device::releaseWriteBuffer(stream,handle,numElems,flags,timeNs);
}

