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
#if 0
#include "SoapyQS1R.hpp"
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Formats.hpp>
#include <chrono>
#include <thread>
#include <algorithm> //min

int _hackrf_rx_callback( hackrf_transfer *transfer )
{
    SoapyQS1R* obj = (SoapyQS1R *) transfer->rx_ctx;
    return(obj->hackrf_rx_callback( (int8_t *) transfer->buffer, transfer->valid_length ) );
}


int SoapyQS1R::hackrf_rx_callback( int8_t *buffer, int32_t length )
{
    std::unique_lock<std::mutex> lock(_buf_mutex);
    _rx_stream.buf_tail = (_rx_stream.buf_head + _rx_stream.buf_count) % _rx_stream.buf_num;
    memcpy(_rx_stream.buf[_rx_stream.buf_tail], buffer, length );

    if ( _rx_stream.buf_count == _rx_stream.buf_num )
    {
        _rx_stream.overflow=true;
        _rx_stream.buf_head = (_rx_stream.buf_head + 1) % _rx_stream.buf_num;
    }else  {
        _rx_stream.buf_count++;
    }
    _buf_cond.notify_one();

    return(0);
}

std::vector<std::string> SoapyQS1R::getStreamFormats(const int direction, const size_t channel) const
{
    std::vector<std::string> formats;

    formats.push_back(SOAPY_SDR_CS8);
    formats.push_back(SOAPY_SDR_CS16);
    formats.push_back(SOAPY_SDR_CF32);
    formats.push_back(SOAPY_SDR_CF64);

    return formats;
}

std::string SoapyQS1R::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
    fullScale = 128;
    return SOAPY_SDR_CS16;
}

SoapySDR::ArgInfoList SoapyQS1R::getStreamArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList streamArgs;

    SoapySDR::ArgInfo buffersArg;
    buffersArg.key="buffers";
    buffersArg.value = std::to_string(BUF_NUM);
    buffersArg.name = "Buffer Count";
    buffersArg.description = "Number of buffers per read.";
    buffersArg.units = "buffers";
    buffersArg.type = SoapySDR::ArgInfo::INT;
    streamArgs.push_back(buffersArg);

    return streamArgs;
}

void SoapyQS1R::Stream::allocate_buffers() {
    buf = (int8_t * *) malloc( buf_num * sizeof(int8_t *) );
    if ( buf ) {
        for ( unsigned int i = 0; i < buf_num; ++i ) {
            buf[i] = (int8_t *) malloc( buf_len );
        }
    }
}

void SoapyQS1R::Stream::clear_buffers() {
    if ( buf ) {
        for ( unsigned int i = 0; i < buf_num; ++i ) {
            if ( buf[i] ) {
                free( buf[i] );
            }
        }
        free( buf );
        buf = NULL;
    }

    buf_count = 0;
    buf_tail = 0;
    buf_head = 0;
    remainderSamps = 0;
    remainderOffset = 0;
    remainderBuff = nullptr;
    remainderHandle = -1;
}

SoapySDR::Stream *SoapyQS1R::setupStream(
    const int direction,
    const std::string &format,
    const std::vector<size_t> &channels,
    const SoapySDR::Kwargs &args )
{
    std::lock_guard<std::mutex> lock(_device_mutex);

    _transfer = libusb_alloc_transfer(0);

    if ( channels.size() > 1 or( channels.size() > 0 and channels.at( 0 ) != 0 ) )
    {
        throw std::runtime_error( "setupStream invalid channel selection" );
    }

    if(direction==SOAPY_SDR_RX){
        if (_rx_stream.opened) {
            throw std::runtime_error("RX stream already opened");
        }

        if ( format == SOAPY_SDR_CS8 )
        {
            SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS8." );
            _rx_stream.format = HACKRF_FORMAT_INT8;
        }else if ( format == SOAPY_SDR_CS16 )
        {
            SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CS16." );
            _rx_stream.format = HACKRF_FORMAT_INT16;
        }else if ( format == SOAPY_SDR_CF32 )
        {
            SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF32." );
            _rx_stream.format= HACKRF_FORMAT_FLOAT32;
        }else if(format==SOAPY_SDR_CF64){
            SoapySDR_log( SOAPY_SDR_DEBUG, "Using format CF64." );
            _rx_stream.format= HACKRF_FORMAT_FLOAT64;
        }else throw std::runtime_error( "setupStream invalid format " + format );

        _rx_stream.buf_num = BUF_NUM;

        if ( args.count( "buffers" ) != 0 )
        {
            try
            {
                int numBuffers_in = std::stoi(args.at("buffers"));
                if (numBuffers_in > 0) {
                    _rx_stream.buf_num = numBuffers_in;
                }
            }
            catch (const std::invalid_argument &){}

        }
        _rx_stream.allocate_buffers();

        _rx_stream.opened = true;

        return RX_STREAM;
    } else {
        throw std::runtime_error("Invalid direction");
    }
}

void SoapyQS1R::closeStream( SoapySDR::Stream *stream )
{
    this->deactivateStream(stream, 0, 0);
    std::lock_guard<std::mutex> lock(_device_mutex);
    if (stream == RX_STREAM) {
        _rx_stream.clear_buffers();
        _rx_stream.opened = false;
    } else if (stream == TX_STREAM) {
        _tx_stream.clear_buffers();
        _tx_stream.opened = false;
    }
    libusb_free_transfer(_transfer) ;
    _transfer = NULL ;
}


size_t SoapyQS1R::getStreamMTU( SoapySDR::Stream *stream ) const
{
    if(stream == RX_STREAM){
        return _rx_stream.buf_len/BYTES_PER_SAMPLE;
    } else if(stream == TX_STREAM){
        return _tx_stream.buf_len/BYTES_PER_SAMPLE;
    } else {
        throw std::runtime_error("Invalid stream");
    }
}

int SoapyQS1R::activateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs,
    const size_t numElems )
{

    if(stream == RX_STREAM){

        std::lock_guard<std::mutex> lock(_device_mutex);


        if(_current_mode==HACKRF_TRANSCEIVER_MODE_RX)
            return 0;

        if(_current_mode==HACKRF_TRANSCEIVER_MODE_TX){

            if(_tx_stream.burst_end){

                while(hackrf_is_streaming(_dev)==HACKRF_TRUE)
                    std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }

            hackrf_stop_tx(_dev);
            
            // determine what (if any) settings  need to be changed for RX; only applicable if there is both a source and sink block
            // sample_rate
            if(_current_samplerate != _rx_stream.samplerate) {
                _current_samplerate = _rx_stream.samplerate;
                SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream - Set RX samplerate to %f", _current_samplerate);
                hackrf_set_sample_rate(_dev,_current_samplerate);
            }
            
            // frequency
            if(_current_frequency != _rx_stream.frequency) {
                _current_frequency = _rx_stream.frequency;
                SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream - Set RX frequency to %lu", _current_frequency);
                hackrf_set_freq(_dev,_current_frequency);
            }
            
            // frequency_correction; assume RX and TX use the same correction
            // This will be the setting of whichever block was last added to the flow graph
            
            // RF Gain (RF Amp for TX & RX)
            if(_current_amp != _rx_stream.amp_gain) {
                _current_amp = _rx_stream.amp_gain;
                SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream - Set RX amp gain to %d", _current_amp);
                hackrf_set_amp_enable(_dev,(_current_amp > 0)?1 : 0 );
            }
            
            // IF Gain (LNA for RX; VGA_TX for TX)
            // BB Gain (VGA for RX; n/a for TX)
            // These are independant values in the hackrf, so no need to change

            // Bandwidth
            if(_current_bandwidth !=_rx_stream.bandwidth) {
                _current_bandwidth =_rx_stream.bandwidth;
                SoapySDR_logf(SOAPY_SDR_DEBUG, "activateStream - Set RX bandwidth to %d", _current_bandwidth);
                hackrf_set_baseband_filter_bandwidth(_dev,_current_bandwidth);
            }
        }

        SoapySDR_logf(SOAPY_SDR_DEBUG, "Start RX");

        //reset buffer tracking before streaming
        {
            _rx_stream.buf_count = 0;
            _rx_stream.buf_head = 0;
            _rx_stream.buf_tail = 0;
        }

        int ret = hackrf_start_rx(_dev, _hackrf_rx_callback, (void *) this);
        if (ret != HACKRF_SUCCESS) {
            SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_start_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
        }

        ret=hackrf_is_streaming(_dev);

        if (ret==HACKRF_ERROR_STREAMING_EXIT_CALLED){

            hackrf_close(_dev);
            hackrf_open_by_serial(_serial.c_str(), &_dev);
            _current_frequency=_rx_stream.frequency;
            hackrf_set_freq(_dev,_current_frequency);
            _current_samplerate=_rx_stream.samplerate;
            hackrf_set_sample_rate(_dev,_current_samplerate);
            _current_bandwidth=_rx_stream.bandwidth;
            hackrf_set_baseband_filter_bandwidth(_dev,_current_bandwidth);
            _current_amp=_rx_stream.amp_gain;
            hackrf_set_amp_enable(_dev,(_current_amp > 0)?1 : 0 );
            hackrf_set_lna_gain(_dev,_rx_stream.lna_gain);
            hackrf_set_vga_gain(_dev,_rx_stream.vga_gain);
            hackrf_start_rx(_dev,_hackrf_rx_callback,(void *) this);
            ret=hackrf_is_streaming(_dev);
        }
        if(ret!=HACKRF_TRUE){
            SoapySDR_logf(SOAPY_SDR_ERROR,"Activate RX Stream Failed.");
            return SOAPY_SDR_STREAM_ERROR;

        }
            _current_mode = HACKRF_TRANSCEIVER_MODE_RX;

    }
    return(0);
}


int SoapyQS1R::deactivateStream(
    SoapySDR::Stream *stream,
    const int flags,
    const long long timeNs )
{

    if(stream == RX_STREAM){

        std::lock_guard<std::mutex> lock(_device_mutex);

        if(_current_mode==HACKRF_TRANSCEIVER_MODE_RX) {

            int ret = hackrf_stop_rx(_dev);
            if (ret != HACKRF_SUCCESS) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_rx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
            }
            _current_mode = HACKRF_TRANSCEIVER_MODE_OFF;
        }
    } else if(stream == TX_STREAM) {

        std::lock_guard<std::mutex> lock(_device_mutex);

        if(_current_mode==HACKRF_TRANSCEIVER_MODE_TX) {
            int ret = hackrf_stop_tx(_dev);
            if (ret != HACKRF_SUCCESS) {
                SoapySDR::logf(SOAPY_SDR_ERROR, "hackrf_stop_tx() failed -- %s", hackrf_error_name(hackrf_error(ret)));
            }
            _current_mode = HACKRF_TRANSCEIVER_MODE_OFF;
        }

    }
    return(0);
}

void readbuf(int8_t * src, void * dst, uint32_t len,uint32_t format,size_t offset){

    if(format==HACKRF_FORMAT_INT8){
        int8_t *samples_cs8=(int8_t *) dst+offset*BYTES_PER_SAMPLE;
        for (uint32_t i=0;i<len;++i){
            samples_cs8[i*BYTES_PER_SAMPLE] = src[i*BYTES_PER_SAMPLE];
            samples_cs8[i*BYTES_PER_SAMPLE+1] = src[i*BYTES_PER_SAMPLE+1];
        }

    }else if(format==HACKRF_FORMAT_INT16){

        int16_t *samples_cs16=(int16_t *) dst+offset*BYTES_PER_SAMPLE;
        for (uint32_t i=0;i<len;++i){
            samples_cs16[i*BYTES_PER_SAMPLE] = (int16_t)(src[i*BYTES_PER_SAMPLE]<<8);
            samples_cs16[i*BYTES_PER_SAMPLE+1] = (int16_t)(src[i*BYTES_PER_SAMPLE+1]<<8);
        }
    }else if(format==HACKRF_FORMAT_FLOAT32){
        float *samples_cf32=(float *) dst+offset*BYTES_PER_SAMPLE;
        for (uint32_t i=0;i<len;++i){
            samples_cf32[i*BYTES_PER_SAMPLE] = (float)(src[i*BYTES_PER_SAMPLE]/127.0);
            samples_cf32[i*BYTES_PER_SAMPLE+1] = (float)(src[i*BYTES_PER_SAMPLE+1]/127.0);
        }
    }else if(format==HACKRF_FORMAT_FLOAT64){
        double *samples_cf64=(double *) dst+offset*BYTES_PER_SAMPLE;
        for (uint32_t i=0;i<len;++i){
            samples_cf64[i*BYTES_PER_SAMPLE] = (double)(src[i*BYTES_PER_SAMPLE]/127.0);
            samples_cf64[i*BYTES_PER_SAMPLE+1] = (double)(src[i*BYTES_PER_SAMPLE+1]/127.0);
        }
    } else {
        SoapySDR_log( SOAPY_SDR_ERROR, "read format not support" );
    }
}

int SoapyQS1R::readStream(
    SoapySDR::Stream *stream,
    void * const *buffs,
    const size_t numElems,
    int &flags,
    long long &timeNs,
    const long timeoutUs )
{
    if(stream != RX_STREAM){
        return SOAPY_SDR_NOT_SUPPORTED;
    }
    /* this is the user's buffer for channel 0 */
    size_t returnedElems = std::min(numElems,this->getStreamMTU(stream));

    size_t samp_avail=0;

    if(_rx_stream.remainderHandle >= 0){

        const size_t n =std::min(_rx_stream.remainderSamps,returnedElems);

        if(n<returnedElems){
            samp_avail=n;
        }

        readbuf(_rx_stream.remainderBuff+_rx_stream.remainderOffset*BYTES_PER_SAMPLE,buffs[0],n,_rx_stream.format,0);

        _rx_stream.remainderOffset+=n;
        _rx_stream.remainderSamps -=n;

        if(_rx_stream.remainderSamps==0){

            this->releaseReadBuffer(stream,_rx_stream.remainderHandle);
            _rx_stream.remainderHandle=-1;
            _rx_stream.remainderOffset=0;
        }

        if(n==returnedElems)
            return returnedElems;
    }

    size_t handle;
    int ret = this->acquireReadBuffer(stream, handle, (const void **)&_rx_stream.remainderBuff, flags, timeNs, timeoutUs);

    if (ret < 0){
        if((ret == SOAPY_SDR_TIMEOUT) && (samp_avail > 0)){
            return samp_avail;
        }
        return ret;
    }

    _rx_stream.remainderHandle=handle;
    _rx_stream.remainderSamps=ret;


    const size_t n =std::min((returnedElems-samp_avail),_rx_stream.remainderSamps);

    readbuf(_rx_stream.remainderBuff,buffs[0],n,_rx_stream.format,samp_avail);
    _rx_stream.remainderSamps -=n;
    _rx_stream.remainderOffset +=n;

    if(_rx_stream.remainderSamps==0){
        this->releaseReadBuffer(stream,_rx_stream.remainderHandle);
        _rx_stream.remainderHandle=-1;
        _rx_stream.remainderOffset=0;
    }

    return(returnedElems);
}


int SoapyQS1R::readStreamStatus(
        SoapySDR::Stream *stream,
        size_t &chanMask,
        int &flags,
        long long &timeNs,
        const long timeoutUs
){

    if(stream != TX_STREAM){
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    //calculate when the loop should exit
    const auto timeout = std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(std::chrono::microseconds(timeoutUs));
    const auto exitTime = std::chrono::high_resolution_clock::now() + timeout;

    //poll for status events until the timeout expires
    while (true)
    {
        if(_tx_stream.underflow){
            _tx_stream.underflow=false;
            SoapySDR::log(SOAPY_SDR_SSI, "U");
            return SOAPY_SDR_UNDERFLOW;
        }

        //sleep for a fraction of the total timeout
        const auto sleepTimeUs = std::min<long>(1000, timeoutUs/10);
        std::this_thread::sleep_for(std::chrono::microseconds(sleepTimeUs));

        //check for timeout expired
        const auto timeNow = std::chrono::high_resolution_clock::now();
        if (exitTime < timeNow) return SOAPY_SDR_TIMEOUT;
    }
}

int SoapyQS1R::acquireReadBuffer(
        SoapySDR::Stream *stream,
        size_t &handle,
        const void **buffs,
        int &flags,
        long long &timeNs,
        const long timeoutUs)
{


    if(stream != RX_STREAM){
        return SOAPY_SDR_NOT_SUPPORTED;
    }

    if ( _current_mode!=HACKRF_TRANSCEIVER_MODE_RX ) {

        //wait for tx to be consumed before switching
        {
            std::unique_lock <std::mutex> lock( _buf_mutex );
            if (not _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs),
                [this]{return this->_tx_stream.buf_count == 0;})) return SOAPY_SDR_TIMEOUT;
        }

        int ret=this->activateStream(stream);
        if(ret<0) return ret;
    }

    std::unique_lock <std::mutex> lock( _buf_mutex );

    while (_rx_stream.buf_count == 0)
    {
        _buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
        if (_rx_stream.buf_count == 0) return SOAPY_SDR_TIMEOUT;
    }

    if(_rx_stream.overflow) {
        flags|=SOAPY_SDR_END_ABRUPT;
        _rx_stream.overflow=false;
        SoapySDR::log(SOAPY_SDR_SSI,"O");
        return  SOAPY_SDR_OVERFLOW;
    }

    handle=_rx_stream.buf_head;
    _rx_stream.buf_head = (_rx_stream.buf_head + 1) % _rx_stream.buf_num;
    this->getDirectAccessBufferAddrs(stream,handle,(void **)buffs);

    return this->getStreamMTU(stream);
}

void SoapyQS1R::releaseReadBuffer(
        SoapySDR::Stream *stream,
        const size_t handle)
{
    if(stream != RX_STREAM){
        throw std::runtime_error("Invalid stream");
    }

    std::unique_lock <std::mutex> lock( _buf_mutex );
    _rx_stream.buf_count--;
}


size_t SoapyQS1R::getNumDirectAccessBuffers(
        SoapySDR::Stream *stream)
{
    if (stream == RX_STREAM) {
        return _rx_stream.buf_num;
    } else if(stream == TX_STREAM){
        return _tx_stream.buf_num;
    } else {
        throw std::runtime_error("Invalid stream");
    }
}

int SoapyQS1R::getDirectAccessBufferAddrs(
        SoapySDR::Stream *stream,
        const size_t handle,
        void **buffs)
{

    if (stream == RX_STREAM) {
        buffs[0]=(void *)_rx_stream.buf[handle];
    } else if (stream == TX_STREAM) {
        buffs[0]=(void *)_tx_stream.buf[handle];
    } else {
        throw std::runtime_error("Invalid stream");
    }

    return 0;
}
#endif
