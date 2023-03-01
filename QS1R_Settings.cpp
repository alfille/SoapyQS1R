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

/* Open a SQ1R by matching a string "serial nmumber" obtained in QS1Rfind */
/* Assumes serial is a null terminated string*/
/* NULL on failure */
bool SoapyQS1R::qs1r_by_serial( const char * serial ) {
    if ( strlen( serial ) == 0 ) {
        return false ;
    }

    _dev = NULL ;
    libusb_device ** devlist ;
    ssize_t usb_count = libusb_get_device_list( SoapyQS1RSession::qs1r_context, &devlist ) ;

    printf("Try opening by serial<%s> count=%lu\n",serial,usb_count);
    for ( auto i =0 ; i < usb_count ; ++i ) {
        struct libusb_device_descriptor desc ;
        libusb_get_device_descriptor( devlist[i], &desc ) ; // always succeeds in modern libusb
        if (desc.idVendor == QS1R_VID && desc.idProduct == QS1R_PID) {
            if ( libusb_open( devlist[i], &_dev ) == 0 ) {
                unsigned char aserial[256] ;
                libusb_get_string_descriptor_ascii( _dev, desc.iSerialNumber, aserial, 256 ) ;
                if ( strcmp( (char *) aserial, serial) == 0 ) {
                    break ;
                } else { 
                    libusb_close( _dev ) ;
                    _dev = NULL ;
                } 
            }
        }
    }
    libusb_free_device_list( devlist, 1 ) ;
    printf("%sby serial\n",_dev==NULL?"Not ":"");
    return _dev != NULL ; ;
}

/* Open the "index" number QS1R device */
/* Uses atoi to get the number from the "index" string" */
/* NULL on failure */
bool SoapyQS1R::qs1r_by_index( const char * index ) {
    int target = atoi( index ) ;
    int idx = 0 ;
    libusb_device ** devlist ;
    ssize_t usb_count = libusb_get_device_list( SoapyQS1RSession::qs1r_context, &devlist ) ;

    printf("Try opening by index<%s=%d> count = %lu\n",index,target,usb_count);
    for ( auto i =0 ; i < usb_count ; ++i ) {
        struct libusb_device_descriptor desc ;
        libusb_get_device_descriptor( devlist[i], &desc ) ; // always succeeds in modern libusb
        if (desc.idVendor == QS1R_VID && desc.idProduct == QS1R_PID) {
            if ( idx == target ) {
                int r = libusb_open( devlist[i], &_dev ) ;
                printf("Open code =%d\n",r);
                if (r) printf("libusb open %s\n",libusb_error_name(r) );
                libusb_free_device_list( devlist, 1 ) ;
                return true ;
            }
            ++ idx ;
        }
    }
    libusb_free_device_list( devlist, 1 ) ;
    printf("Not by index\n");
    return false ;
}

bool SoapyQS1R::openDevice( const SoapySDR::Kwargs &args ) 
{

    if (args.count("label") != 0)
        SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

    /* Open the QS1R by matching serial number or index */
    if (args.count("serial") == 0)
        throw std::runtime_error("no QS1r device matches");
    _serial = args.at("serial");

    return qs1r_by_serial(_serial.c_str()) || qs1r_by_index( args.at("index").c_str()) ;
}

SoapyQS1R::SoapyQS1R( const SoapySDR::Kwargs &args ):
    /* Default (startup) state */

    _sample_rate(50e3),
    _bandwidth(40e3),
    _samples_per_read(4096),

    _RX_FREQ(0.0),
    _freq_corr(0.0),
    _antenna("RX BNC LPF"),
    _ticks(0)
{
    if ( !openDevice(args) )
    {
        throw std::runtime_error("no QS1r device matches");
    }

    if ( ! configure_device( ) ) {
            throw std::runtime_error("Cannot set QS1R usb configuration");
    }
    
    /* Check so see if firmware loaded */
    uint32_t sn = 0 ;    
    if ( args.count("force")!=0 
        || firmware_read_sn( &sn ) 
        || sn!=3032011 ) 
    {
        if ( ! firmware_write( "/usr/share/QS1R/firmware/qs1r_firmware_11022011.hex" ) ) {
            throw std::runtime_error("Cannot load QS1R firmware file");
        }
        libusb_close(_dev);
        
        // Need to reopen!!!
        // pause for close to succeed
        struct timespec reset_time = { 4, 0 } ;
        nanosleep( &reset_time, NULL ) ;
        if ( !openDevice(args) || !configure_device() ) {
            throw std::runtime_error("Cannot reopen QS1R after firmware file");
        }
    }
    firmware_read_sn( &sn ) && printf("Firmware %08d\n",sn) ;            

    /* Check so see if FPGA loaded */
    uint32_t fw = 0 ;
    if ( args.count("force")!=0 || FPGA_read_sn( &fw ) || fw!=0x07192011 ) {
        if (!FPGA_write( "/usr/share/QS1R/firmware/QS1R_WINRAD_04112011.rbf")) {
            throw std::runtime_error("Cannot load FPGA firmware file");
        }
    }
    FPGA_read_sn( &fw ) && printf("FPGA %08X\n",fw) ;            
}

SoapyQS1R::~SoapyQS1R( void )
{
        if ( _dev )
        {
                libusb_close( _dev );
        }

        /* cleanup device handles */
}


/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyQS1R::getDriverKey( void ) const
{
        return("QS1R");
}


std::string SoapyQS1R::getHardwareKey( void ) const
{
        return "LTC2208" ;
}


SoapySDR::Kwargs SoapyQS1R::getHardwareInfo( void ) const
{
        SoapySDR::Kwargs info;

        info["origin"] = "https://github.com/alfille/SoapyQS1R" ;
        info["ADC"] = "LTC2208";
        

        return(info);

}


/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyQS1R::getNumChannels( const int dir ) const
{
        return(1);
}


bool SoapyQS1R::getFullDuplex( const int direction, const size_t channel ) const
{
        return(false);
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyQS1R::getSettingInfo(void) const
{
    SoapySDR::ArgInfoList setArgs;

    SoapySDR::ArgInfo ditherArg;
    ditherArg.key = "dithering";
    ditherArg.value = "false";
    ditherArg.name = "ADC Dither";
    ditherArg.description = "ADC dithering enable";
    ditherArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(ditherArg);

    SoapySDR::ArgInfo randomizerArg;
    randomizerArg.key = "randomizing";
    randomizerArg.value = "false";
    randomizerArg.name = "ADC Randomizer";
    randomizerArg.description = "ADC randomizing enable";
    randomizerArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(randomizerArg);

    SoapySDR::ArgInfo gainArg;
    gainArg.key = "gain";
    gainArg.value = "low";
    gainArg.name = "PGA gain";
    gainArg.description = "ADC PGA signal gain";
    gainArg.type = SoapySDR::ArgInfo::STRING;
    gainArg.options.push_back("low");
    gainArg.optionNames.push_back("Low");
    gainArg.options.push_back("high");
    gainArg.optionNames.push_back("High");
    setArgs.push_back(gainArg);

    SoapySDR::ArgInfo bypassArg;
    bypassArg.key = "bypass";
    bypassArg.value = "false";
    bypassArg.name = "DAC Bypass";
    bypassArg.description = "Bypass DAC (audio) output";
    bypassArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(bypassArg);

    SoapySDR::ArgInfo mutingArg;
    mutingArg.key = "muting";
    mutingArg.value = "false";
    mutingArg.name = "External Muting";
    mutingArg.description = "Mute External DAC (audio) input";
    mutingArg.type = SoapySDR::ArgInfo::BOOL;
    setArgs.push_back(mutingArg);

    SoapySDR::ArgInfo clockArg;
    clockArg.key = "clock";
    clockArg.value = "24";
    clockArg.name = "DAC clock";
    clockArg.description = "DAC (audio) clock sampling rate";
    clockArg.type = SoapySDR::ArgInfo::STRING;
    clockArg.options.push_back("24");
    clockArg.optionNames.push_back("24kSPS");
    clockArg.options.push_back("48");
    clockArg.optionNames.push_back("48kSPS");
    setArgs.push_back(clockArg);

    return setArgs;
}

void SoapyQS1R::writeSetting(const std::string &key, const std::string &value)
{
        if(key=="dithering") {
                DDC_putbit( DDC_CONTROL_REG1, ADC_DITHER_ENABLE, value=="true"?1:0 ) ;
        } else if (key=="randomizing") {
                DDC_putbit( DDC_CONTROL_REG1, ADC_RANDOMIZER_ENABLE, value=="true"?1:0 ) ;
        } else if (key=="gain") {
                DDC_putbit( DDC_CONTROL_REG1, ADC_RANDOMIZER_ENABLE, value=="high"?1:0 ) ;
        } else if (key=="bypass") {
                DDC_putbit( DDC_CONTROL_REG0, DAC_BYPASS, value=="true"?1:0 ) ;
        } else if (key=="muting") {
                DDC_putbit( DDC_CONTROL_REG0, DAC_EXT_MUTE_ENABLE, value=="true"?1:0 ) ;
        } else if (key=="clock") {
                DDC_putbit( DDC_CONTROL_REG0, DAC_CLOCK_SELECT, value=="48"?1:0 ) ;
        }

}

std::string SoapyQS1R::readSetting(const std::string &key) const
{
    int value ;
    if ( key=="dithering" && DDC_getbit( DDC_CONTROL_REG1, ADC_DITHER_ENABLE, &value ) ) {
        return ((value==1) ? "true" : "false") ;
    } else if (key=="randomizing" && DDC_getbit( DDC_CONTROL_REG1, ADC_RANDOMIZER_ENABLE, &value ) ) {
        return ((value==1) ? "true" : "false") ;
    } else if (key=="gain" && DDC_getbit( DDC_CONTROL_REG1, ADC_RANDOMIZER_ENABLE, &value ) ) {
        return ((value==1) ? "high" : "low") ;
    } else if (key=="bypass" && DDC_getbit( DDC_CONTROL_REG0, DAC_BYPASS, &value ) ) {
        return ((value==1) ? "true" : "false") ;
    } else if (key=="muting" && DDC_getbit( DDC_CONTROL_REG0, DAC_EXT_MUTE_ENABLE, &value ) ) {
        return ((value==1) ? "true" : "false") ;
    } else if (key=="clock" && DDC_getbit( DDC_CONTROL_REG0, DAC_CLOCK_SELECT, &value ) ) {
        return ((value==1) ? "48" : "24") ;
    }
    return "";
}

/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyQS1R::listAntennas( const int direction, const size_t channel ) const
{
    std::vector<std::string> options;
    options.push_back( "RX BNC LPF" );
    options.push_back( "RX SMA" );
    return(options);
}


void SoapyQS1R::setAntenna( const int direction, const size_t channel, const std::string &name )
{
    if (direction == SOAPY_SDR_RX) {
        _antenna = name ;
    } else {
        throw std::runtime_error("setAntenna failed: QS1R only supports RX");
    }
}


std::string SoapyQS1R::getAntenna( const int direction, const size_t channel ) const
{
    return(_antenna);
}


/*******************************************************************
 * Frontend corrections API
 ******************************************************************/

/*******************************************************************
 * Gain API
 ******************************************************************/

// only "gain" is the unquantified high and low in general settings.

/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyQS1R::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
    if ( name == "RF" ) {
    uint32_t f = DDC_FREQ( frequency ) ;
        if ( write_multibus( DDC_FREQ_REG, f ) ) {
            _RX_FREQ = frequency ;
        }
    } else if ( name == "CORR" ) {
        _freq_corr = frequency ;
        setFrequency( direction, channel, "RF", _RX_FREQ, args ) ;
    } else {
        throw std::runtime_error( "setFrequency(" + name + ") unknown name" );
    }
}


double SoapyQS1R::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
    if ( name == "RF" ) {
        return _RX_FREQ ;
    } else if ( name == "CORR" ) {
        return _freq_corr ;
    } else {
        return 0.0 ;
    }
}

SoapySDR::ArgInfoList SoapyQS1R::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
    SoapySDR::ArgInfoList freqArgs;
    // TODO: frequency arguments
    return freqArgs;
}

std::vector<std::string> SoapyQS1R::listFrequencies( const int direction, const size_t channel ) const
{
    std::vector<std::string> names;
    names.push_back("RF");
    names.push_back("CORR");
    return names;
}


SoapySDR::RangeList SoapyQS1R::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
    SoapySDR::RangeList results;
    if ( name == "RF" ) {
        if (_antenna == "RX BNC LPF") {
            results.push_back(SoapySDR::Range(15000, 55000000));
        } else if (_antenna == "RX SMA") {
            results.push_back(SoapySDR::Range(15000, 300000000));
        }
    } else if (name == "CORR" ) {
        results.push_back(SoapySDR::Range(-50000, 50000));
    }
    return results;
}


/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyQS1R::setSampleRate( const int direction, const size_t channel, const double rate )
{
    std::lock_guard<std::mutex> lock(_device_mutex);

    if(direction==SOAPY_SDR_RX){
        if ( rate <= 25e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG,   25000 ) ) {
                _sample_rate = 25e3;
                _bandwidth = 40e3 ;
                _samples_per_read = 4096 ;
            }
        } else if ( rate <= 50e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG,   50000 ) ) {
                _sample_rate = 50e3 ;
                _bandwidth = 40e3 ;
                _samples_per_read = 4096 ;
            }
        } else if ( rate <= 125e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG,  125000 ) ) {
                _sample_rate = 125e3 ;
                _bandwidth = 100e3 ;
                _samples_per_read = 8192 ;
            }
        } else if ( rate <= 250e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG,  250000 ) ) {
                _sample_rate = 250e3 ;
                _bandwidth = 200e3 ;
                _samples_per_read = 8192 ;
            }
        } else if ( rate <= 625e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG,  625000 ) ) {
                _sample_rate = 625e3 ;
                _bandwidth = 500e3 ;
                _samples_per_read = 16384 ;
            }
        } else if ( rate <= 1250e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG, 1250000 ) ) {
                _sample_rate = 1250e3 ;
                _bandwidth = 1000e3 ;
                _samples_per_read = 16384 ;
            }
        } else if ( rate <= 1562.5e3 ) {
            if ( write_multibus( DDC_SAMPLE_RATE_REG, 1562500 ) ) {
                _sample_rate = 1562.5e3 ;
                _bandwidth = 1200e3 ;
                _samples_per_read = 16384 ;
            }
        } else {
            if ( write_multibus( DDC_SAMPLE_RATE_REG, 2500000 ) ) {
                _sample_rate = 2500e3 ;
                _bandwidth = 2000e3 ;
                _samples_per_read = 16384 ;
            }
        }
    }
}


double SoapyQS1R::getSampleRate( const int direction, const size_t channel ) const
{
    std::lock_guard<std::mutex> lock(_device_mutex);
    return(_sample_rate);
}


std::vector<double> SoapyQS1R::listSampleRates( const int direction, const size_t channel ) const
{
    std::vector<double> options;
    options.push_back(   25e3 );
    options.push_back(   50e3 );
    options.push_back(  125e3 );
    options.push_back(  250e3 );
    options.push_back(  625e3 );
    options.push_back( 1250e3 );
    options.push_back( 1562.5e3 );
    options.push_back( 2500e3 );
    return(options);
}


void SoapyQS1R::setBandwidth( const int direction, const size_t channel, const double bw )
{
    double sr ;
    if ( bw <= 20e3 ) {
        sr = 25e3 ;
    } else if ( bw <= 40e3 ) {
        sr = 50e3 ;
    } else if ( bw <= 100e3 ) {
        sr = 125e3 ;
    } else if ( bw <= 200e3 ) {
        sr = 250e3 ;
    } else if ( bw <= 500e3 ) {
        sr = 625e3 ;
    } else if ( bw <= 1000e3 ) {
        sr = 1250e3 ;
    } else if ( bw <= 1200e3 ) {
        sr = 1562.5e3 ;
    } else {
        sr = 2000e3 ;
    }
    setSampleRate( direction, channel, sr ) ;
}


double SoapyQS1R::getBandwidth( const int direction, const size_t channel ) const
{
    return (_bandwidth);
}


std::vector<double> SoapyQS1R::listBandwidths( const int direction, const size_t channel ) const
{
    std::vector<double> options;
    options.push_back( 20e3 );
    options.push_back( 40e3 );
    options.push_back( 100e3 );
    options.push_back( 200e3 );
    options.push_back( 500e3 );
    options.push_back( 1000e3 );
    options.push_back( 1200e3 );
    options.push_back( 2000e3 );
    return(options);
}
