# SoapyQS1R
SoapySDR driver for the QS1R radio -- a software defined radio in the HF frequency range.

# Background
* The QS1R was a very elegant SDR radio available ~ 2010. It has been out of production for a while and all support has vanished. The creator has moved to other projects. Originally an open-source design and code, the sophisticated software (SDRMAX V) was available only in binary or under NDA.
* Current uses of the QS1R are as a 7-channel CW scanner (skimmer)
* SDRMAX V works with difficulty as platforms have progressed. It uses an older version of QT for example. Updating that software would not be redistributable.
* Creating a modern driver (SoapySDR) opens up the full ecosystem of SDR software. And it is open source.

# Design
## Source material
* SoapyHackRF code. A very similar interface. HackRF is 8-bit vs 16-bit and has transmit, which is optional on the QS1R.
* qs_io.py: A simple example code openly provided by the QS1R designer, Phil Covington.
* QS1R API documentation describing the USB interface and the device configuration fields.

## C++ cmake
* SoapySDR and it's drivers are C++ code.
* Soapy drivers require SoapySDR (dev version) 
* USB requires libusb-1.0 dev version
