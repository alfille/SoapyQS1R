#-------------------------------------------------------------------------------
# Name:        qs_io.py
# Purpose:
#
# Author:      phil n8vb
#
# Created:     08/07/2011
# Copyright:   (c) Philip Covington, N8VB 2011
# Licence:     <your licence>
#-------------------------------------------------------------------------------
#!/usr/bin/env python

"""
"""

import usb.core as usbc
import array
import struct
import time

class RequestType:
    """
    USB Request Type Enumeration.
    """
    VRT_VENDOR_IN = 0xc0
    VRT_VENDOR_OUT = 0x40

class Request:
    """
    Vendor Request Enumeration.
    """
    VRQ_FPGA_LOAD = 0x02
    VRQ_FPGA_SET_RESET = 0x04
    VRQ_MULTI_WRITE = 0x05
    VRQ_I2C_WRITE = 0x08
    VRQ_I2C_SPEED_SET = 0x0b
    VRQ_CPU_SPEED_SET = 0x0c
    VRQ_EP_RESET = 0x0d

    VRQ_I2C_READ = 0x81
    VRQ_SN_READ = 0x83
    VRQ_EEPROM_TYPE_READ = 0x84
    VRQ_I2C_SPEED_READ = 0x85
    VRQ_MULTI_READ = 0x86
    VRQ_DEBUG_READ = 0x87

class FX2RAM:
    """
    FX2 RAM Enumeration.
    """
    FX2_RAM_RESET = 0xe600
    FX2_WRITE_RAM_REQ = 0xA0

class FPGALoad:
    """
    FPGA Load stages.
    """
    FL_BEGIN = 0x0
    FL_END = 0x2

class FPGARegisters:
    """
    FPGA Multibus Registers.
    """
    MB_DDC_VERSION = 0x0
    MB_CONTROL0 = 0x1
    MB_CONTROL1 = 0x2
    MB_SAMPLERATE = 0x3
    MB_GPIO0_CNTRL = 0x4
    MB_GPIO0_IO = 0x5
    MB_RFBIO_CTRL = 0x6
    MB_RFBIO_IO = 0x7
    MB_EXT_CTRL = 0x8
    MB_EXT_IO = 0x9
    MB_FREQRX0 = 0x0a
    MB_FREQRX1 = 0x0b
    MB_FREQRX2 = 0x0c
    MB_FREQRX3 = 0x0d

class MBControl0:
    """
    FPGA Register Control 0 Bit Definitions.
    """
    DAC_BYPASS = 0x1
    DAC_EXT_MUTE_EN = 0x2
    WB_BYPASS = 0x4
    DAC_CLK_SEL = 0x8
    MASTER_RESET = 0x10

class MBControl1:
    """
    FPGA Register Control 1 Bit Definitions.
    """
    PGA = 0x1
    RAND = 0x2
    DITH = 0x4

class VersionNumbers:
    """
    Version Numbers for firmware and FPGA.
    """
    VN_FX2_FW = '03032011'
    VN_DDC_VER = 0x07192011

class I2CAddresses:
    """
    I2C Address Enumeration.
    """
    FX2_EEPROM = 0x51

class QsIO:
    """
    The QsIO Communications Class.
    """
    def __init__(self):
        self.vendorId= 0xfffe
        self.productId= 0x8
        self.found = False
        self.timeout = 1000
        self.devs = []
        self.dev = None

    def find_devices(self):
        """
        Finds all QS1R devices on USB.
        """
        self.devs = usbc.find(find_all=True,
                            idVendor=self.vendorId,
                            idProduct=self.productId)
        return self.devs

    def use_device(self, index=0):
        """
        Use a particular QS1R device.
        """
        self.dev = self.devs[index]
        if not self.dev is None:
            self.found = True
            try:
                self.dev.set_configuration(1)
                self.dev.set_interface_altsetting(0)
            except usbc.USBError as (strerror):
                print "qs_io error: {0}".format(strerror)
            except:
                print 'Unknown error'
        else:
            raise ValueError('Device Not Found')

    def read_fw_sn(self):
        """
        Reads the firmware serial number from the QS1R.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            val = self.dev.ctrl_transfer(bmRequestType=RequestType.VRT_VENDOR_IN,
                                         bRequest=Request.VRQ_SN_READ,
                                         wValue=0,
                                         wIndex=0,
                                         data_or_wLength=4,
                                         timeout=self.timeout)
            if not val is None:
                s = struct.unpack('<I', val)[0]
                return "{0:08d}".format(s)
            else:
                return None

    def load_firmware(self, fileName):
        """
        Loads QS1R firmware file.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return False
        else:
            aval = array.array('B', [1,])
            if self._write_cpu_ram(FX2RAM.FX2_RAM_RESET, aval) == None:
                raise ValueError('Could not put CPU into reset')
            else:
                file = None
                success = True
                sarr = array.array('B')
                file = open(fileName, 'r')
                if not file is None:
                    while(True):
                        line = file.readline()
                        if len(line) == 0:
                            break
                        if line[0] != ':':
                            raise ValueError('Firmware file appears to be corrupt')
                            success = False
                            break
                        else:
                            flen = int(line[1:3], 16)
                            faddr = int(line[3:7], 16)
                            type = int(line[7:9], 16)
                            if type == 0:
                                sbuf = line[9:(flen * 2) + 9]
                                li = []
                                for i in range(0, flen*2,2):
                                    li.append(int(sbuf[i:i+2], 16))
                                ar = array.array('B', li)
                                self._write_cpu_ram(faddr, ar)
                            elif type == 1:
                                break
                            else:
                                raise ValueError('Firmware extended address not supported')
                    file.close()
                aval = array.array('B', [0,])
                if self._write_cpu_ram(FX2RAM.FX2_RAM_RESET, aval) == None:
                    raise ValueError('Could not take CPU out of reset')
                    return False
                else:
                    file.close()
                    return success

    def load_fpga(self, fileName):
        """
        Loads QS1R FPGA file.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            val = self.dev.ctrl_transfer( bmRequestType=RequestType.VRT_VENDOR_OUT,
                                            bRequest=Request.VRQ_FPGA_LOAD,
                                            wValue=0,
                                            wIndex=FPGALoad.FL_BEGIN,
                                            data_or_wLength=[],
                                            timeout=self.timeout)
            if (val != 0):
                return None
            else:
                file = None
                file = open(fileName, 'rb')
                if not file is None:
                    file.seek(0)
                    while(True):
                        f = file.read(1024)
                        if (len(f) == 0):
                            break
                        else:
                            a = array.array('B')
                            a.fromstring(f)
                            flen = self.write_ep4(a)
                            if (flen != len(a)):
                                return None
                else:
                    print 'bad filename'
                    return None

                val = self.dev.ctrl_transfer( bmRequestType=RequestType.VRT_VENDOR_OUT,
                                            bRequest=Request.VRQ_FPGA_LOAD,
                                            wValue=0,
                                            wIndex=FPGALoad.FL_END,
                                            data_or_wLength=[],
                                            timeout=self.timeout)
                if (val != 0):
                    return None
                else:
                    return True

    def _write_cpu_ram(self, startAddress, wrbuffer):
        if (len(wrbuffer) > 64):
            raise ValueError('Max length is 64 bytes!')
        else:
            val = self.dev.ctrl_transfer(bmRequestType=RequestType.VRT_VENDOR_OUT,
                                         bRequest=FX2RAM.FX2_WRITE_RAM_REQ,
                                         wValue=startAddress,
                                         wIndex=0,
                                         data_or_wLength=wrbuffer,
                                         timeout=self.timeout)
            if (val == len(wrbuffer)):
                return val
            else:
                return None

    def write_ep2(self, wrbuffer):
        """
        Write to endpoint 2.  Ep2 writes to the DAC.
        """
        return self.dev.write(0x2, wrbuffer, interface=0, timeout=self.timeout)

    def write_ep4(self, wrbuffer):
        """
        Write to endpoint 4.  Ep4 is used for FPGA configuration.
        """
        #if not type(wrbuffer) is array.array:
        #    raise ValueError('Buffer must be of type array.array')
        return self.dev.write(0x4, wrbuffer, interface=0, timeout=self.timeout)

    def read_ep6(self, length):
        """
        Read from endpoint 6.  Ep6 is the DDC I/Q data input.
        """
        return self.dev.read(0x86, length, interface=0, timeout=self.timeout)

    def read_ep8(self, length):
        """
        Read from endpoint 8. Ep8 is the Wideband Spectrum data input.
        """
        return self.dev.read(0x88, length, interface=0, timeout=self.timeout)

    def read_eeprom(self, address, offset, length):
        """
        Read from an EEPROM connected to the I2C bus.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            command = array.array('B')
            command.append((0xff00 & offset) >> 8)
            command.append(0xff & offset)
            val = self.write_i2c(address, command)
            if not val:
                raise ValueError('Error reading from EEPROM')
            else:
                return self.read_i2c(address, length)

    def write_eeprom(self, address, offset, values=[]):
        """
        Write to an EEPROM connected to the I2C bus.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        elif (len(values) == 0):
            return None
        else:
            for value in values:
                command = array.array('B')
                command.append((0xff00 & offset) >> 8)
                command.append(0xff & offset)
                command.append(value)
                val = self.write_i2c(address, command)
                if not val:
                    raise ValueError('Error writing EEPROM')
                else:
                    offset = offset + 1
                    time.sleep(0.010)
            return True

    def read_i2c(self, address, length):
        """
        Read fron the QS1R I2C bus.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            val = self.dev.ctrl_transfer( bmRequestType=RequestType.VRT_VENDOR_IN,
                                            bRequest=Request.VRQ_I2C_READ,
                                            wValue=address,
                                            wIndex=0,
                                            data_or_wLength=length,
                                            timeout=self.timeout)

            if (length != len(val)):
                raise ValueError('Error reading I2C Bus')
            else:
                return val

    def write_i2c(self, address, values=[]):
        """
        Write to the QS1R I2C bus.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        elif (len(values) == 0):
            return None
        else:
            val = self.dev.ctrl_transfer( bmRequestType=RequestType.VRT_VENDOR_OUT,
                                            bRequest=Request.VRQ_I2C_WRITE,
                                            wValue=address,
                                            wIndex=0,
                                            data_or_wLength=values,
                                            timeout=self.timeout)

            if (len(values) != val):
                raise ValueError('Error writing I2C Bus')
            else:
                return True

    def write_multibus_int(self, register_number, value):
        """
        Write to the QS1R multibus registers.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            data = struct.pack('<I', value)
            val = self.dev.ctrl_transfer(bmRequestType=RequestType.VRT_VENDOR_OUT,
                                         bRequest=Request.VRQ_MULTI_WRITE,
                                         wValue=register_number,
                                         wIndex=0,
                                         data_or_wLength=data,
                                         timeout=self.timeout)
            if (val != len(data)):
                raise ValueError('Error writing to multibus')
            else:
                return True

    def read_multibus_int(self, register_number):
        """
        Read from the QS1R I2C multibus registers.
        """
        if not self.found:
            raise ValueError('Not connected to device')
            return None
        else:
            val = self.dev.ctrl_transfer(bmRequestType=RequestType.VRT_VENDOR_IN,
                                         bRequest=Request.VRQ_MULTI_READ,
                                         wValue=register_number,
                                         wIndex=0,
                                         data_or_wLength=4,
                                         timeout=self.timeout)
            return struct.unpack('<I', val)[0]

def _main():
    qsio = QsIO()
    devs = qsio.find_devices()
    if (len(devs) > 0):
        qsio.use_device(0)
    if qsio.found == True:
        print 'device found'
    raw_input('press return to quit')

if __name__ == '__main__':
    _main()
    
