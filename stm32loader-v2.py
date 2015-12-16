#!/usr/bin/env python3

# -*- coding: utf-8 -*-
# vim: sw=4:ts=4:si:et:enc=utf-8

# Author: Ivan A-R <ivan@tuxotronic.org>
# Project page: http://tuxotronic.org/wiki/projects/stm32loader
#
# This file is part of stm32loader.
#
# stm32loader is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free
# Software Foundation; either version 3, or (at your option) any later
# version.
#
# stm32loader is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License
# along with stm32loader; see the file COPYING3.  If not see
# <http://www.gnu.org/licenses/>.

import sys
import serial
import time
import argparse

try:
    from progressbar import *
    usepbar = 1
except:
    usepbar = 0

# Verbose level
QUIET = 20

# these come from AN2606
chip_ids = {
    0x412: "STM32 Low-density",
    0x410: "STM32 Medium-density",
    0x414: "STM32 High-density",
    0x420: "STM32 Medium-density value line",
    0x428: "STM32 High-density value line",
    0x430: "STM32 XL-density",
    0x416: "STM32 Medium-density ultralow power line",
    0x411: "STM32F2xx",
    0x413: "STM32F4xx",
}

def mdebug(level, message):
    if(QUIET >= level):
        print(message)
        #print >> sys.stderr , message


class CmdException(Exception):
    pass

class CommandInterface:
    extended_erase = 0

    def open(self, aport='/dev/tty.usbserial-ftCYPMYJ', abaudrate=115200) :
        self.sp = serial.Serial(
            port=aport,
            baudrate=abaudrate,     # baudrate
            bytesize=8,             # number of databits
            parity=serial.PARITY_EVEN,
            stopbits=1,
            xonxoff=0,              # don't enable software flow control
            rtscts=0,               # don't enable RTS/CTS flow control
            timeout=5               # set a timeout value, None for waiting forever

        )

    def _wait_for_ask(self, info = ""):
        # wait for ask
        try:
            ask = ord(self.sp.read())
            #print(hex(ask))
        except:
            raise CmdException("Can't read port or timeout")
        else:
            if ask == 0x79:
                # ACK
                return 1
            else:
                if ask == 0x1F:
                    # NACK
                    raise CmdException("NACK "+info)
                else:
                    # Unknown responce
                    raise CmdException("Unknown response. "+info+": "+hex(ask))

    def _wait_for_initchip_ask(self, info = ""):
        # wait for ask
        # bootloader V22  ,
        # send 0x7f, ack 0x7f 0x79
        while True:
            try:
                ask = ord(self.sp.read())
            except:
                raise CmdException("Can't read port or timeout")
            else:
                if ask == 0x79:
                    # ACK
                    return 1
                elif ask == 0x7f:
                   continue 
                else:
                    if ask == 0x1F:
                        # NACK
                        raise CmdException("NACK "+info)
                    else:
                        # Unknown responce
                        raise CmdException("Unknown response. "+info+": "+hex(ask))


    def reset(self):
        self.sp.setDTR(0)
        time.sleep(0.1)
        self.sp.setDTR(1)
        time.sleep(0.5)

    def initChip(self):
        # Set boot
        self.sp.setRTS(0)
        self.reset()
        self.sp.write(b'\x7F')       # Syncro
        return self._wait_for_initchip_ask("Syncro")

    def releaseChip(self):
        self.sp.setRTS(1)
        self.reset()

    def cmdGeneric(self, cmd):
        self.sp.write(bytes([cmd]))
        self.sp.write(bytes([cmd ^ 0xFF]))# Control byte
        return self._wait_for_ask(hex(cmd))

    def cmdGet(self):
        if self.cmdGeneric(0x00):
            mdebug(10, "*** Get command");
            len = ord(self.sp.read())
            version = ord(self.sp.read())
            mdebug(10, "    Bootloader version: "+hex(version))
            dattmp = self.sp.read(len)

            #if 0x44 in dat:
            if dattmp.find(b'\x44') != -1:
                self.extended_erase = 1

            i = 0
            dat = []
            while i < len:
                dat.append(hex(dattmp[i]))
                i +=1
            mdebug(10, "    Available commands: "+", ".join(dat))
            self._wait_for_ask("0x00 end")
            return version
        else:
            raise CmdException("Get (0x00) failed")

    def cmdGetVersion(self):
        if self.cmdGeneric(0x01):
            mdebug(10, "*** GetVersion command")
            version = ord(self.sp.read())
            self.sp.read(2)
            self._wait_for_ask("0x01 end")
            mdebug(10, "    Bootloader version: "+hex(version))
            return version
        else:
            raise CmdException("GetVersion (0x01) failed")

    def cmdGetID(self):
        if self.cmdGeneric(0x02):
            mdebug(10, "*** GetID command")
            len = ord(self.sp.read())
            id = self.sp.read(len+1)
            self._wait_for_ask("0x02 end")
            i = 0
            v = 0
            while i < (len+1):
                v = v*0x100
                v += id[i]
                i+= 1
            #print(hex(v))
            return v

            #return reduce(lambda x, y: x*0x100+y, map(ord, id))
        else:
            raise CmdException("GetID (0x02) failed")


    def _encode_addr(self, addr):
        byte3 = (addr >> 0) & 0xFF
        byte2 = (addr >> 8) & 0xFF
        byte1 = (addr >> 16) & 0xFF
        byte0 = (addr >> 24) & 0xFF
        crc = byte0 ^ byte1 ^ byte2 ^ byte3
        return (bytes([byte0]) + bytes([byte1]) + bytes([byte2]) + bytes([byte3]) + bytes([crc]))


    def cmdReadMemory(self, addr, lng):
        assert(lng <= 256)
        if self.cmdGeneric(0x11):
            mdebug(10, "*** ReadMemory command")
            self.sp.write(self._encode_addr(addr))
            self._wait_for_ask("0x11 address failed")
            N = (lng - 1) & 0xFF
            crc = N ^ 0xFF
            self.sp.write(bytes([N]) + bytes([crc]))
            self._wait_for_ask("0x11 length failed")
            return self.sp.read(lng)
            #return map(lambda c: ord(c), self.sp.read(lng))
        else:
            raise CmdException("ReadMemory (0x11) failed")


    def cmdGo(self, addr):
        if self.cmdGeneric(0x21):
            mdebug(10, "*** Go command")
            self.sp.write(self._encode_addr(addr))
            self._wait_for_ask("0x21 go failed")
        else:
            raise CmdException("Go (0x21) failed")


    def cmdWriteMemory(self, addr, data):
        assert(len(data) <= 256)
        if self.cmdGeneric(0x31):
            mdebug(10, "*** Write memory command")
            self.sp.write(self._encode_addr(addr))
            self._wait_for_ask("0x31 address failed")
            lng = (len(data)-1) & 0xFF
            mdebug(10, "    %s bytes to write" % [lng+1]);
            self.sp.write(bytes([lng])) # len really
            crc = 0xFF
            for c in data:
                crc = crc ^ c
                self.sp.write(bytes([c]))
            self.sp.write(bytes([crc]))
            self._wait_for_ask("0x31 programming failed")
            mdebug(10, "    Write memory done")
        else:
            raise CmdException("Write memory (0x31) failed")


    def cmdEraseMemory(self, sectors = None):
        if self.extended_erase:
            return cmd.cmdExtendedEraseMemory()

        if self.cmdGeneric(0x43):
            mdebug(10, "*** Erase memory command")
            if sectors is None:
                # Global erase
                self.sp.write(b'\xFF')
                self.sp.write(b'\x00')
            else:
                # Sectors erase
                self.sp.write(bytes([(len(sectors)-1) & 0xFF]))
                crc = 0xFF
                for c in sectors:
                    crc = crc ^ c
                    self.sp.write(bytes([c]))
                self.sp.write(bytes([crc]))
            self._wait_for_ask("0x43 erasing failed")
            mdebug(10, "    Erase memory done")
        else:
            raise CmdException("Erase memory (0x43) failed")

    def cmdExtendedEraseMemory(self):
        if self.cmdGeneric(0x44):
            mdebug(10, "*** Extended Erase memory command")
            # Global mass erase
            self.sp.write('\xFF')
            self.sp.write('\xFF')
            # Checksum
            self.sp.write('\x00')
            tmp = self.sp.timeout
            self.sp.timeout = 30
            print("Extended erase (0x44), this can take ten seconds or more")
            self._wait_for_ask("0x44 erasing failed")
            self.sp.timeout = tmp
            mdebug(10, "    Extended Erase memory done")
        else:
            raise CmdException("Extended Erase memory (0x44) failed")

    def cmdWriteProtect(self, sectors):
        if self.cmdGeneric(0x63):
            mdebug(10, "*** Write protect command")
            self.sp.write(bytes([(len(sectors)-1) & 0xFF]))
            crc = 0xFF
            for c in sectors:
                crc = crc ^ c
                self.sp.write(bytes([c]))
            self.sp.write(bytes([crc]))
            self._wait_for_ask("0x63 write protect failed")
            mdebug(10, "    Write protect done")
        else:
            raise CmdException("Write Protect memory (0x63) failed")

    def cmdWriteUnprotect(self):
        if self.cmdGeneric(0x73):
            mdebug(10, "*** Write Unprotect command")
            self._wait_for_ask("0x73 write unprotect failed")
            self._wait_for_ask("0x73 write unprotect 2 failed")
            mdebug(10, "    Write Unprotect done")
        else:
            raise CmdException("Write Unprotect (0x73) failed")

    def cmdReadoutProtect(self):
        if self.cmdGeneric(0x82):
            mdebug(10, "*** Readout protect command")
            self._wait_for_ask("0x82 readout protect failed")
            self._wait_for_ask("0x82 readout protect 2 failed")
            mdebug(10, "    Read protect done")
        else:
            raise CmdException("Readout protect (0x82) failed")

    def cmdReadoutUnprotect(self):
        if self.cmdGeneric(0x92):
            mdebug(10, "*** Readout Unprotect command")
            self._wait_for_ask("0x92 readout unprotect failed")
            self._wait_for_ask("0x92 readout unprotect 2 failed")
            mdebug(10, "    Read Unprotect done")
        else:
            raise CmdException("Readout unprotect (0x92) failed")


# Complex commands section

    def readMemory(self, addr, lng):
        data = b'' 
        if usepbar:
            widgets = ['Reading: ', Percentage(),', ', ETA(), ' ', Bar()]
            pbar = ProgressBar(widgets=widgets,maxval=lng, term_width=79).start()
        
        while lng > 256:
            if usepbar:
                pbar.update(pbar.maxval-lng)
            else:
                mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
            data = data + self.cmdReadMemory(addr, 256)
            addr = addr + 256
            lng = lng - 256
        if usepbar:
            pbar.update(pbar.maxval-lng)
            pbar.finish()
        else:
            mdebug(5, "Read %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        data = data + self.cmdReadMemory(addr, lng)
        return data

    def writeMemory(self, addr, data):
        lng = len(data)
        if usepbar:
            widgets = ['Writing: ', Percentage(),' ', ETA(), ' ', Bar()]
            pbar = ProgressBar(widgets=widgets, maxval=lng, term_width=79).start()
        
        offs = 0
        while lng > 256:
            if usepbar:
                pbar.update(pbar.maxval-lng)
            else:
                mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
            self.cmdWriteMemory(addr, data[offs:offs+256])
            offs = offs + 256
            addr = addr + 256
            lng = lng - 256
        if usepbar:
            pbar.update(pbar.maxval-lng)
            pbar.finish()
        else:
            mdebug(5, "Write %(len)d bytes at 0x%(addr)X" % {'addr': addr, 'len': 256})
        self.cmdWriteMemory(addr, data[offs:offs+lng] + (b'\xFF' * (256-lng)) )


#	def __init__(self) :
#        pass

def get_opt():
    parser = argparse.ArgumentParser(prog=sys.argv[0], 
            usage='%(prog)s [-hqVewvr] [-l length] [-p port] [-b baud] [-a addr] [-g addr] [file.bin]',
            description='load stm32fxx\'s firmware by uart',
            epilog='python3.4 stm32loader-v2.py -ewv example/main.bin')
    parser.add_argument('-q', dest='Quiet',action='store_true',
            help='Quiet         [default:off]')
    parser.add_argument('-V',dest='Verbose',action='store_true',
            help='Verbose   [default:off]')
    parser.add_argument('-e', dest='Erase', action='store_true', 
            help='Erase     [default:off]')
    parser.add_argument('-w', dest='Write', action='store_true', 
            help='Write     [default:off]')
    parser.add_argument('-v', dest='Verify', action='store_true', 
            help='Verify    [default:off]')
    parser.add_argument('-r', dest='Read', action='store_true', 
            help='Read   [default:off]')

    parser.add_argument('-l', dest='Length',metavar='length',type=int,
            help='Length of read')
    parser.add_argument('-p', dest='Port', metavar='port',  
            help='Serial port [default:/dev/ttyS0]')
    parser.add_argument('-b', dest='Baud', metavar='baud', type=int, 
            help='Baud speed [default:115200]')
    parser.add_argument('-a', dest='A', metavar='addr', 
            help='Target address')
    parser.add_argument('-g', dest='G', metavar='addr', 
            help='Address to start running at [0x08000000, usually]')
    parser.add_argument('file',nargs='?',  
            help='[open file]')

    args = parser.parse_args()
    return args

if __name__ == "__main__":
    
    # Import Psyco if available
    try:
        import psyco
        psyco.full()
        print("Using Psyco...")
    except ImportError:
        pass

    conf = {
            'port': '/dev/tty.usbserial-ftCYPMYJ',
            'baud': 115200,
            'address': 0x08000000,
            'len': 0,
            'erase': 0,
            'write': 0,
            'verify': 0,
            'read': 0,
            'go_addr':-1,
        }

# http://www.python.org/doc/2.5.2/lib/module-getopt.html

    args = get_opt()

    QUIET = 5
    
    if args.Verbose:
        QUIET = 10
    if args.Quiet:
        QUIET = 0

    if args.Erase:
        conf['erase'] = 1
    if args.Write:
        conf['write'] = 1
    if args.Verify:
        conf['verify'] = 1
    if args.Read:
        conf['read'] = 1
    if args.Length:
        conf['len'] = args.Length

    if args.Port:
        conf['port'] = args.Port
    if args.Baud:
        conf['baud'] = args.Baud
    if args.A:
        conf['address'] = eval(args.A)
    if args.G:
        conf['go_addr'] = eval(args.G)

    cmd = CommandInterface()
    cmd.open(conf['port'], conf['baud'])
    mdebug(10, "Open port %(port)s, baud %(baud)d" % {'port':conf['port'], 'baud':conf['baud']})
    try:
        try:
            cmd.initChip()
        except:
            print( "Can't init. Ensure that BOOT0 is enabled and reset device")


        bootversion = cmd.cmdGet()
        mdebug(0, "Bootloader version %X" % bootversion)
        id = cmd.cmdGetID()
        mdebug(0, "Chip id: 0x%x (%s)" % (id, chip_ids.get(id, "Unknown")))
#        cmd.cmdGetVersion()
        #cmd.cmdReadoutProtect()
        #cmd.cmdReadoutUnprotect()
        #cmd.cmdWriteUnprotect()
        #cmd.cmdWriteProtect([0, 1])

        if (conf['write'] or conf['verify']):
            fh = open(args.file,'rb')
            data = fh.read()
            #print(len(data))

        if conf['erase']:
            cmd.cmdEraseMemory()

        if conf['write']:
            cmd.writeMemory(conf['address'], data)

        if conf['verify']:
            verify = cmd.readMemory(conf['address'], len(data))
            if(data == verify):
                print( "Verification OK")
            else:
                print( "Verification FAILED")
                print( str(len(data)) + ' vs ' + str(len(verify)))
                for i in xrange(0, len(data)):
                    if data[i] != verify[i]:
                        print( hex(i) + ': ' + hex(data[i]) + ' vs ' + hex(verify[i]))

        if not conf['write'] and conf['read']:
            rdata = cmd.readMemory(conf['address'], conf['len'])
            fh = open(args.file, 'wb')
            fh.write(rdata)

        if conf['go_addr'] != -1:
            cmd.cmdGo(conf['go_addr'])

    finally:
        cmd.releaseChip()

