#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2017 by Thomas C. McDermott, N5EG.
# 
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3, or (at your option)
# any later version.
# 
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this software; see the file COPYING.  If not, write to
# the Free Software Foundation, Inc., 51 Franklin Street,
# Boston, MA 02110-1301, USA.
# 

import numpy as np
from gnuradio import gr
import serial
import pmt as pmt


# AMBE 3000 programming strings

reset = bytearray.fromhex("61 00 07 00 34 05 00 00 0F 00 00")
setDstar = bytearray.fromhex("61 00 0d 00 0a 01 30 07 63 40 00 00 00 00 00 00 48")
getProdId = bytearray.fromhex("61 00 01 00 30")
getVersion = bytearray.fromhex("61 00 01 00 31")
# setDMR = bytearray.fromhex("61 00 0D 00 0A 04 31 07 54 24 00 00 00 00 00 6F 48")
encodeAMBE = bytearray.fromhex("61 00 0B 01 01 48")
encodePCM = bytearray.fromhex("61 01 42 02")
silence = bytearray.fromhex("AC AA 40 20 00 44 40 80 80")




class ThumbDV(gr.sync_block):
    """
    docstring for block ThumbDV
    """
    def __init__(self, DVmode, DeviceID):
        gr.sync_block.__init__(self,
            name="ThumbDV",
            in_sig=[np.float32],
            out_sig=[np.float32])

# define message input and a message output ports

        self.message_port_register_out(pmt.intern('msg_out'))
        self.message_port_register_in(pmt.intern('msg_in'))
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_msg)


        self.DVmode = DVmode
        self.DeviceID = DeviceID   # string, for example: /dev/ttyUSB0
#        self.port = None

        self.port = self.open_port(self.DeviceID)

        if self.port:
            self.port.flushInput()
            self.port.flushOutput()
        else:
            print 'Fatal: Cannot open DVSI on serial port'

        self.port.write(reset)
        print 'Reset Response : ',
        leng, dat = self.DVSIRecv(self.port)


# TODO just for initial debug
	self.port.write(getProdId)
        print 'Product ID: ',
        leng, dat = self.DVSIRecv(self.port)
	self.port.write(getVersion)
        print 'Product Version: ',
        leng, dat = self.DVSIRecv(self.port)
# back to our regular programming

	self.port.write(setDstar)
        print 'DSTAR Setup Response: ',
        leng, dat = self.DVSIRecv(self.port)




    def work(self, input_items, output_items):
        in0 = input_items[0]
        out = output_items[0]
        # <+signal processing here+>
        out[:] = in0
        return len(output_items[0])



    def handle_msg(self, msg):
        # handle the received message
 #       self.message_port_pub(pmt.intern('msg_out'), pmt.intern('message received!'))        

    def send_msg(self, msg):
        self.message_port_pub(pmt.intern("<port name>"), <pmt message>)

 
# GRC callback handler
    def set_DVmode(self, DVmode):
        self.DVmode = DVmode
        # TODO reset and re-configure the ThumbDV device
        return


    def open_port(self, DeviceID):
       # Open Serial port to the device
        port = serial.Serial(DeviceID, baudrate=460800, timeout=1.0,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, xonxoff=False,
            rtscts=False, dsrdtr=False)

        if port:
            print 'Port = ', port
        port.flushInput()
        port.flushOutput()
        return port


    def DVSIRecv(self, port):
    # Only use timeout during initialization, not during work.
    # Otherwise the gnuradio work thread could be blocked (bad).

        rcvheader = port.read(4)              # Received serial packet length

        print 'RecvHeader:',
        for i in range(0, len(rcvheader)):
            print hex(ord(rcvheader[i])),
#        print '   Len of RecvHeader=', len(rcvheader)

        if (ord(rcvheader[0]) != 0x61):
            print 'Invalid response type from serial DVSI read:',
            print '[0] = ', ord(rcvheader[0])
            return len(rcvheader), ''

        # may have data from multiple responses, read only one response
        if len(rcvheader) == 4:
            packetLen = (ord(rcvheader[1]) * 256) + ord(rcvheader[2])  
            data = self.port.read(packetLen)  # Received serial packet data
            packet = rcvheader + data           


            print 'RecvData:',
            for i in range(0, len(data)):
                print hex(ord(data[i])),
#            print '   Len of RecvData=', len(data)


                       
        else:
            print 'Error: received serial port DVSI header error'
            return 0, ''

        return packetLen, packet


'''
 some pyserial notes, methods, and parameters.  Names will change in version 3

* Transmit is blocking unless transmit timeout is set (defaults to zero).
* Receive timeout=0.0 is non-blocking, returns zero or more up to requested bytes
* inWaiting  number of bytes waiting to be read from serial input buffer
* outWaiting  number of bytes in the serial output buffer

'''

# Override the base class stop method to close the serial port when the
# flowgraph is terminated

#    def stop(self):
#        if(self.port):
#            self.port.close()
#        super(stop, self)
#        return


    

