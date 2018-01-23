#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 
# Copyright 2018  Thomas C. McDermott, N5EG.
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
setDMR = bytearray.fromhex("61 00 0D 00 0A 04 31 07 54 24 00 00 00 00 00 6F 48")
decodeAMBE = bytearray.fromhex("61 00 0B 01 01 48")
encodePCM = bytearray.fromhex("61 01 42 02 00 A0")
silence = bytearray.fromhex("AC AA 40 20 00 44 40 80 80")



class ThumbDV(gr.basic_block):
    """
    docstring for block ThumbDV
    """
    def __init__(self, DVmode, DeviceID):
        gr.basic_block.__init__(self,
            name="ThumbDV",
            in_sig=[np.float32],
            out_sig=[np.float32])

        self.message_port_register_out(pmt.intern('msg_out'))
        self.message_port_register_in(pmt.intern('msg_in'))
        self.set_msg_handler(pmt.intern('msg_in'), self.handle_rxmsg)

#        self.set_min_output_buffer(160)  #Does not work in Python, only C++ 

        self.DVmode = DVmode
        self.DeviceID = DeviceID   # string, for example: /dev/ttyUSB0
        self.InQueueDepth = 0      # input stream groups - output stream groups

        self.port = self.open_port(self.DeviceID)

        if self.port:
            self.port.flushInput()
            self.port.flushOutput()
        else:
            print 'FATAL: Cannot open DVSI on serial port'

        self.RxAwaiting = 0  # idle
        self.RxData = 0
        self.RxCommand = 0


# TODO Does Reset always give a serial response?  Do we need to block
# TODO until one arrives?

        self.DVSISend(self.port, reset, '')
        print 'Reset Response : ',
        command, dat = self.DVSIRecv(self.port, block=True)
        print command

## TODO just for initial debug
#	self.port.write(getProdId)
#        print 'Product ID: ',
#        leng, dat = self.DVSIRecv(self.port)
#	self.port.write(getVersion)
#        print 'Product Version: ',
#        leng, dat = self.DVSIRecv(self.port)
# back to our regular programming


        if self.DVmode == 0:
            self.DVSISend(self.port, setDstar, '')
            print 'DSTAR Setup Response: ',
        elif self.DVmode == 1:
            self.DVSISend(self.port, setDMR, '')
            print 'DMR Setup Response: ',    
        else:
            print 'Invalid Voice Mode Selected'
        command, dat = self.DVSIRecv(self.port, block=True)
        print command


    def forecast(self, noutput_items, ninput_items_required):
        #setup size of input_items[0] for work call
        # forecast the number of input items required per output
        # item produced.  160 bytes out per 160 bytes in --> 1:1 ratio
        # Does not need to be exact, but should be close
        ninput_items_required[0] = noutput_items


    def general_work(self, input_items, output_items):
        ''' Receive float32 samples on the streaming input.
            Convert to 2's complement int16, batch into 20 msec groups
            and send to the AMBE encoder using serial out.
            Receive AMBE encoded bytes on message in, send to 
            AMBE decoder, receive back int16 samples. Convert
            to float32 and send to streaming output port.

            Conversions need to be done by numpy for run-time efficiency.

            During work, read the serial port for response
            packets from the AMBE chip. ACKs are thrown away,
            encoded and decoded voice samples are utilized.
         '''

# Problem with over-running the DVSI chip serial input. The code pulls
# samples from the input stream as fast as possible without regard to how
# fast output samples are being pulled.  Each input pull sends a serial
# packet to the DVSI chip, which buffers some packets but then seems
# to lose sync.
#
# Revise to count how many (input - output) stream groups (of 160 samples)
# are allowed which is roughly the input queue depth to the DVSI chip.
# Limit pulling of input stream samples so that (input - output) groups is
# bounded to be less than about 5.   There needs to be a few packets in
# flight to the DVSI input without any yet produced output samples because
# the DVSI chip delay is about 56 milliseconds but the packetization rate
# is 20 milliseconds. Thus there will be 3 or so packets in the DVSI
# pipeline at any time.

        inp = input_items[0]
        out = output_items[0]

        if len(out) < 160:     # require minimum output buffer size
            return 0           # because set.min_output_buffer doesn't work

# if there are >= 160 samples on the streaming input then read,
# convert to [np.int16 big-endian], scale to 16-bit dynamic range,
# and send to the DVSI chip to encode PCM to AMBE.

        if len(inp) >= 160:
            if self.InQueueDepth <= 5:
                PCMf = inp[0:160] * 32768.0  # scalar multiply because numpy array
                PCMint = np.array(PCMf, dtype=np.dtype('>i2'))
                self.consume(0, 160)    # consume 160 floats from input[0]
                self.InQueueDepth += 1
                self.DVSISend(self.port, encodePCM, bytearray(PCMint))


#Check the DVSI to see if it sent us any serial data.  If it's a command
#response then throw away.  If it's an encode response, send the encoded bytes
#to the message output port. If it's a decode response, send the PCM payload
#to the streaming output port as floating point.
#
#   In DSTAR mode:
#        9 bytes encoded are at a bit rate of 3600 baud, of which
#          2/3 is data and 1/3 is FEC.
#        The bytes are FEC+CODEC decoded using the ThumbDV chip into
#          160 x int16 voice samples (20 milliseconds at 8 ksps rate).         
#        Append the 9 data bytes to an AMBE encode message and send to the
#          DVSI device.
#        The chip has ~58 msec delay, command responses are pipelined with
#          PCM audio and AMBE samples coming back the serial port
#          sometime later.
#

        data, command = self.DVSIRecv(self.port)   # non-blocking

#        if command == -1:  # command error
#            return 0
#
#        if command == 0:   # no or partial response
#            return 0    

        if command == 1:    # received encoded samples
            if len(data) == 9:
                dat = np.frombuffer(data, dtype=np.uint8)
                self.send_msg(dat)  #send out the message port
                return 0	 # did not produce output stream samples
            else:
                print ("AMBE Encoder invalid packet length : ", len(data))
                return 0

        elif command == 2:    # received decoded samples
            if len(data) == 320:  
                # Convert to [np.float32] then fp normalize
                ints = np.frombuffer(data, dtype=np.dtype('>i2'))
                floats = np.array(ints, dtype=np.float32)
                floats = floats / 32768.0  # scalar because numpy array
                out[0:160] = floats
                self.InQueueDepth -= 1
                if self.InQueueDepth < 0:
                    print 'ERROR: Underrun of InQueueDepth: ', self.InQueueDepth
                return 160    # produced 160 samples on the output stream
            else:
                print ("AMBE Decoder invalid packet length : ", len(data))
                return 0

	else:
            return 0


    def handle_rxmsg(self, msg):
        '''
        Decode received message: PMT array of 9, dtype=np.uint8
        received at a 20 milliseconds/message rate.  Send to ThumbDV
        and ask it to decode to audio PCM.
        '''
        dat = pmt.to_python(msg)  # dat is an np.array[9] of type np.uint8
        self.DVSISend(self.port, decodeAMBE, dat.tobytes())
              

    def send_msg(self, msg):
        '''
        Received a block of encodedAMBE bytes from the serial port in
        response to an encode request as a python array. Convert to PMT array
        of type np.uint8 and send out msg_out port.
        '''
        dat = pmt.to_pmt(msg)
        self.message_port_pub(pmt.intern('msg_out'), dat)


# GRC callback handler
    def set_DVmode(self, DVmode):
        self.DVmode = DVmode
        # TODO reset and re-configure the ThumbDV device
        return

    def set_DeviceID(self, DeviceID):
        self.DeviceID = DeviceID
        return


    def open_port(self, DeviceID):
       # Open Serial port to the device in non-blocking mode
        port = serial.Serial(DeviceID, baudrate=460800, timeout=0,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE, xonxoff=False,
            rtscts=False, write_timeout=0, dsrdtr=False)

        port.flushInput()
        port.flushOutput()
        return port



    def DVSISend(self, port, command, data):
        '''Non-blocking mode: output buffer could be full.
           Check the # bytes to see if there's room.
        ''' 
        if port.out_waiting < (1020 - len(command) - len(data)):
            x = port.write(command)
            y = 0
            if len(data) >= 0:
                y = port.write(data)
            return 0
        else:
            print "wsO"   # write to serial Overflowed
            return -1	  # command and data were not sent


    def DVSIRecv(self, port, block=False):
#       '''Non-blocking mode: serial.read() can return zero bytes.
#          The receive buffer normally holds maximum of 1020 bytes.
#          Use state machine to process one complete packet at a time.
#
#          Command Return Values:
#          ----------------------
#          -1  Error.
#           0  No data for caller (which could also be a 
#                Channel 0 (command ACK) response, which we don't use.
#           1  Channel 1 (AMBE encoded samples) response.
#           2  Channel 2 (PCM samples) response.
#       '''

        if block:
            exhaust = 0
            while port.in_waiting < 5:
                exhaust += 1
                if exhaust > 20000000:
                    print 'DVSIRecv Blocking Exhaust counter expired'
                    return "", -1


        if self.RxAwaiting == 0:    # receiver is idle
            if port.in_waiting < 4:   # 4 is packet header size
                return "", 0
            else:
                rcvheader = port.read(4)    # Received serial packet length
                if (ord(rcvheader[0]) != 0x61):
                    print 'Invalid response type from serial DVSI read:',
                    print '[0] = ', ord(rcvheader[0])
                    self.RxAwaiting = 0
                    return "", -1     # error trying to receive
                else:               
                    packetLen = (ord(rcvheader[1]) * 256) + ord(rcvheader[2])
                    self.RxAwaiting = packetLen  # how many characters to await
                    self.RxCommand = ord(rcvheader[3])  # the command we got

        if port.in_waiting < self.RxAwaiting:
            return "", 0      # do not have the whole dataframe yet
        else:
            if self.RxCommand == 0:
                self.RxData = port.read(self.RxAwaiting)  
            elif self.RxCommand == 1:
                dummy = port.read(2) # strip payload header
                self.RxData = port.read(self.RxAwaiting-2)
            elif self.RxCommand == 2:
                dummy = port.read(2) # strip payload header
                self.RxData = port.read(self.RxAwaiting-2)
            else:
                print "Bad command header received from DVSI"
                self.RxAwaiting = 0
                return "", -1
 
            self.RxAwaiting = 0  # finished rx'ing packet, wait for next
            return self.RxData, self.RxCommand


