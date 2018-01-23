
Gnuradio interface for NWDR Thumb DV USB Stick
==============================================

This module implements an interface from gnuradio (and GRC) to
a North West Digital Radio (NWDR) AMBE Vocoder USB Stick
which contains a DVSI 3000 AMBE encoder/decoder chip.
The stick implements a USB based device that emulates
a serial interface at 460,800 bps.  You must specify which USB
interface you have the NWDR stick plugged into.  On Ubuntu this
would typically be something like /dev/ttyUSB0

Implemented in Python 2.7, not yet using standard make/build/install
method. To use this as is:

1. Copy the ThumbDV.py and ThumbDV.py.xml  modules into the
  .grc_gnuradio directory.
2. Hand-edit the path string in the XML file.

Expect this module to change as further related OOT modules are
implemented.

The gr-ThumbDV module encapsulates the interface to the
NWDR USB stick so that it provides 4 interfaces:
* 8 ksps 32-bit floating point input of unencoded audio samples.
  Samples on this input will be encoded by the DVSI chip.
* 8 ksps 32-bit floating point output of decoded audio samples.
  Samples on this output have been decoded by the DVSI chip.
* message input consisting of a 9-byte array containing one
  AMBE encoded sample that will be decoded by the DVSI chip.
* message output consisting of a 9-byte array with one AMBE
  encoded sample produced by the DVSI chip.

The module only encodes audio to messages and decodes
messages to audio.  To use this with DStar, DMR, etc requires
other modules that take the raw 4800 bps stream, synchronize,
frame, and insert and extract AMBE coded 72-bit audio packets.

The module by itself can only implement audio encode + decode
testing. Loop the message output back to the input to allow
testing of the DVSI chip function.


