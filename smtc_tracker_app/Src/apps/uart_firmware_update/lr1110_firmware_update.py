# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# LR1110 Firmware Update
# -----------------------------------------------------------------------------
# Revised BSD License
# Copyright Semtech Corporation 2021. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Semtech corporation nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# -----------------------------------------------------------------------------

import serial
import struct
import array
import argparse

# -----------------------------------------------------------------------------

GET_VERSION_CMD = 0x00
GET_VERSION_LEN = 0x0000
GET_VERSION_ANSWER_LEN = 0x0004

GET_MODEM_E_VERSION_CMD = 0x01
GET_MODEM_E_VERSION_LEN = 0x0000
GET_MODEM_E_VERSION_ANSWER_LEN = 0x000E

WRITE_LR1110_UPDATE_CMD = 0x02
WRITE_LR1110_UPDATE_LEN = 0x0002 + 0x0100
WRITE_LR1110_UPDATE_ANSWER_LEN = 0x0002


# -----------------------------------------------------------------------------

class LR1110:
    """
    Proxy class for updating a LR1110 through a serial link.
    """

    def __init__( self, uart) :
        self.uart = uart
        self.fragment_id = 0

    def get_version( self ):
        """
        Return a tuple containing the hardware version, type and firmware version in this order.
        This will only work when the LR1110 runs a Transceiver firmware or is in bootloader mode.
        """
        # Prepare and send the command
        cmd = struct.pack( "<BH", GET_VERSION_CMD, GET_VERSION_LEN )
        self.uart.write( cmd )
        # Get and parse the answer
        ans = uart.read( 3 + GET_VERSION_ANSWER_LEN )
        t, l, hw, type, fw = struct.unpack( "<BHBBH", ans )
        return hw, type, fw

    def get_modem_e_version( self) :
        """
        Return a tuple containing the bootloader version, functionality, firmware and LoRaWAN version in this order.
        This will only work when the LR1110 runs a Modem-E firmware.
        """
        # Prepare and send the command
        cmd = struct.pack( "<BH", GET_MODEM_E_VERSION_CMD, GET_MODEM_E_VERSION_LEN )
        self.uart.write( cmd )
        # Get and parse the answer
        ans = uart.read( 3 + GET_MODEM_E_VERSION_ANSWER_LEN )
        t, l, bl, func, fw, lw = struct.unpack( "<BHIIIH", ans )
        return bl, func, fw, lw

    def write_lr1110_update( self, fragment_bytes ):
        """
        Write a LR1110 firmware fragment.
        Writing the first fragment will reboot the LR1110 in bootloader mode.
        Writing the last fragment will reboot the LR1110 that will then run the updated firmware.
        """
        if len( fragment_bytes ) > WRITE_LR1110_UPDATE_LEN:
            print( "Invalid fragment size" )
            return
        # Prepare and send the command
        cmd = struct.pack( "<BH", WRITE_LR1110_UPDATE_CMD, len( fragment_bytes ) + 2 )
        cmd += struct.pack( "<H", self.fragment_id )
        cmd += fragment_bytes
        print( "Sending fragment %d... " % self.fragment_id, end='' )
        self.uart.write( cmd )
        print( "done." )
        # Get and parse the answer
        ans = uart.read( WRITE_LR1110_UPDATE_ANSWER_LEN + 3 )
        t, l, fragment_id = struct.unpack( "<BHH", ans )
        if fragment_id != self.fragment_id:
            print( "ans.fragment_id: %x, expected: %x" % ( fragment_id, self.fragment_id ) )
        self.fragment_id += 1

# -----------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser( description="Update the LR1110 firmware of a device over a serial link" )
    parser.add_argument( "--port", help = "Device name of the serial port" )
    parser.add_argument( "--file", help = "Name of the LR1110 firmware binary file" )
    parser.add_argument( "type", help = "Type of firmware", nargs="?", choices=( "modem", "transceiver" ), default="modem" )

    args = parser.parse_args( )

    with serial.Serial( args.port, 921600, timeout=10 ) as uart:
        lr1110 = LR1110( uart )
        with open( args.file, "rb" ) as infile:
            total_bytes = 0
            # Transmit fragments of 64 32-bits words
            bytes_read = infile.read( 256 )
            while bytes_read:
                # Byte swap every 32-bit word from the binary file.
                total_bytes += len( bytes_read )
                word_array = array.array( 'I' )
                word_array.frombytes( bytes_read )
                word_array.byteswap( )
                lr1110.write_lr1110_update( word_array.tobytes( ) )
                bytes_read = infile.read(256)
        # Verification
        print( "Firmware version:" )
        if args.type == "modem":
            bl, func, fw, lw = lr1110.get_modem_e_version( )
            print( "  Bootloader: 0x%.8x" % bl )
            print( "  Functionality: 0x%.8x" % func )
            print( "  Firmware: 0x%.8x" % fw )
            print( "  LoRaWAN: 0x%.8x" % lw )
        else:
            hw, type, fw = lr1110.get_version( )
            print( "  HW: 0x%x" % hw )
            print( "  Type: 0x%x" % type )
            print( "  FW: 0x%x" % fw )
