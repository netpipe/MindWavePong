#########################################################################################
#      This file is part of mindwave-supercollider.                                     #
#                                                                                       #
#      mindwave-supercollider is free software: you can redistribute it and/or modify   #
#      it under the terms of the GNU General Public License as published by             #
#      the Free Software Foundation, either version 3 of the License, or                #
#      (at your option) any later version.                                              #
#                                                                                       #
#      mindwave-supercollider is distributed in the hope that it will be useful,        #
#      but WITHOUT ANY WARRANTY; without even the implied warranty of                   #
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                    #
#      GNU General Public License for more details.                                     #
#                                                                                       #
#      You should have received a copy of the GNU General Public License                #
#      along with mindwave-supercollider.  If not, see <http://www.gnu.org/licenses/>.  #
#########################################################################################

import select, serial, threading

# Byte codes
CONNECT = '\xc0'
DISCONNECT = '\xc1'
AUTOCONNECT = '\xc2'
SYNC = '\xaa'
EXCODE = '\x55'
POOR_SIGNAL = '\x02'
ATTENTION = '\x04'
MEDITATION = '\x05'
BLINK = '\x16'
HEADSET_CONNECTED = '\xd0'
HEADSET_NOT_FOUND = '\xd1'
HEADSET_DISCONNECTED = '\xd2'
REQUEST_DENIED = '\xd3'
STANDBY_SCAN = '\xd4'
ASIC_EEG_POWER = '\x83'
RAW_WAVE = '\x80'
BATTERY_CODE = '\x01'

# Status codes
STATUS_CONNECTED = 'connected'
STATUS_SCANNING = 'scanning'
STATUS_STANDBY = 'standby'

def decode_2s_complement_big_endian(binvalue):
  import bitstring
  b = bitstring.BitArray("0x"+binvalue.encode('hex'))
  return b.intbe

def decode_3byte_littleendian_unsigned(binvalue):
  import bitstring
  b = bitstring.BitArray("0x"+binvalue.encode('hex'))
  return b.uintle

class Headset(object):
    """
A MindWave Headset
"""

    class DongleListener(threading.Thread):
        """
Serial listener for dongle device.
"""
        def __init__(self, headset, *args, **kwargs):
            """Set up the listener device."""
            self.headset = headset
            super(Headset.DongleListener, self).__init__(*args, **kwargs)

        def run(self):
            """Run the listener thread."""
            s = self.headset.dongle

            # Re-apply settings to ensure packet stream
            s.write(DISCONNECT)
            d = s.getSettingsDict()
            for i in xrange(2):
                d['rtscts'] = not d['rtscts']
                s.applySettingsDict(d)

            while True:
                # Begin listening for packets
                try:
                    if s.read() == SYNC and s.read() == SYNC:
                        # Packet found, determine plength
                        while True:
                            plength = ord(s.read())
                            if plength != 170:
                                break
                        if plength > 170:
                            continue

                        # Read in the payload
                        payload = s.read(plength)

                        # Verify its checksum
                        val = sum(ord(b) for b in payload)
                        val &= 0xff
                        val = ~val & 0xff
                        chksum = ord(s.read())

                        if val == chksum:
                      #  if True: # ignore bad checksums
                            self.parse_payload(payload)
                        else:
                            for handler in self.headset.checksum_mismatch_handlers:
                              handler(self.headset, val, chksum)
                except (select.error, OSError):
                    break
                except serial.SerialException:
                    s.close()
                    break

        def parse_payload(self, payload):
            """Parse the payload to determine an action."""
            while payload:
                # Parse data row
                excode = 0
                try:
                    code, payload = payload[0], payload[1:]
                except IndexError:
                    pass
                while code == EXCODE:
                    # Count excode bytes
                    excode += 1
                    try:
                        code, payload = payload[0], payload[1:]
                    except IndexError:
                        pass
                if ord(code) < 0x80:
                    # This is a single-byte code
                    try:
                        value, payload = payload[0], payload[1:]
                    except IndexError:
                        pass
                    if code == POOR_SIGNAL:
                        # Poor signal
                        old_poor_signal = self.headset.poor_signal
                        self.headset.poor_signal = ord(value)
                        if self.headset.poor_signal > 0:
                            if old_poor_signal == 0:
                                for handler in \
                                    self.headset.poor_signal_handlers:
                                    handler(self.headset,
                                            self.headset.poor_signal)
                        else:
                            if old_poor_signal > 0:
                                for handler in \
                                    self.headset.good_signal_handlers:
                                    handler(self.headset,
                                            self.headset.poor_signal)
                    elif code == ATTENTION:
                        # Attention level
                        self.headset.attention = ord(value)
                        for handler in self.headset.attention_handlers:
                            handler(self.headset, self.headset.attention)
                    elif code == MEDITATION:
                        # Meditation level
                        self.headset.meditation = ord(value)
                        for handler in self.headset.meditation_handlers:
                            handler(self.headset, self.headset.meditation)
                    elif code == BLINK:
                        # Blink strength
                        self.headset.blink = ord(value)
                        for handler in self.headset.blink_handlers:
                            handler(self.headset, self.headset.blink)
                else:
                    # This is a multi-byte code
                    try:
                        vlength, payload = ord(payload[0]), payload[1:]
                    except IndexError:
                        continue
                    value, payload = payload[:vlength], payload[vlength:]
                    # Multi-byte EEG and Raw Wave codes not included
                    if code == ASIC_EEG_POWER:
                        # delta, theta, low-alpha, high-alpha, low-beta, high-beta, low-gamma, mid-gamma
                        delta = decode_3byte_littleendian_unsigned(value[0:3])
                        self.headset.delta = decode_3byte_littleendian_unsigned(value[0:3])
                        theta = decode_3byte_littleendian_unsigned(value[3:6])
                        self.headset.theta = decode_3byte_littleendian_unsigned(value[3:6])
                        lowalpha = decode_3byte_littleendian_unsigned(value[6:9])
                        self.headset.lowalpha = decode_3byte_littleendian_unsigned(value[6:9])
                        highalpha = decode_3byte_littleendian_unsigned(value[9:12])
                        self.headset.highalpha = decode_3byte_littleendian_unsigned(value[9:12])
                        lowbeta = decode_3byte_littleendian_unsigned(value[12:15])
                        self.headset.lowbeta = decode_3byte_littleendian_unsigned(value[12:15])
                        highbeta = decode_3byte_littleendian_unsigned(value[15:18])
                        self.headset.highbeta = decode_3byte_littleendian_unsigned(value[15:18])
                        lowgamma = decode_3byte_littleendian_unsigned(value[18:21])
                        self.headset.lowgamma = decode_3byte_littleendian_unsigned(value[18:21])
                        midgamma = decode_3byte_littleendian_unsigned(value[21:24])
                        self.headset.midgamma = decode_3byte_littleendian_unsigned(value[21:24])
                        for handler in self.headset.eeg_power_handlers:
                          handler(self.headset, delta, theta, lowalpha, highalpha, lowbeta, highbeta, lowgamma, midgamma)
                    elif code == RAW_WAVE:
                        hval = decode_2s_complement_big_endian(value)
                        for handler in self.headset.raw_wave_handlers:
                          handler(self.headset, hval)
                    # See Mindset Communications Protocol
                    elif code == HEADSET_CONNECTED:
                        # Headset connect success
                        run_handlers = self.headset.status != STATUS_CONNECTED
                        self.headset.status = STATUS_CONNECTED
                        self.headset.headset_id = value.encode('hex')
                        if run_handlers:
                            for handler in \
                                self.headset.headset_connected_handlers:
                                handler(self.headset)
                    elif code == HEADSET_NOT_FOUND:
                        # Headset not found
                        if vlength > 0:
                            not_found_id = value.encode('hex')
                            for handler in \
                                self.headset.headset_notfound_handlers:
                                handler(self.headset, not_found_id)
                        else:
                            for handler in \
                                self.headset.headset_notfound_handlers:
                                handler(self.headset, None)
                    elif code == HEADSET_DISCONNECTED:
                        # Headset disconnected
                        headset_id = value.encode('hex')
                        for handler in \
                            self.headset.headset_disconnected_handlers:
                            handler(self.headset, headset_id)
                    elif code == REQUEST_DENIED:
                        # Request denied
                        for handler in self.headset.request_denied_handlers:
                            handler(self.headset, payload.encode('hex'))

                    elif code == STANDBY_SCAN:
                        # Standby/Scan mode
                        try:
                            byte = ord(value[0])
                        except IndexError:
                            byte = None
                        if byte:
                            run_handlers = (self.headset.status !=
                                            STATUS_SCANNING)
                            self.headset.status = STATUS_SCANNING
                            if run_handlers:
                                for handler in self.headset.scanning_handlers:
                                    handler(self.headset)
                        else:
                            run_handlers = (self.headset.status !=
                                            STATUS_STANDBY)
                            self.headset.status = STATUS_STANDBY
                            if run_handlers:
                                for handler in self.headset.standby_handlers:
                                    handler(self.headset)


    def __init__(self, device, headset_id=None, open_serial=True):
        """Initialize the headset."""
        # Initialize headset values
        self.dongle = None
        self.listener = None
        self.device = device
        self.headset_id = headset_id
        self.poor_signal = 255
        self.attention = 0
        self.meditation = 0
        self.blink = 0
        self.status = None
        self.delta = 0
        self.theta = 0
        self.gamma = 0
        self.lowalpha = 0
        self.highalpha = 0
        self.lowbeta = 0
        self.highbeta = 0
        self.lowgamma = 0
        self.midgamma = 0


        # Create event handler lists
        self.poor_signal_handlers = []
        self.good_signal_handlers = []
        self.attention_handlers = []
        self.meditation_handlers = []
        self.blink_handlers = []
        self.headset_connected_handlers = []
        self.headset_notfound_handlers = []
        self.headset_disconnected_handlers = []
        self.request_denied_handlers = []
        self.scanning_handlers = []
        self.standby_handlers = []
        self.eeg_power_handlers = []
        self.raw_wave_handlers = []
        self.checksum_mismatch_handlers = []

        # Open the socket
        if open_serial:
            self.serial_open()

    def connect(self, headset_id=None):
        """Connect to the specified headset id."""
        if headset_id:
            self.headset_id = headset_id
        else:
            headset_id = self.headset_id
            if not headset_id:
                self.autoconnect()
                return
        self.dongle.write(''.join([CONNECT, headset_id.decode('hex')]))

    def autoconnect(self):
        """Automatically connect device to headset."""
        self.dongle.write(AUTOCONNECT)

    def disconnect(self):
        """Disconnect the device from the headset."""
        self.dongle.write(DISCONNECT)

    def serial_open(self):
        """Open the serial connection and begin listening for data."""
        # Establish serial connection to the dongle
        if not self.dongle or not self.dongle.isOpen():
            self.dongle = serial.Serial(self.device, 115200)

        # Begin listening to the serial device
        if not self.listener or not self.listener.isAlive():
            self.listener = self.DongleListener(self)
            self.listener.daemon = True
            self.listener.start()

    def serial_close(self):
        """Close the serial connection."""
        self.dongle.close()

if __name__ == "__main__":

    def attention_handler(headset, value):
      print ("Attention {0}".format(value))

    def meditation_handler(headset, value):
      print ("Meditation {0}".format(value))

    def blink_handler(headset, value):
      print ("Blink {0}".format(value))

    def eeg_power_handler(headset, delta, theta, lowalpha, highalpha, lowbeta, highbeta, lowgamma, midgamma):
      print ("Delta: {0}, Theta: {1}, LowAlpha: {2}, HighAlpha: {3}, LowBeta: {4}, HighBeta: {5}, LowGamma: {6}, MidGamma: {7}".format(delta,theta,lowalpha,highalpha,lowbeta,highbeta,lowgamma,midgamma))

    def raw_wave_handler(headset, val):
      print (val, " ")

    def checksum_mismatch_handler(headset, val, chksum):
      print ("CHKSUM MISMATCH: expected {0}, actual {1}".format(val,chksum))

    h = Headset('/dev/ttyUSB1','6F0A')
    h.attention_handlers.append(attention_handler)
    h.meditation_handlers.append(meditation_handler)
    h.blink_handlers.append(blink_handler)
    h.eeg_power_handlers.append(eeg_power_handler)
    h.raw_wave_handlers.append(raw_wave_handler)
    h.checksum_mismatch_handlers.append(checksum_mismatch_handler)

    raw_input ("Press enter to stop....")

