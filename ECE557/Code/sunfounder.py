from PCA9685 import PWM


class Servo(object):
    '''Servo driver class'''
    _MIN_PULSE_WIDTH = 500
    _MAX_PULSE_WIDTH = 2500
    _DEFAULT_PULSE_WIDTH = 1500
    _FREQUENCY = 60

    _DEBUG = False
    _DEBUG_INFO = 'DEBUG "Servo.py":'

    def __init__(self, channel, offset=135, direction=1, lock=True, bus_number=None, address=0x40):
        ''' Init a servo on specific channel, this offset '''
        if channel < 0 or channel > 15:
            raise ValueError("Servo channel \"{0}\" is not in (0, 15).".format(channel))
        self._debug_("Debug on")
        self.channel = channel
        self.offset = offset
        self.lock = lock
        self.direction = direction

        self.pwm = PWM(bus_number=bus_number, address=address)
        self.frequency = self._FREQUENCY
        self.write(0)
        print(f'Servo {channel} moved to default')

    
    def _debug_(self,message):
        if self._DEBUG:
            print(self._DEBUG_INFO,message)
            

    def setup(self):
        self.pwm.setup()
        

    def _angle_to_analog(self, angle):
        ''' Calculate 12-bit analog value from giving angle '''
        pulse_wide   = self.pwm.map(angle, 0, 270, self._MIN_PULSE_WIDTH, self._MAX_PULSE_WIDTH)
        analog_value = int(float(pulse_wide) / 1000000 * self.frequency * 4096)
        self._debug_('Angle %d equals Analog_value %d' % (angle, analog_value))
        return analog_value

    @property
    def frequency(self):
        return self._frequency

    @frequency.setter
    def frequency(self, value):
        self._frequency = value
        self.pwm.frequency = value

    @property
    def offset(self):
        return self._offset

    @offset.setter
    def offset(self, value):
        ''' Set offset for much user-friendly '''
        self._offset = value
        self._debug_('Set offset to %d' % self.offset)

    def write(self, angle):
        ''' Turn the servo with giving angle. '''
        angle = self.offset + angle * self.direction
        if self.lock:
            if angle > 270:
                angle = 270
            if angle < 0:
                angle = 0
        else:
            if angle < 0 or angle > 270:
                raise ValueError("Servo \"{0}\" turn angle \"{1}\" is not in (0, 270).".format(self.channel, angle))
        val = self._angle_to_analog(angle)
        self.pwm.write(self.channel, 0, val)
        self._debug_('Turn angle = %d' % angle)

    @property
    def debug(self):
        return self._DEBUG

    @debug.setter
    def debug(self, debug):
        ''' Set if debug information shows '''
        if debug in (True, False):
            self._DEBUG = debug
        else:
            raise ValueError('debug must be "True" (Set debug on) or "False" (Set debug off), not "{0}"'.format(debug))

        if self._DEBUG:
            print(self._DEBUG_INFO, "Set debug on")
        else:
            print(self._DEBUG_INFO, "Set debug off")


def install(offset_list, direction_list = []):
    all_servos = [0] * 16
    for i in range(16):
        all_servos[i] = Servo(i, offset=offset_list[i], direction=direction_list[i])
    for i in range(16):
        all_servos[i].setup()

    print('All initiated!')
    return all_servos