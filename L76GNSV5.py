# quectel L76 GNSS library for Micropython
# pycom pytrack module for wipy, lopy, sipy ...
# Andre Peeters
# andre@andrethemac.be
# v4 2018-03-24
# v6 converted to threading model to read messages
# v7 fix for longer messages on newer L76 chips (v4.10)
# based upon the original L76GLNSS library
# and the modifications by neuromystix
# every lookup of coordinates or other GPS data has to wait for the
# right GPS message, no caching of GPS data
# MIT licence

from machine import Timer, RTC
import time
import gc
import binascii
import _thread

# TODO: annotate sattelites in view


class L76GNSS:

    GPS_I2CADDR = const(0x10)
    NMEA_MESSAGES = ('RMC','VTG','GGA','GSA','GSV','GLL')

    def __init__(self, pytrack=None, sda='P22', scl='P21', timeout=180, clock_sync=True, debug=False):
        if pytrack is not None:
            self.i2c = pytrack.i2c
        else:
            from machine import I2C
            self.i2c = I2C(0, mode=I2C.MASTER, pins=(sda, scl))

        self.chrono = Timer.Chrono()
        self.timeout = timeout
        self.timeout_status = True
        self.reg = bytearray(1)
        self.i2c.writeto(GPS_I2CADDR, self.reg)
        self.fix = False
        self.debug = debug
        self.timeLastFix = 0
        self.ttf = -1
        self.lastmessage = 0
        self.RMC = None
        self.VTG = None
        self.GGA = None
        self.GSA = None
        self.GSV = None
        self.GLL = None
        self.Latitude = None
        self.Longitude = None
        self.thread = None
        self.kill_thread = False
        self.thread_running = False
        self._start_thread(self)
        self.clock_sync = clock_sync
        if self.debug:
            print("self.thread",self.thread)

    def _start_thread(self, debug=False):
        self.thread = _thread.start_new_thread(self._reading_messages, (self.debug, ) )

    def _kill_thread(self, debug=False):
        if debug:
            print("kill_thread is True")
        self.kill_thread = True

    def _get_thread_status(self, debug=False):
        print("thread running is",self.thread_running)

    def _read(self):
        """read the data stream form the gps"""
        # Changed from 64 to 255 - I2C L76 says it can read till 255 bytes
        reg = b''
        try:
            reg = self.i2c.readfrom(GPS_I2CADDR, 255)
        except:
            pass
        return reg

    @staticmethod
    def _convert_coord(coord, orientation):
        """convert a ddmm.mmmm to dd.dddddd degrees"""
        coord = (float(coord) // 100) + ((float(coord) % 100) / 60)
        if orientation == 'S' or orientation == 'W':
            coord *= -1
        return coord

    def time_fixed(self):
        """how long till the last fix"""
        return int(time.ticks_ms()/1000) - self.timeLastFix

    def _mixhash(self, keywords, sentence):
        """return hash with keywords filled with sentence"""
        ret = {}
        while len(keywords) - len(sentence) > 0:
            sentence += ('',)
        if len(keywords) == len(sentence):
            # for k, s in zip(keywords, sentence):
            #     ret[k] = s
            ret = dict(zip(keywords, sentence))
            try:
                ret['Latitude'] = self._convert_coord(ret['Latitude'], ret['NS'])
            except:
                pass
            try:
                ret['Longitude'] = self._convert_coord(ret['Longitude'], ret['EW'])
            except:
                pass
            return ret
        else:
            return None

    def _GGA(self, sentence):
        """essentials fix and accuracy data"""
        keywords = ['NMEA', 'UTCTime', 'Latitude', 'NS', 'Longitude', 'EW',
                    'FixStatus', 'NumberOfSV', 'HDOP',
                    'Altitude', 'M', 'GeoIDSeparation', 'M', 'DGPSAge', 'DGPSStationID']
        self.GGA = self._mixhash(keywords, sentence)
        self.GGA['lastmessage'] = time.gmtime()
        try:
            if int(self.GGA['FixStatus']) >= 1 :
                self.Latitude = self.GGA['Latitude']
                self.Longitude = self.GGA['Longitude']
        except:
            pass
        return self.GGA

    def _GLL(self, sentence):
        """GLL sentence (geolocation)"""
        keywords = ['NMEA', 'Latitude', 'NS', 'Longitude', 'EW',
                    'UTCTime', 'dataValid', 'PositioningMode']
        self.GLL = self._mixhash(keywords, sentence)
        self.GLL['lastmessage'] = time.gmtime()
        try:
            if self.GLL['PositioningMode'] != 'N':
                self.Latitude = self.GLL['Latitude']
                self.Longitude = self.GLL['Longitude']
        except:
            pass
        return self.GLL

    def _RMC(self, sentence):
        """required minimum position data"""
        if len(sentence) == 11:
            sentence.append('N')
        keywords = ['NMEA', 'UTCTime', 'dataValid', 'Latitude', 'NS', 'Longitude', 'EW',
                    'Speed', 'COG', 'Date', '', '', 'PositioningMode']
        if len(sentence) > len(keywords):
            keywords.append('NavigationaalStatus')
        self.RMC = self._mixhash(keywords, sentence)
        self.RMC['lastmessage'] = time.gmtime()
        try:
            if self.RMC['PositioningMode'] != 'N':
                self.Latitude = self.RMC['Latitude']
                self.Longitude = self.RMC['Longitude']
        except:
            pass
        if self.clock_sync:
            self.setRTCClock()
        return self.RMC

    def _VTG(self, sentence):
        """track and ground speed"""
        keywords = ['NMEA', 'COG-T', 'T', 'COG-M', 'M', 'SpeedKnots', 'N', 'SpeedKm', 'K',
                    'PositioningMode']
        self.VTG = self._mixhash(keywords, sentence)
        self.VTG['lastmessage'] = time.gmtime()
        # try:
        #     self.fix = self.VTG['PositioningMode'] != 'N'
        # except:
        #     print(self.VTG)
        #     print(sentence)
        #     self.VTG = None
        return self.VTG

    def _GSA(self, sentence):
        """fix state, the sattelites used and DOP info"""
        keywords = ['NMEA', 'Mode', 'FixStatus',
                    'SatelliteUsed01', 'SatelliteUsed02', 'SatelliteUsed03',
                    'SatelliteUsed04', 'SatelliteUsed05', 'SatelliteUsed06',
                    'SatelliteUsed07', 'SatelliteUsed08', 'SatelliteUsed09',
                    'SatelliteUsed10', 'SatelliteUsed11', 'SatelliteUsed12',
                    'PDOP', 'HDOP', 'VDOP']
        if len(sentence) > len(keywords):
            keywords.append('GNSSSystemID')
        self.GSA = self._mixhash(keywords, sentence)
        self.GSA['lastmessage'] = time.gmtime()
        # try:
        #     self.fix = int(self.GSA['FixStatus']) >= 1
        # except:
        #     print(self.GSA)
        #     print(sentence)
        #     self.GSA = None
        return self.GSA

    def _GSV(self, sentence):
        """four of the sattelites seen"""
        keywords = ['NMEA', 'NofMessage', 'SequenceNr', 'SatellitesInView',
                    'SatelliteID1', 'Elevation1', 'Azimuth1', 'SNR1',
                    'SatelliteID2', 'Elevation2', 'Azimuth2', 'SNR2',
                    'SatelliteID3', 'Elevation3', 'Azimuth3', 'SNR3',
                    'SatelliteID4', 'Elevation4', 'Azimuth4', 'SNR4']
        if len(sentence) > len(keywords):
            keywords.append('SignalID')
        self.GSV = self._mixhash(keywords, sentence)
        self.GSV['lastmessage'] = time.gmtime()
        return self.GSV

    def _pmtkAck(self, sentence):
        keywords = ['PMTK', 'command', 'response']
        return self._mixhash(keywords, sentence)
        # if sentence[2] == 3:
        #     return True
        # else:
        #     return False

    def _decodeNMEA(self, nmea, debug=False):
        """turns a message into a hash"""
        nmea_sentence = nmea[:-3].split(',')
        sentence = nmea_sentence[0][3:]
        nmea_sentence[0] = sentence
        if debug:
            print(sentence, "->", nmea_sentence)
        if sentence == 'RMC':
            return self._RMC(nmea_sentence)
        if sentence == 'VTG':
            return self._VTG(nmea_sentence)
        if sentence == 'GGA':
            return self._GGA(nmea_sentence)
        if sentence == 'GSA':
            return self._GSA(nmea_sentence)
        if sentence == 'GSV':
            return self._GSV(nmea_sentence)
        if sentence == 'GLL':
            return self._GLL(nmea_sentence)
        if sentence == '001':
            return self._pmtkAck(nmea_sentence)
        return None

    def _read_message(self, messagetype='GLL', debug=False):
        if type(messagetype) == type(()):
            mt = []
            for m in messagetype:
                mt += [m[-3:]]
            messagetype = mt
        else:
            messagetype = messagetype[-3:]
        if debug:
            print("messagetype", messagetype)
        if message == 'RMC':
            return self.RMC
        if message == 'VTG':
            return self.VTG
        if message == 'GGA':
            return self.GGA
        if message == 'GSA':
            return self.GSA
        if message == 'GSV':
            return self.GSV
        if message == 'GLL':
            return self.GLL

    def _reading_messages(self, debug=False):
        while True:
            self.thread_running = True
            self.lastmessage = time.gmtime()
            debug = False
            nmea_raw = self._read_message_raw(debug=debug)
            self._decodeNMEA(nmea=nmea_raw, debug=debug)
            if self.kill_thread:
                self.thread_running = False
                if debug:
                    print("kill thread")
                _thread.exit()

    def _read_message_raw(self, debug=False):
        """reads output from the GPS and translates it to a message"""
        nmea = b''
        start = nmea.find(b'$')
        while start < 0:
            nmea += self._read() #.strip(b'\r\n')
            start = nmea.find(b'$')
        if debug:
            print("nmea raw", len(nmea), start, nmea)
        nmea = nmea[start:]
        # end = nmea.find(b'*')
        end = nmea.find(b'\r\n')
        while end < 0:
            nmea += self._read() #.strip(b'\r\n')
            # end = nmea.find(b'*')
            end = nmea.find(b'\r\n')
        nmea = nmea[:end+3].decode('utf-8')
        nmea = nmea[:-3]
        nmea = nmea.replace('\n','')
        if debug:
            if nmea is not None:
                print("nmea raw fix", self.fix, len(nmea), nmea)
        gc.collect()
        return nmea

    def fixed(self,debug=False):
        """fixed yet? returns true or false"""
        return self.fix

    def get_fix(self, force=True, debug=False, timeout=None):
        """look for a fix, use force to refix, returns true or false"""
        if force:
            self.fix = False
        if timeout is None:
            timeout = self.timeout
        self.chrono.reset()
        self.chrono.start()
        chrono_running = True
        while chrono_running and self.fix is False:
            GGA = GLL = RMC = VTG = GSA = 0
            try:
                RMC = (self.RMC['PositioningMode'] != 'N' ) * 1
                # VTG = (self.VTG['PositioningMode'] != 'N' ) * 2
                GGA = (int(self.GGA['FixStatus']) >= 1 ) * 4
                # GSA = (int(self.GSA['FixStatus']) >= 1 ) * 8
                # GLL = (self.GLL['PositioningMode'] != 'N' ) * 16
                # if (RMC+VTG+GGA+GSA+GLL) == 31:
                if (RMC+GGA) == 5:
                    self.fix = True
            except:
                self.fix = False
            if debug:
                print('fix',self.fix)
                print('RMC',self.RMC is None)
                print('VTG',self.VTG is None)
                print('GGA',self.GGA is None)
                print('GSA',self.GSA is None)
                print('GLL',self.GLL is None)
                print('-'*80)
            if self.chrono.read() > timeout:
                chrono_running = False
        self.chrono.stop()
        self.ttf = -1
        if self.fix:
            self.ttf = self.chrono.read()
        if debug:
            print("fix in", self.chrono.read(), "seconds")
        return self.fix

    def gps_message(self, messagetype=None, debug=False):
        """returns the last message from the L76 gps"""
        return self._read_message(messagetype=messagetype, debug=debug)

    def coordinates(self, debug=False):
        """you are here"""
        if not self.fix:
            self.get_fix(debug=debug)
        age = time.time() - time.mktime(self.lastmessage)
        return dict(latitude=self.Latitude, longitude=self.Longitude, ttf=self.ttf, age=age)

    def get_speed_RMC(self):
        """returns your speed and direction as return by the ..RMC message"""
        if self.RMC is not None:
            return dict(speed=self.RMC['Speed'], COG=self.RMC['COG'])
        else:
            return None

    def get_speed(self):
        """returns your speed and direction in degrees"""
        if self.VTG is not None:
            return dict(speed=self.VTG['SpeedkM'], COG=self.VTG['COG-T'])
        else:
            return None

    def get_location(self, MSL=False,debug=False):
        """location, altitude and HDOP"""
        latitude, longitude, HDOP, altitude = None, None, None, None
        if not self.fix:
            self.get_fix(debug=debug)
        if self.GGA is not None:
            latitude = self.GGA['Latitude']
            longitude = self.GGA['Longitude']
            HDOP = self.GGA['HDOP']
            if MSL:
                altitude = self.GGA['GeoIDSeparation']
            else:
                altitude = self.GGA['Altitude']
            age = time.time() - time.mktime(self.GGA['lastmessage'])
            return dict(latitude=latitude, longitude=longitude, HDOP=HDOP, altitude=altitude, ttf=self.ttf, age=age)
        else:
            return None

    def getUTCTime(self, debug=False):
        """return UTC time or None when nothing if found"""
        utc_time = None
        for msg in (self.GLL, self.RMC, self.GGA):
            try:
                utc_time = msg['UTCTime']
                utc_time = "{}:{}:{}".format(utc_time[0:2], utc_time[2:4], utc_time[4:6])
            except:
                pass
        return utc_time

    def getUTCDateTime(self, debug=False):
        """return UTC date time or None when nothing if found"""
        if self.RMC is not None:
            utc_time = self.RMC['UTCTime']
            utc_date = self.RMC['Date']
            if str(utc_date)[-2:] == '80':
                return None
            return "20{}-{}-{}T{}:{}:{}+00:00".format(utc_date[4:6], utc_date[2:4], utc_date[0:2],
                                                      utc_time[0:2], utc_time[2:4], utc_time[4:6])
        else:
            return None

    def getUTCDateTimeTuple(self, debug=False):
        """return UTC date time or None when nothing if found"""
        if self.RMC is not None:
            utc_time = self.RMC['UTCTime']
            utc_date = self.RMC['Date']
            if str(utc_date)[-2:] == '80':
                return None
            year = '20'
            year += utc_date[4:6]
            if debug:
                print("date",utc_date,"time",utc_time)
            return (int(year), int(utc_date[2:4]), int(utc_date[0:2]), int(utc_time[0:2]), int(utc_time[2:4]), int(utc_time[4:6]))
        else:
            return None

    def setRTCClock(self,debug=False):
        r = RTC()
        rr = self.getUTCDateTimeTuple(debug=debug)
        if rr:
            r.init(rr)

    def enterStandBy(self, debug=False):
        """ standby mode, needs powercycle to restart"""
        message = bytearray('$PMTK161,0*28\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)

    def hotStart(self, debug=False):
        """ HotStart the receiver, using data in nv store"""
        message = bytearray('$PMTK101*32\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def warmStart(self, debug=False):
        """ warmStart the receiver, not using data in nv store, using last know messages"""
        message = bytearray('$PMTK102*31\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def coldStart(self, debug=False):
        """ coldStart the receiver, not using any data """
        message = bytearray('$PMTK103*30\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def fullColdStart(self, debug=False):
        """ full cold start the receiver, as cold start as in powercycle"""
        message = bytearray('$PMTK104*37\r\n')
        self.i2c.writeto(GPS_I2CADDR, message)
        self.fix = False
        # return self._read_message(messagetype='001', debug=debug)

    def setPeriodicMode(self, mode=0,
                        runtime=1000, sleeptime=1000,
                        secruntime=10000, secsleeptime=10000, debug=False):
        """
        mode :
            0 : fully on
            1 : periodic backup
            2 : periodic standby
            4 : perpetual standy by => needs powercycle to start the gps
            8 : allways locate standby
            9 : allways locate backup
        runtime: time the unit is fully operational
        sleeptime: time the unit is in standy/backup modus
        secruntime: time the unit is fully operation if the first runtime doesn't get a fix
        secsleeptime: time the unit is in standy/backup modus if the first runtime doesn't get a fix
        """
        if mode in (0, 1, 2, 8, 9):
            message = 'PMTK225,{},{},{},{},{}'.format(mode,runtime, sleeptime, secruntime, secsleeptime)
            checksum = self._get_checksum(message)
            message = bytearray('${}*{}\r\n'.format(message, checksum))
            if debug:
                print("setPeriodicMode",message)
            self.i2c.writeto(GPS_I2CADDR, message)
        # return self._read_message(messagetype='001', debug=debug)

    def setAlwaysOn(self, debug=False):
        self.setPeriodicMode(mode=0)

    def setAlwaysLocateMode(self, mode=8, debug=False):
        if mode in (8, 9):
            message = 'PMTK225,{}'.format(mode)
            checksum = self._get_checksum(message)
            message = bytearray('${}*{}\r\n'.format(message, checksum))
            if debug:
                print("setAlwaysLocateMode",message)
            self.i2c.writeto(GPS_I2CADDR, message)
            return True
            # response = self._read_message(messagetype='001', debug=debug)
            # if debug:
            #     print("response",response)
            # if response['response'] == 3:
            #     return True
            # else:
            #     return False

    def _get_checksum(self, message):
        """calculates the checksum"""
        mc = ord(message[0])
        for m in message[1:]:
            mc = mc ^ ord(m)
        return '{:x}'.format(mc).upper()

    def _check_checksum(self, message):
        """check the checksum of the message"""
        message = message[1:]
        message, checksum = message.split('*')
        return self._get_checksum(message) == checksum
