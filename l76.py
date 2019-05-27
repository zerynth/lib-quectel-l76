"""
.. module:: l76

**********
L76 Module
**********

This module implements the Zerynth driver for the Quectel L76 GNSS chip (`Product page <https://www.quectel.com/product/l76.htm>`_).


The following functionalities are implemented:

    * retrieve the current location fix if present
    * retrieve the current UTC time

The driver starts a background thread continuously tracking the last available location fix. The frequency of fixes can be customized.
The driver support serial mode only.

Location fixes are obtained by parsing NMEA sentences of type RMC and GGA. Obtaining a fix or UTC time are thread safe operations.

   """

import streams
import threading


SERIAL=0
I2C=1


class L76():
    """

.. class:: L76(ifc, mode=SERIAL, baud=9600,clock=400000, addr=0x00)

    Creates an intance of L76.

    :param ifc: interface used. One of SERIAL0, SERIAL1, ...
    :param mode: one of SERIAL or I2C. Only SERIAL mode supported at the moment.
    :param baud: serial speed
    :param clock: I2C clock frequency (not supported)
    :param addr: I2C address (not supported)

    Example: ::

        from quectel.l76 import l76

        ...

        gnss = l76.L76(SERIAL1)
        gnss.start()
        mpl.init()
        alt = mpl.get_alt()
        pres = mpl.get_pres()

    """


    def __init__(self,ifc,mode=SERIAL,baud=9600,clock=400000,addr=0x00):
        self.mode = mode
        self.running=False
        self.ifc = ifc
        self.baud = baud
        self.th = None
        self._tm = [0,0,0,0,0,0,0]
        self._lat = 0
        self._lon = 0
        self._spd = 0
        self._cog = 0
        self._nsat = 0
        self._hdop = 0
        self._alt  = 0
        self._cfix = [None]*8
        self._time = None
        self._fix = None
        self._rmc = False
        self._gga = False
        self._utc = False
        self._lock = threading.Lock()


    def start(self,rstpin=None,rstval=0):
        """
.. method:: start(rstpin=None,rstval=0)

        Start the L76.

        :param rstpin: if given, uses :samp:`rstpin` as the reset pin of L76
        :param rstval: the value to move :samp:`rstpin` to

        """
        # init rst if needed
        if rstpin is not None:
            pinMode(rstpin,OUTPUT_PUSHPULL)
            digitalWrite(rstpin,rstval)
            sleep(100)

        if self.mode==SERIAL:
            self.drv = streams.serial(self.ifc,baud=self.baud,set_default=False)
        else:
            raise UnsupportedError
        if self.th:
            self.running=True
        else:
            self.th = thread(self._run)
        sleep(2000)


    def stop(self):
        """
.. method:: stop()

        Stop the L76 by putting it into backup mode. It can be restarted only by setting the FORCE_ON pin to high.
        Refer to the L76 documentation for details `here <https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Series_Hardware_Design_V3.1.pdf>`_

        """
        if self.mode==SERIAL:
            self.drv.write("$PMTK225,4*2F\r\n")
        else:
            raise UnsupportedError

    def pause(self):
        """
.. method:: pause()

        Stop the L76 by putting it into standby mode. It can be restarted by calling :ref:`resume`.
        Refer to the L76 documentation for details `here <https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Series_Hardware_Design_V3.1.pdf>`_

        """
        if self.mode==SERIAL:
            self.drv.write("$PMTK161,0*28\r\n")
        else:
            raise UnsupportedError

    def resume(self):
        """
.. method:: resume()

        Wake up the L76 from standby mode.
        Refer to the L76 documentation for details `here <https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Series_Hardware_Design_V3.1.pdf>`_

        """
        if self.mode==SERIAL:
            self.drv.write("$PMTK101*32\r\n")
        else:
            raise UnsupportedError


    def set_rate(self,rate=1000):
        """
.. method:: set_rate(rate=1000)

        Set the frequency for location fix (100-10000 milliseconds is the available range).

        """
        msg = "PMTK220,"+str(rate)
        crc=ord(msg[0])
        for i in range(1,len(msg)):
            crc = crc^ord(msg[i])
        msg="$"+msg+"*"+hex(crc,"")+"\r\n"
        if self.mode==SERIAL:
            self.drv.write(msg)
        else:
            raise UnsupportedError

    def fix(self):
        """
.. method:: fix()

        Return the current fix or None if no fix is available.
        A fix is a tuple with the following elements:

            * latitude in decimal format (-89.9999 - 89.9999)
            * longitude in decimal format (-179.9999 - 179.9999)
            * altitude in meters
            * speed in Km/h
            * course over ground as degrees from true north
            * horizontal dilution of precision (0.5 - 99.9)
            * number of satellites for this fix
            * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss,microseconds)

        """
        self._lock.acquire()
        r = self._fix
        self._fix = None
        self._lock.release()
        return r

    def has_fix(self):
        """
.. method:: has_fix()
    
        Return True if a fix is available

        """
        self._lock.acquire()
        r = self._fix
        self._lock.release()
        return r is not None

    def utc(self):
        """
.. method:: utc()

        Return the current UTC time or None if no UTC time is available.
        A UTC time is a tuple of (yyyy,MM,dd,hh,mm,ss,microseconds).

        UTC time can be wrong if no fix has ever been obtained.
        """
        r= None
        self._lock.acquire()
        if self._time:
            r = self._time
            self._time = None
        self._lock.release()
        return r

    def has_utc(self):
        """
.. method:: has_utc()
    
        Return True if a UTC time is available

        """
        self._lock.acquire()
        r = self._time
        self._lock.release()
        return r is not None


    ##################### Private

    def _set_time(self,tm,dt):
        self._tm[0]=2000+int(dt[4:])
        self._tm[1]=int(dt[2:4])
        self._tm[2]=int(dt[0:2])
        self._tm[3]=int(tm[0:2])
        self._tm[4]=int(tm[2:4])
        self._tm[5]=int(tm[4:6])
        self._tm[6]=int(tm[7:])
        self._cfix[-1]=self._tm
        self._utc=True

    def _set_pos(self,lat,latp,lon,lonp):
        self._lat = int(lat[0:2]) +int(lat[2:4])/60 + int(lat[6:])/3600
        self._lon = int(lon[0:3]) +int(lon[3:5])/60 + int(lon[7:])/3600
        if latp=="S":
            self._lat = -self._lat
        if lonp=="W":
            self._lon = -self._lon
        self._cfix[0]=self._lat
        self._cfix[1]=self._lon

    def _set_spd(self,spd):
        self._spd = float(spd)*1.852 # konts to km/h
        self._cfix[3] = self._spd

    def _set_cog(self,cog):
        self._cog = float(cog)
        self._cfix[4] = self._cog

    def _set_hdop(self,hdop):
        self._hdop = float(hdop)
        self._cfix[5] = self._hdop

    def _set_alt(self,alt):
        self._alt = float(alt)
        self._cfix[2] = self._alt

    def _set_nsat(self,nsat):
        self._nsat = int(nsat)
        self._cfix[6] = self._nsat

    def _parse(self,line):
        prefix = line[1:3]
        cmd = line[3:6]
        if cmd == "RMC":
            flds = line.split(",")
            if len(flds)<14:
                return
            tm = flds[1]
            dt = flds[9]
            self._set_time(tm,dt)
            dv = flds[2]
            if dv=="V":
                # no fix
                return
            self._set_pos(flds[3],flds[4],flds[5],flds[6])
            self._set_spd(flds[7])
            self._set_cog(flds[8])
            self._rmc = True
        elif cmd == "GGA":
            flds = line.split(",")
            if len(flds)<14:
                return
            if flds[6]=="0":
                # no fix
                return
            self._set_hdop(flds[8])
            self._set_nsat(flds[7])
            self._set_alt(flds[9])
            self._gga = True
        else:
            pass
        if self._utc:
            # we have time
            self._lock.acquire()
            self._time = (self._tm[0],self._tm[1],self._tm[2],self._tm[3],self._tm[4],self._tm[5],self._tm[6])
            self._lock.release()
        if self._rmc and self._gga:
            # we have a fix
            self._lock.acquire()
            self._fix = (
                    self._cfix[0],
                    self._cfix[1],
                    self._cfix[2],
                    self._cfix[3],
                    self._cfix[4],
                    self._cfix[5],
                    self._cfix[6],
                    (self._tm[0],self._tm[1],self._tm[2],self._tm[3],self._tm[4],self._tm[5],self._tm[6])
                    )
            self._rmc = False
            self._gga = False
            self._utc = False
            self._lock.release()


    def _run(self):
        buffer=bytearray(256)
        self.running=True
        while True:
            if self.running:
                line = self.drv.readline(buffer=buffer,size=256)
                # print(">",len(line),line)
                self._parse(line)
            else:
                sleep(5000)

