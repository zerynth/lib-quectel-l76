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
from quectel.nmea import nmea

SERIAL=0
I2C=1

class L76(nmea.NMEA_Receiver):
    """
.. class:: L76(ifc, mode=SERIAL, baud=9600,clock=400000, addr=0x00, reset=None, reset_on=0)

    Create an instance of the L76 class.

    :param ifc: serial interface to use (for example :samp:`SERIAL1`, :samp:`SERIAL2`, etc...)
    :param mode: one of SERIAL or I2C. Only SERIAL mode supported at the moment.
    :param baud: serial port baudrate
    :param clock: I2C clock frequency (not supported)
    :param addr: I2C address (not supported)
    :param reset: optional reset pin
    :param reset_on: reset pin active level

    Example: ::

        from quectel.l76 import l76

        ...

        gnss = l76.L76(SERIAL1)
        gnss.start()
        mpl.init()
        alt = mpl.get_alt()
        pres = mpl.get_pres()

    """

    def __init__(self,ifc,mode=SERIAL,baud=9600,clock=400000,addr=0x00,reset=None,reset_on=0):
        if mode!=SERIAL:
            raise UnsupportedError
        self.mode = mode
        self.ifc = ifc
        self.baud = baud
        self.running = False
        self.talking = False
        self.th = None
        self.rstpin = reset
        self.rstval = reset_on
        nmea.NMEA_Receiver.__init__(self)
        # exit reset
        if self.rstpin is not None:
            pinMode(self.rstpin,OUTPUT_PUSHPULL)
            digitalWrite(self.rstpin,self.rstval)
            digitalWrite(self.rstpin,self.rstval)
            sleep(100)
            digitalWrite(self.rstpin,HIGH^ self.rstval)
        sleep(2000) # boot time
        # put in lowest power consumption mode
        if self.mode==SERIAL:
            self.drv = streams.serial(self.ifc,baud=self.baud,set_default=False)
            self.drv.write("$PMTK161,0*28\r\n")
            sleep(100)
            self.drv.close()
        self.drv = None

    def start(self):
        """
.. method:: start()

        Start the L76 and the receiver thread.

        :returns: *True* if receiver thread has been started, *False* if already active.

        """
        if self.th:
            return False
        
        if self.rstpin is None:
            if self.mode==SERIAL:
                self.drv = streams.serial(self.ifc,baud=self.baud,set_default=False)
                self.drv.write("$PMTK101*32\r\n")
                sleep(100)
                self.drv.close()
            else:
                raise UnsupportedError

        self.enable(True)
        self.running = True
        self.talking = True
        self.th = thread(self._run)

        # restart receiver
        if self.rstpin is not None:
            digitalWrite(self.rstpin,self.rstval)
            sleep(100)
            digitalWrite(self.rstpin,HIGH^ self.rstval)
            sleep(2000) # boot time
        return True

    def stop(self):
        """
.. method:: stop()

        Stop the L76 by using the lowest power consumption mode and terminates the receiver thread.
        It can be restarted by calling :ref:`start`.

        :returns: *True* if receiver thread has been stopped, *False* if already inactive.

        """
        if not self.running:
            return False
        
        if self.mode==SERIAL:
            self.drv.write("$PMTK161,0*28\r\n")
        else:
            raise UnsupportedError
        self.running = False
        self.talking = False
        sleep(1000)
        self.enable(False)
        return True

    def pause(self):
        """
.. method:: pause()

        Pause the L76 by putting it into standby mode. It can be restarted by calling :ref:`resume`.
        Refer to the L76 documentation for details `here <https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Series_Hardware_Design_V3.1.pdf>`_

        """
        if not self.running:
            raise RuntimeError
        if self.mode==SERIAL:
            self.drv.write("$PMTK161,0*28\r\n")
        else:
            raise UnsupportedError
        sleep(1000)
        self.talking = False
        self.enable(False)

    def resume(self):
        """
.. method:: resume()

        Wake up the L76 from standby mode entered by calling :ref:`resume`.
        Refer to the L76 documentation for details `here <https://www.quectel.com/UploadImage/Downlad/Quectel_L76_Series_Hardware_Design_V3.1.pdf>`_

        """
        if not self.running:
            raise RuntimeError
        if self.mode==SERIAL:
            self.drv.write("$PMTK101*32\r\n")
        else:
            raise UnsupportedError
        self.enable(True)
        self.talking = True
        sleep(1000) # wakeup time

    def set_rate(self,rate=1000):
        """
.. method:: set_rate(rate=1000)

        Set the frequency for location fix (100-10000 milliseconds is the available range).

        """
        if not self.running:
            raise RuntimeError
        msg = "PMTK220,"+str(rate)
        crc=ord(msg[0])
        for i in range(1,len(msg)):
            crc = crc^ord(msg[i])
        msg="$"+msg+"*"+hex(crc,"")+"\r\n"
        if self.mode==SERIAL:
            self.drv.write(msg)
        else:
            raise UnsupportedError

    ##################### Private

    def _run(self):
        """
----
NMEA
----

    Additional methods from the base class :any:`nmea.NMEA_Receiver`.

.. method:: fix()

        Return the current fix or *None* if not available.
        A fix is a tuple with the following elements:

            * latitude in decimal format (-89.9999 - 89.9999)
            * longitude in decimal format (-179.9999 - 179.9999)
            * altitude in meters
            * speed in Km/h
            * course over ground as degrees from true north
            * number of satellites for this fix
            * horizontal dilution of precision (0.5 - 99.9)
            * vertical dilution of precision (0.5 - 99.9)
            * positional dilution of precision (0.5 - 99.9)
            * UTC time as a tuple (yyyy,MM,dd,hh,mm,ss,microseconds)

.. method:: has_fix()
    
        Return *True* if a fix is available

.. method:: utc()

        Return the current UTC time or *None* if not available.
        A UTC time is a tuple of (yyyy,MM,dd,hh,mm,ss,microseconds).

        UTC time can be wrong if no fix has ever been obtained.

.. method:: has_utc()
    
        Return *True* if a UTC time is available

        """
        buffer = bytearray(256)
        if self.mode==SERIAL:
            self.drv = streams.serial(self.ifc,baud=self.baud,set_default=False)

        while self.running:
            try:
                chs = nmea.readline(self.ifc,buffer)
                if self.debug:
                    self.print_d(buffer[:buffer.find(b'\0')])
                if not self.talking:
                    continue
                if chs>=1:
                    self.parse(buffer, chs)
                else:
                    self.print_d("L76 check",chs)
            except Exception as e:
                if self.talking:
                    self.print_d("L76 loop", e)

        if self.mode==SERIAL:
            self.drv.close()
        self.th = None
