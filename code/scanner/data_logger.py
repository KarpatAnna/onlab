import sys
import threading
import time
import serial
import logging
import logging.handlers
import datetime
import binascii
import traceback

# Import helper functions
from libary.BleMeasDataCollector import *
from libary.util import *
import logging
import datetime
import traceback
import time
from threading import Thread
# Configure UUIDs
BoschDemo_svc        = uuid_base_from_16bytes(0x02a63290_1a00_b83e_af18_025703723367)
BoschDemo_TxPwr_char = uuid_base_from_16bytes(0x02a63290_1a01_b83e_af18_025703723367)
BoschDemo_Ctr_char   = uuid_base_from_16bytes(0x02a63290_1a02_b83e_af18_025703723367)
BoschDemo_Gpio_char  = uuid_base_from_16bytes(0x02a63290_1a03_b83e_af18_025703723367)

class DataLogger():
    # Configure Data Logger
    logprefix = "Test_"
    logformat = 'csv'
    log_level = logging.ERROR
    collector = None
    pressure_char_version = 2 # Can either be 1 for lower clipping at 100 kPa or 2 for lower clipping at 90 kPa.
    logger = logging.getLogger(__name__)
    loggerThread = Thread()
    _sentinel = threading.Event()
    def __init__(self, logLevel, logger = None) -> None:
        if logLevel is not None:
            self.log_level = logLevel
        self.logger = logger
        if self.logger is None:
            self._CreateGeneralLogger()
        self.logger.info("DataLogger class created")

    def GetDate(self):
        today = datetime.datetime.now()
        return today.strftime(r'%Y-%m-%d_%H_%M_%S')

    def _CreateGeneralLogger(self):
        # General log
        # Prepare logging
        logging.basicConfig(
            level = self.log_level,
            handlers = [],
        )
        human_readable_formatter = logging.Formatter(fmt='%(asctime)s [%(thread)d/%(threadName)s] %(message)s',)
        log_file_handler = logging.FileHandler(self.GetDate()+'.log', mode='w')
        log_file_handler.setFormatter(human_readable_formatter)
        self.logger = logging.getLogger(__name__)
        self.logger.addHandler(log_file_handler)

    def CreateDataLogger(self,DataName = None):
        self.logger.info("Creation of new data logger started")
        if DataName is None:
            DataName = self.logprefix + self.GetDate()
        self.data_logger = self.logger.getChild('raw_data')
        self.datalogfilen = DataName + '_raw.' + self.logformat
        self.logger.info(f"Data will be saved to: {self.datalogfilen}")
        if self.logformat == 'csv':
            with open(self.datalogfilen, 'w') as dfileh:
                dfileh.write('date_time,address,RSSI\n')
            data_formatter = logging.Formatter(fmt='%(asctime)s,%(message)s')
        elif self.logformat == 'json':
            with open(self.datalogfilen, 'w') as dfileh:
                dfileh.write('[\n')
            data_formatter = logging.Formatter(fmt='  ["%(asctime)s", %(message)s],')
        else:
            self.logger.error(f"Unknown log format was given: {self.logformat}")
            raise ValueError('Unknown log format: expecting either "csv" or "json"!')
        
        data_formatter.default_time_format='%Y-%m-%dT%H:%M:%S'
        dtnow = datetime.datetime.now()
        tzoffset = dtnow.astimezone().tzinfo.utcoffset(dtnow).seconds
        tzoffset_h = int(tzoffset / 60 / 60)
        tzoffset_m = int(abs(tzoffset) / 60) % 60
        data_formatter.default_msec_format='%s.%03d'+'{:+03d}:{:02d}'.format(tzoffset_h, tzoffset_m)

        self.data_file_handler = logging.FileHandler(self.datalogfilen, mode='a')
        self.data_file_handler.setFormatter(data_formatter)
        self.logger.info("Data file handler created")
        self.data_logger.addHandler(self.data_file_handler)
        self.data_logger.setLevel(logging.DEBUG)
        self.logger.info("Data logger initialized")
        return self.datalogfilen, self.data_file_handler
        
    def close_data_file_logger_handle(self):
        # Close log file
        self.logger.info("Closing data logger file and clearing logger")
        self.data_file_handler.flush()
        self.data_file_handler.close()
        self.data_logger.handlers.clear()
        self.data_logger = None
        self.logger.info("Data logger file closed, logger destroyed")
        
        # Add closing bracket only for JSON file
        if self.logformat == 'json':
            with open(self.datalogfilen, 'r+') as dfileh:
                dfileh.seek(0, 2)
                file_len = dfileh.tell()
                dfileh.seek(file_len-3)
                dfileh.truncate()
                dfileh.write('\n]')

    def InitDongle(self):
        # Prepare nRF dongle
        _, nrf_ds = get_available_nrf_dongles()    
        if len(nrf_ds) == 1:
            serial_port = nrf_ds[0]
        else:
            self.logger.error("More than 1 dongle detected, unhandled")
            raise ValueError("More than 1 dongle detected!")
        self.logger.warning('Serial port used: {}'.format(serial_port))
        self.collector = BleMeasDataCollector(serial_port,self.logger, self.data_logger)
        self.collector.open()
        self.logger.info("Collector class created")
        return self.collector

    def ContinuousLogging(self):
        reconnect = True
        while not self._sentinel.is_set():
            try:
                if reconnect:
                    try:
                        self.InitDongle()
                        reconnect = False
                        time.sleep(1)
                    except ValueError:
                        # Reconnection failure
                        traceback.print_exc()
                        self.logger.error('Re-connecting to dongle failed! (ValueError)')
                if not reconnect:
                    self.collector.scan(scan_duration=1, interval_ms=200, window_ms=180)
                    self.logger.info("Dongle scanning")
            except nrf_exceptions.NordicSemiException as err:
                if err.error_code == nrf_ble_driver.driver.NRF_ERROR_SD_RPC_NO_RESPONSE:
                    # Connection initially lost
                    reconnect = True
                    self.collector.close()
                    self.logger.error('"NRF_ERROR_SD_RPC_NO_RESPONSE": Lost connection to dongle!')
                    time.sleep(1)
                elif err.error_code == nrf_ble_driver.driver.NRF_ERROR_INVALID_STATE:
                    # Connection initially lost
                    reconnect = True
                    self.collector.close()
                    self.logger.error('"NRF_ERROR_INVALID_STATE": Lost connection to dongle!')     
                    time.sleep(1)
                elif err.error_code == nrf_ble_driver.driver.NRF_ERROR_SD_RPC_H5_TRANSPORT_STATE:
                    # Reconnection failure
                    self.logger.error('Re-connecting to dongle failed!')
                    reconnect = True
                    self.collector.close()
                    self.logger.error('"NRF_ERROR_INVALID_STATE": Lost connection to dongle!')
                    time.sleep(1)
                else:
                    traceback.print_exc()
                    raise err
            time.sleep(1)
        self._ClearUp()

    def _ClearUp(self):
        self.logger.info("BLE clear up called")
        self.collector.close()   
        self.collector = []
        self.logger.info("Collector class destroyed")
        self.close_data_file_logger_handle()

    def StopBleLogger(self):
        self.logger.info("BLE logger will be stopped")
        if not self._sentinel.is_set():
            self.logger.info("Sentinel signal sent out to logger thread")
            self._sentinel.set()
        self.logger.info("Waiting logger thread to be terminated")
        try:
            self.loggerThread.join()
        except: pass
        self.logger.info("Logger thread terminated")

    def StartLogging(self):
        if self.loggerThread.is_alive():
            self.logger.warning("Logger already ongoing, no new instance created!")
            return
        self._sentinel.clear()
        self.logger.info("Create new logger thread")
        self.loggerThread = Thread(target=self.ContinuousLogging, args=())
        self.logger.info("Start new logger thread")
        self.loggerThread.start()

def StartBleLogger(logLevel = None, fileName = None):
    logger = DataLogger(logLevel)
    logger.CreateDataLogger(fileName)
    #logger.InitDongle()
    logger.StartLogging()
    return logger

# Main code
if __name__ == '__main__':    
    StartBleLogger()
