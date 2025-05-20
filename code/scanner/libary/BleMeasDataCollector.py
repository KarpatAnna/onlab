import sys
import time
import serial
import logging
import logging.handlers
import datetime
import binascii
import traceback
import uuid

# Import helper functions
from libary.util import *
import logging
import datetime
import traceback
import time
from threading import Thread

# Main class for Bosch demo application
class BleMeasDataCollector(BleDataCollector):
    pressure_char_version = 2 # Can either be 1 for lower clipping at 100 kPa or 2 for lower clipping at 90 kPa.
    def __init__(self, serial_port, logger, data_logger, log_severity_level='error'):
        self.known_devices = {}
        self.connectable_devices = {}
        self.logger = logger
        self.data_logger = data_logger
        super(BleMeasDataCollector, self).__init__(serial_port, log_severity_level)

    def on_gap_evt_adv_report(self, ble_driver, conn_handle, peer_addr, rssi, adv_type, adv_data):
        # Type of advertising message
        adv_type_name = BleMeasDataCollector.ADV_TYPE_NAMES[adv_type]

        #print(adv_data.records)

        address_string = ':'.join('{0:02X}'.format(b) for b in peer_addr.addr)

        man_spec_data = adv_data.records[nrf_ble_driver.BLEAdvData.Types.manufacturer_specific_data]
        #dongle_manufacturer_data = 0x15

        #if dongle_manufacturer_data in man_spec_data:
        #print("found it")
        #self.data_logger.info(man_spec_data) 
        if address_string not in self.known_devices:
            self.known_devices[address_string] = peer_addr
        
        #print(man_spec_data)
        if address_string == "DC:98:88:B3:60:C0":
            log_string = 'Received {}, address: {}, RSSI: {} dBm\r\n'.format(
                        adv_type_name, address_string, rssi
                    )
            self.data_logger.info(log_string)


            company_identifier          = bytes(man_spec_data[ 0: 2])
            device_type_byte            = bytes(man_spec_data[ 2: 3])
            adv_data_length             = int.from_bytes(man_spec_data[ 3: 4], 'big', signed=True)
            beacon_uuid                 = bytes(man_spec_data[ 4:20])
            major_value                 = int.from_bytes(man_spec_data[20:22], 'big', signed=True)
            minor_value                 = int.from_bytes(man_spec_data[22:24], 'big', signed=True)
            measured_rssi               = int.from_bytes(man_spec_data[24:25], 'big', signed=True)
            adv_data_info               = bytes(man_spec_data[25:26])

            if company_identifier.hex() == '5900':
                company = 'Nordic Semiconductor ASA <0x0059>'
            else:
                company = company_identifier.hex()
            if device_type_byte.hex() == '02':
                device_type = 'Beacon <0x02>'
            else: 
                device_type = device_type_byte.hex()
            beacon_uuid_bytes = uuid.UUID(bytes=beacon_uuid)
            log_string = 'Company: {}, Device type: {}, Data Length: {} bytes, UUID: {}, Major Value: {}, Minor Value: {}, RSSI at 1m: {} dBm, Advertised Data: {}\r\n'.format(
                        company, device_type, adv_data_length, beacon_uuid_bytes, major_value, minor_value, measured_rssi, adv_data_info.hex().upper()
                    )
            self.data_logger.info(log_string)
    