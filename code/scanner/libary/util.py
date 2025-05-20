import serial
import pickle
import serial.tools.list_ports
import logging
import binascii
from queue import Queue, Empty

import pc_ble_driver_py.config as ble_config
ble_config.__conn_ic_id__ = 'NRF52'

import pc_ble_driver_py.ble_driver as nrf_ble_driver
import pc_ble_driver_py.ble_adapter as nrf_ble_adapter
import pc_ble_driver_py.exceptions as nrf_exceptions

# Helper for custom 128bit UUIDs
def uuid_base_from_16bytes(uuid_val, uuid_type = nrf_ble_driver.driver.BLE_UUID_TYPE_VENDOR_BEGIN):
    base_uuid = uuid_val.to_bytes(16, byteorder='big')
    base = nrf_ble_driver.BLEUUIDBase(list(base_uuid), uuid_type)
    return nrf_ble_driver.BLEUUID(int.from_bytes(base_uuid[2:4], 'big'), base)

def uuid128_to_str(uuid_val):
    uuid_string = ''.join('{0:02X}'.format(b) for b in uuid_val)
    return uuid_string[:8] + '-' + uuid_string[8:12] + '-' + uuid_string[12:16] + '-' + uuid_string[16:20] + '-' + uuid_string[20:]

def uuid_to_str(uuid):
    if type(uuid.value) == int:
        uuid_value = uuid.value
    else:
        uuid_value = uuid.value.value
    return '0x'+binascii.hexlify(uuid_value.to_bytes(2, byteorder='big')).decode('utf-8') if uuid.base.base is None else uuid128_to_str(uuid.base.base)

# (De-)Serialization of service and characteristic handles
def serializeServices(services):
    def serializeUuid(uuid):
        if type(uuid.value) == int:
            return { 'value': uuid.value, 'base': (uuid.base.base, uuid.base.type) }
        else:
            return { 'value': uuid.value.value, 'base': (uuid.base.base, uuid.base.type) }

    def serializeDescriptors(descs):
        return [{'handle': d.handle, 'uuid': serializeUuid(d.uuid)} for d in descs]

    def serializeChars(chars):
        serializeProps = lambda props: { 'broadcast': props.broadcast, 'read': props.read, 'write_wo_resp': props.write_wo_resp, 'write': props.write, 'notify': props.notify, 'indicate': props.indicate, 'auth_signed_wr': props.auth_signed_wr }
        return [{'uuid': serializeUuid(c.uuid), 'char_props': serializeProps(c.char_props), 'handle_decl': c.handle_decl, 'handle_value': c.handle_value, 'end_handle': c.end_handle, 'descs': serializeDescriptors(c.descs)} for c in chars]
    
    return [{'uuid': serializeUuid(s.uuid), 'start_handle': s.start_handle, 'end_handle': s.end_handle, 'chars': serializeChars(s.chars)} for s in services]

def deserializeServices(services):
    def deserializeUuid(uuid):
        return nrf_ble_driver.BLEUUID(uuid['value'], nrf_ble_driver.BLEUUIDBase(*uuid['base']))

    def deserializeChar(char):
        deserializeProps = lambda props: nrf_ble_driver.BLECharProperties(props['broadcast'], props['read'], props['write_wo_resp'], props['write'], props['notify'], props['indicate'], props['auth_signed_wr'])
        charo = nrf_ble_driver.BLECharacteristic(deserializeUuid(char['uuid']), deserializeProps(char['char_props']), char['handle_decl'], char['handle_value'])
        charo.end_handle = char['end_handle']
        charo.descs = [nrf_ble_driver.BLEDescriptor(deserializeUuid(d['uuid']), d['handle']) for d in char['descs']]
        return charo
    
    serviceso = [nrf_ble_driver.BLEService(deserializeUuid(s['uuid']), s['start_handle'], s['end_handle']) for s in services]
    for o, s in zip(serviceso, services):
        for c in s['chars']:
            o.char_add(deserializeChar(c))
    return serviceso

# Helpers to initialize nRF dongles
def get_available_nrf_dongles():
    descs = nrf_ble_driver.BLEDriver.enum_serial_ports()
    choices = ["{}: {}".format(d.port, d.serial_number) for d in descs]

    if len(descs) == 0:
        descs = [port for port, desc, hwid in serial.tools.list_ports.comports()]
        choices = ["{}: {}".format(port, desc) for port, desc, hwid in serial.tools.list_ports.comports()]
    else:
        descs = [d.port for d in descs]
    
    nrf_ds = []
    for c, d in zip(choices, descs):
        if c.split(': ')[1].startswith('nRF Connect USB CDC ACM'):
            nrf_ds += [d]
    
    return choices, nrf_ds

# BLE data logger
class BleDataCollector(nrf_ble_driver.BLEDriverObserver, nrf_ble_driver.BLEAdapterObserver):
    ADV_TYPE_NAMES = {
            None: 'SCAN_RSP',
            nrf_ble_driver.BLEGapAdvType.connectable_undirected: 'ADV_IND',
            nrf_ble_driver.BLEGapAdvType.connectable_directed: 'ADV_DIRECT_IND',
            nrf_ble_driver.BLEGapAdvType.scanable_undirected: 'ADV_SCAN_IND',
            nrf_ble_driver.BLEGapAdvType.non_connectable_undirected: 'ADV_NONCONN_IND',
        }

    def __init__(self, serial_port, log_severity_level="info"):
        super(BleDataCollector, self).__init__()
        
        driver = nrf_ble_driver.BLEDriver(serial_port=serial_port, auto_flash=False, baud_rate=115200, log_severity_level=log_severity_level)
        self.adapter = nrf_ble_adapter.BLEAdapter(driver)

        self.conn_q = Queue()
        self.adapter.observer_register(self)
        self.adapter.driver.observer_register(self)
        self.adapter.default_mtu = 250

    def open(self, icid = 'NRF52'):
        self.adapter.driver.open()
        if icid == 'NRF51':
            self.adapter.driver.ble_enable(
                BLEEnableParams(
                    vs_uuid_count=1,
                    service_changed=0,
                    periph_conn_count=0,
                    central_conn_count=1,
                    central_sec_count=0,
                )
            )
        elif icid == 'NRF52':
            gatt_cfg = nrf_ble_driver.BLEConfigConnGatt()
            gatt_cfg.att_mtu = self.adapter.default_mtu
            gatt_cfg.tag = 1
            self.adapter.driver.ble_cfg_set(nrf_ble_driver.BLEConfig.conn_gatt, gatt_cfg)

            self.adapter.driver.ble_enable()

    def close(self):
        self.adapter.driver.close()
    
    def scan(self, scan_duration=30, interval_ms=200, window_ms=150, active=True):
        params = nrf_ble_driver.BLEGapScanParams(interval_ms=interval_ms, window_ms=window_ms, timeout_s=int(scan_duration), active=active)
        self.adapter.driver.ble_gap_scan_start(scan_params=params)

    def connect(self, peer_addr, discover_handles=True, handles_filen='handles_bak.pkl', min_conn_interval_ms=90, max_conn_interval_ms=110, conn_sup_timeout_ms=3_000, slave_latency=0):
        cparams_discovery = nrf_ble_driver.BLEGapConnParams(min_conn_interval_ms=90, max_conn_interval_ms=110, conn_sup_timeout_ms=3_000, slave_latency=0)
        self.adapter.connect(peer_addr, tag=1, conn_params=cparams_discovery)
        
        conn_handle = self.conn_q.get(timeout=conn_sup_timeout_ms)

        # Restore handles
        if not discover_handles:
            with open(handles_filen, 'rb') as ifh:
                self.adapter.db_conns[conn_handle].services = deserializeServices(pickle.load(ifh))

        # Discover service and characteristic handles
        else:
            self.adapter.service_discovery(conn_handle)
            
            # Fix 128-bit characteristics
            for s in self.adapter.db_conns[conn_handle].services:
                char_start_handles = [c.handle_decl for c in s.chars]

                # Iterate over chars
                for kc, c in enumerate(s.chars):

                    # Check if it has a 128-bit UUID
                    if c.uuid.base.type == nrf_ble_driver.BLEUUID.Standard.unknown.value:
                        
                        # Re-read UUID value
                        self.adapter.driver.ble_gattc_read(conn_handle, c.handle_decl, 0)
                        response = self.adapter.evt_sync[conn_handle].wait(evt=nrf_ble_driver.BLEEvtID.gattc_evt_read_rsp)
                        if response is None or response is False:
                            logging.warning('Failed discovering 128-bit UUID for characteristic')
                        else:
                            if response['status'] != nrf_ble_driver.BLEGattStatusCode.success:
                                logging.warning('Failed discovering 128-bit UUID for characteristic')
                            
                            else:
                                # Check response length.
                                if len(response['data']) != 19:
                                    logging.warning('Failed discovering 128-bit UUID for characteristic')

                                else:
                                    base_uuid = int.from_bytes(bytes(response['data'][3:19]), 'little')
                                    base = nrf_ble_driver.BLEUUIDBase(response['data'][3:19][::-1], nrf_ble_driver.driver.BLE_UUID_TYPE_VENDOR_BEGIN)
                                    self.adapter.driver.ble_vs_uuid_add(base)
                                    c.uuid = uuid_base_from_16bytes(base_uuid)
                        
                        # Re-discover descriptors based on Find Information Request procedure
                        c.end_handle = (char_start_handles[kc + 1] - 1) if (kc + 1) < len(char_start_handles) else s.end_handle
                        
                        descriptors = []
                        start_handle = c.handle_value
                        while start_handle <= c.end_handle:
                            self.adapter.driver.ble_gattc_desc_disc(conn_handle, start_handle, c.end_handle)
                            response_desc = self.adapter.evt_sync[conn_handle].wait(evt=nrf_ble_driver.BLEEvtID.gattc_evt_desc_disc_rsp)
                            if response_desc['status'] == nrf_ble_driver.BLEGattStatusCode.attribute_not_found:
                                break
                            else:
                                descriptors  += response_desc['descriptors']
                                start_handle =  max([d.handle for d in response_desc['descriptors']]) + 1
                        
                        if descriptors[0].uuid.value == c.uuid.value:
                            # Assume this to be the value handle with 128-bit UUID
                            # MIGHT NOT ALWAYS BE CORRECT!!!
                            descriptors[0].uuid = c.uuid

                        c.descs = descriptors

            # Save handles
            with open(handles_filen, 'wb') as ofh:
                gapdb = serializeServices(self.adapter.db_conns[conn_handle].services)
                pickle.dump(gapdb, ofh)
        
        cparams = nrf_ble_driver.BLEGapConnParams(min_conn_interval_ms, max_conn_interval_ms, conn_sup_timeout_ms, slave_latency)
        self.adapter.conn_param_update(conn_handle, cparams)
        
        return conn_handle
    
    def disconnect(self, conn_handle):
        self.adapter.disconnect(conn_handle)
    
    def read_char(self, conn_handle, char_uuid, service_uuid=None):
        attr_handle = self.get_char_value_handle(conn_handle, char_uuid, service_uuid)
        if attr_handle is None:
            return nrf_ble_driver.BLEGattStatusCode.attribute_not_found, None
        else:
            return self.adapter.read_req(conn_handle, None, attr_handle=attr_handle)

    def read_user_desc(self, conn_handle, char_uuid, service_uuid=None):
        attr_handle = self.get_char_user_description_handle(conn_handle, char_uuid, service_uuid)
        if attr_handle is None:
            return nrf_ble_driver.BLEGattStatusCode.attribute_not_found, None
        else:
            return self.adapter.read_req(conn_handle, None, attr_handle=attr_handle)
    
    def read_client_characteristic_configuration(self, conn_handle, char_uuid, service_uuid=None):
        attr_handle = self.get_char_client_configuration_handle(conn_handle, char_uuid, service_uuid)
        if attr_handle is None:
            return nrf_ble_driver.BLEGattStatusCode.attribute_not_found, None
        else:
            return self.adapter.read_req(conn_handle, None, attr_handle=attr_handle)
    
    def write_char(self, conn_handle, char_uuid, data, service_uuid=None, response=True):
        attr_handle = self.get_char_value_handle(conn_handle, char_uuid, service_uuid)
        if attr_handle is None:
            return nrf_ble_driver.BLEGattStatusCode.attribute_not_found
        else:
            if response:
                return self.adapter.write_req(conn_handle, None, data, attr_handle=attr_handle)
            else:
                return self.adapter.write_cmd(conn_handle, None, data, attr_handle=attr_handle)
    
    def write_client_characteristic_configuration(self, conn_handle, char_uuid, data, service_uuid=None):
        attr_handle = self.get_char_client_configuration_handle(conn_handle, char_uuid, service_uuid)
        if attr_handle is None:
            return nrf_ble_driver.BLEGattStatusCode.attribute_not_found
        else:
            return self.adapter.write_req(conn_handle, None, data, attr_handle=attr_handle)
    
    def get_char_value_handle(self, conn_handle, uuid, service_uuid=None):
        assert isinstance(uuid, nrf_ble_driver.BLEUUID), 'Invalid argument type'
        for s in self.adapter.db_conns[conn_handle].services:
            if service_uuid is None or s.uuid == service_uuid:
                for c in s.chars:
                    if c.uuid == uuid:
                        for d in c.descs:
                            if d.uuid == uuid:
                                return d.handle
        return None

    def get_char_user_description_handle(self, conn_handle, uuid, service_uuid=None):
        assert isinstance(uuid, nrf_ble_driver.BLEUUID), 'Invalid argument type'
        for s in self.adapter.db_conns[conn_handle].services:
            if service_uuid is None or s.uuid == service_uuid:
                for c in s.chars:
                    if c.uuid == uuid:
                        for d in c.descs:
                            uuid_value = d.uuid.value if type(d.uuid.value) == int else d.uuid.value.value
                            if uuid_value == nrf_ble_driver.driver.BLE_UUID_DESCRIPTOR_CHAR_USER_DESC:
                                return d.handle
        return None
    
    def get_char_client_configuration_handle(self, conn_handle, uuid, service_uuid=None):
        assert isinstance(uuid, nrf_ble_driver.BLEUUID), 'Invalid argument type'
        for s in self.adapter.db_conns[conn_handle].services:
            if service_uuid is None or s.uuid == service_uuid:
                for c in s.chars:
                    if c.uuid == uuid:
                        for d in c.descs:
                            uuid_value = d.uuid.value if type(d.uuid.value) == int else d.uuid.value.value
                            if uuid_value == nrf_ble_driver.driver.BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG:
                                return d.handle
        return None

    def enable_notification(self, conn_handle, char_uuid, service_uuid=None):
        return self.write_client_characteristic_configuration(conn_handle, char_uuid, [0x00, 0x01], service_uuid=service_uuid)
    
    def enable_indication(self, conn_handle, char_uuid, service_uuid=None):
        return self.write_client_characteristic_configuration(conn_handle, char_uuid, [0x00, 0x02], service_uuid=service_uuid)
    
    def disable_indication_notification(self, conn_handle, char_uuid, service_uuid=None):
        return self.write_client_characteristic_configuration(conn_handle, char_uuid, [0x00, 0x00], service_uuid=service_uuid)

    def on_gap_evt_connected(self, ble_driver, conn_handle, peer_addr, role, conn_params):
        logging.debug('New connection: {}'.format(conn_handle))
        self.conn_q.put(conn_handle)

    def on_indication(self, ble_driver, conn_handle, uuid, data):
        logging.debug('Indication received for UUID '+uuid_to_str(uuid)+': 0x' + binascii.hexlify(bytes(data)).decode('utf-8'))
    
    def on_notification(self, ble_driver, conn_handle, uuid, data):
        logging.debug('Notification received for UUID '+uuid_to_str(uuid)+': 0x' + binascii.hexlify(bytes(data)).decode('utf-8'))

class BleDataConnection:
    def __init__(self, collector, peer_addr, discover_handles=True, handles_filen='handles_bak.pkl', min_conn_interval_ms=90, max_conn_interval_ms=110, conn_sup_timeout_ms=3_000, slave_latency=0):
        self.collector = collector
        self.peer_addr = peer_addr
        self.kwargs = {
                'discover_handles': discover_handles,
                'handles_filen': handles_filen,
                'min_conn_interval_ms': min_conn_interval_ms,
                'max_conn_interval_ms': max_conn_interval_ms,
                'conn_sup_timeout_ms': conn_sup_timeout_ms,
                'slave_latency': slave_latency,
            }
    
    def __enter__(self):
        self.conn_handle = self.collector.connect(self.peer_addr, **self.kwargs)
        if self.conn_handle is None:
            raise ConnectionError('Could not establish connection!')
        return self.conn_handle
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.collector.disconnect(self.conn_handle)