"""Wire compatibility, freshness and reconnect regression tests."""
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from app import Dashboard
from mqtt_protocol import decode_telemetry

def motion_packet(faults=0,left=30,right=24):
    value=1|(1<<2)|(faults<<5)|(left<<9)|(right<<18)|(42<<27)|(35<<34)|(100<<41)|(200<<54)
    return value.to_bytes(9,'little')

class TelemetryTests(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.d=Dashboard('192.0.2.1',clock=lambda:self.now)
        self.d.broker_connected=self.d.monitor_ready=True
    def send(self,kind,data,retained=False):
        self.d.on_message(None,None,SimpleNamespace(topic=self.d.root+'/'+kind,payload=data,retain=retained))
    def test_motion_and_faults(self):
        self.send('motion',motion_packet(4))
        item=self.d.snapshot()['telemetry']['motion']
        self.assertTrue(item['fresh'])
        self.assertEqual(item['data']['left_rpm'],30)
        self.assertEqual(item['data']['right_pwm'],35)
        self.assertEqual(item['data']['left_progress'],100)
        self.assertEqual(item['data']['fault_labels'],['Wheel stalled'])
        self.send('vehicle',bytes([5|(1<<3)]))
        self.assertEqual(self.d.snapshot()['telemetry']['vehicle']['data']['state'],'FAULT')
    def test_all_wire_payloads(self):
        self.assertEqual(decode_telemetry('line',bytes([5|(2<<3)]))['barcode'],'B')
        self.assertEqual(decode_telemetry('terrain',(125|(2<<11)).to_bytes(2,'little')),dict(height_mm=125,phase=2))
        obstacle=(500|(80<<12)|(200<<23)|(300<<35)).to_bytes(6,'little')
        self.assertEqual(decode_telemetry('obstacle',obstacle),dict(nearest_mm=500,width_mm=80,left_mm=200,right_mm=300))
    def test_each_topic_has_independent_freshness(self):
        self.send('motion',motion_packet())
        self.now+=3
        self.send('status',b'\x07')
        self.send('vehicle',b'\x01')
        state=self.d.snapshot()
        self.assertTrue(state['pico']['fresh'])
        self.assertFalse(state['telemetry']['motion']['fresh'])
        self.assertTrue(state['telemetry']['vehicle']['fresh'])
        self.assertIsNone(state['telemetry']['terrain']['data'])
    def test_retained_and_bad_payloads_do_not_refresh(self):
        self.send('motion',motion_packet(),True)
        self.assertIsNone(self.d.snapshot()['telemetry']['motion']['data'])
        self.send('motion',motion_packet());self.now+=3
        for packet in (b'',b'\x00',bytes([255])*9):self.send('motion',packet)
        self.assertFalse(self.d.snapshot()['telemetry']['motion']['fresh'])
        self.assertEqual(self.d.snapshot()['telemetry']['motion']['data']['left_rpm'],30)
    def test_reconnect_needs_new_sample(self):
        self.send('motion',motion_packet())
        self.d.on_disconnect(None,None,None,None,None)
        self.d.broker_connected=self.d.monitor_ready=True
        self.assertFalse(self.d.snapshot()['telemetry']['motion']['fresh'])
        self.send('motion',motion_packet())
        self.assertTrue(self.d.snapshot()['telemetry']['motion']['fresh'])
    def test_padding_and_range_rejection(self):
        packets=[('vehicle',b'\x07'),('line',b'\xc0'),('line',b'\x28'),
                 ('terrain',(2001).to_bytes(2,'little')),('terrain',b'\x00\xc0'),
                 ('obstacle',(4001).to_bytes(6,'little')),('obstacle',(1<<47).to_bytes(6,'little')),
                 ('motion',(1<<67).to_bytes(9,'little')),('motion',(101<<27).to_bytes(9,'little'))]
        for kind,payload in packets:
            with self.subTest(kind=kind,payload=payload):
                with self.assertRaises(ValueError):decode_telemetry(kind,payload)
    def test_history_and_events_are_bounded(self):
        for _ in range(200):self.send('motion',motion_packet())
        self.assertEqual(len(self.d.snapshot()['motion_history']),120)
        self.assertLessEqual(len(self.d.snapshot()['events']),40)
    def test_controller_explains_start_block_without_weakening_guards(self):
        packet=bytearray(25);packet[0]=1;packet[3]=7;packet[4]=1|2|4|64
        packet[6:8]=(900).to_bytes(2,'little');packet[10:12]=(850).to_bytes(2,'little')
        packet[24]=7
        self.send('status',b'\x07')
        self.send('controller',packet)
        state=self.d.snapshot()
        self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['start_block_label'],'Junction / line stop latched')
        packet[3]=0;packet[4]=1|2|64;self.send('controller',packet)
        self.assertTrue(self.d.snapshot()['start_allowed'])
        packet[0]=2
        with self.assertRaises(ValueError):decode_telemetry('controller',packet)

    def test_timing_diagnostics_and_stopped_reset(self):
        packet=bytearray(31);packet[0]=2;packet[1]=5;packet[3]=3;packet[4]=1|2|64;packet[5]=2
        packet[25:29]=(12000).to_bytes(4,'little');packet[29:31]=(15000).to_bytes(2,'little')
        motor=bytearray(motion_packet(2));motor[0]&=~3
        self.send('status',b'\x07');self.send('motion',motor);self.send('controller',packet)
        state=self.d.snapshot()
        self.assertTrue(state['reset_allowed']);self.assertFalse(state['start_allowed'])
        self.assertEqual(state['telemetry']['controller']['data']['timing_gap_us'],12000)
        self.assertEqual(state['telemetry']['controller']['data']['timing_limit_us'],15000)
        packet[4]|=128;self.send('controller',packet)
        self.assertFalse(self.d.snapshot()['reset_allowed'])
        packet[0]=1
        with self.assertRaises(ValueError):decode_telemetry('controller',packet)
        self.send('ack',b'\x04');self.assertEqual(self.d.snapshot()['last_ack']['command'],'reset')
        self.send('ack',b'\x06');self.assertTrue(self.d.snapshot()['last_ack']['rejected'])

    def test_subscribes_every_topic(self):
        result=[]
        client=SimpleNamespace(subscribe=lambda topics:(result.extend(topics) or (0,1)))
        self.d.on_connect(client,None,None,SimpleNamespace(is_failure=False),None)
        self.assertEqual({topic.rsplit('/',1)[-1] for topic,qos in result},
                         {'status','ack','motion','line','terrain','obstacle','vehicle','controller'})

if __name__=='__main__':unittest.main()
