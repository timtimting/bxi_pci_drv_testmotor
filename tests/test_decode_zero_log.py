import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location('decode_zero_log', Path(__file__).resolve().parents[1] / 'tools/decode_zero_log.py')
decoder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(decoder)


class DecoderTests(unittest.TestCase):
    def test_fragments_and_summary(self):
        text = ('========== RETEST 1/10 POWER OFF/ON ==========\n'
                '[time_us=1 bus=0 can_id=0x7F2 len=4] hex: 45 4E 43 32 text: ENC2\n'
                '[time_us=2 bus=0 can_id=0x7F2 len=6] hex: 3A 20 4F 4B 0A 0D text: : OK\n'
                '\nRESULT retry=1 PASS\n')
        result = decoder.decode(text)
        self.assertIn('ENC2: OK\n', result)
        self.assertEqual(result.count('OK'), 1)
        self.assertIn('RESULT retry=1 PASS', result)

    def test_new_format(self):
        self.assertEqual(decoder.decode('ENC1: raw=100\r\n'), 'ENC1: raw=100\n')

    def test_bad_frame(self):
        with self.assertRaises(ValueError):
            decoder.decode('[time_us=1 bus=0 can_id=0x7F2 len=5] hex: 01 text: x\n')


if __name__ == '__main__':
    unittest.main()
