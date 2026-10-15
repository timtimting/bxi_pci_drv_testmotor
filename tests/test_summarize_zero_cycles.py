import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('summary', Path(__file__).resolve().parents[1] / 'tools/summarize_zero_cycles.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


class SummaryTests(unittest.TestCase):
    def test_revisits_raw_wrapping_and_missing(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'run.csv'
            path.write_text('cycle,result,error_motor_turns,target_reached,physical_before_motor_turns,test_kind,revisit_hit,before_enc2_raw_deg\n'
                            '# offset changed after failure\n'
                            '1,FAIL,-13,true,6.235,random,false,359.8\n'
                            '2,PASS,0,true,1.2,random,false,nan\n'
                            '3,FAIL,6,true,6.24,revisit,true,-0.2\n'
                            '4,PASS,0,false,2,revisit,false,10\n')
            result = summary.analyze([path])
            self.assertEqual(result['total'], 4)
            self.assertEqual(result['failures'], 2)
            self.assertEqual(result['fallbacks'], 1)
            self.assertEqual(result['confirmed_returns'], 1)
            self.assertEqual(result['repeated_returns'], 1)
            self.assertEqual(result['raw_missing'], 1)
            self.assertEqual(result['raw_angles'][0]['failures'], 2)
            self.assertEqual(result['positions'][62]['failures'], 2)
            self.assertIn('失败 2/2', summary.render(result, .1, 1))

    def test_legacy_requires_ratio(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'old.csv'
            path.write_text('cycle,result,before_output_rad,error_motor_turns\n1,FAIL,1,6\n')
            with self.assertRaises(ValueError):
                summary.analyze([path])
            self.assertEqual(summary.analyze([path], gear_ratio=20)['failures'], 1)


if __name__ == '__main__':
    unittest.main()
