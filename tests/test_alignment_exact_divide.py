"""CPU verification of the actual portable Slang division helper; never dispatches GPU."""
import pathlib
import re
import shutil
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'ros2/src/so101_station_vulkan'
SHADER = PACKAGE / 'shaders/alignment.slang'


class ExactAlignmentDivideTest(unittest.TestCase):
    def test_shared_helper_against_hardware_and_integer_oracles(self):
        source = SHADER.read_text()
        for parent in ('alignment.slang', 'resident.slang'):
            self.assertEqual((PACKAGE / 'shaders' / parent).read_text().count('#include "exact_divide.slang"'), 1)
        canonical = (PACKAGE / 'shaders/exact_divide.slang').read_text()
        match = re.search(r'// BEGIN EXACT ALIGNMENT DIVISION\n(.*?)// END EXACT ALIGNMENT DIVISION', canonical, re.S)
        self.assertIsNotNone(match, 'actual shader must contain extractable exact division helper')
        helper = match.group(1)
        self.assertNotRegex(helper, r'\b(double|uint64_t|uint64|int64_t|int64|new|malloc|while|wait|Interlocked|GroupMemoryBarrier)\b')
        self.assertNotRegex(helper, r'\b(for|while)\s*\([^)]*;\s*;')
        loops = re.findall(r'for\s*\(([^)]*)\)', helper)
        self.assertCountEqual(loops, [
            'uint correction=0;correction<4;++correction',
            'int bit=22;bit>=0;--bit',
            'uint shift=0;shift<23 && a<8388608u;++shift',
            'uint shift=0;shift<23 && b<8388608u;++shift',
        ])
        self.assertIn('alignment_divide(tx,tz)', source)
        self.assertIn('alignment_divide(ty,tz)', source)
        self.assertIn('alignment_divide((float(i%p.color_width)-p.cx),p.fx)', source)
        self.assertIn('alignment_divide((float(i/p.color_width)-p.cy),p.fy)', source)
        compiler = shutil.which('c++')
        self.assertIsNotNone(compiler, 'CPU-only oracle requires a C++17 compiler')
        with tempfile.TemporaryDirectory(prefix='alignment-divide-') as directory:
            directory = pathlib.Path(directory)
            (directory / 'alignment_divide_shared.h').write_text(helper)
            executable = directory / 'oracle'
            subprocess.run([compiler, '-O2', '-std=c++17', '-ffp-contract=off',
                            '-I', str(directory), str(PACKAGE / 'test/alignment_divide_host_test.cpp'),
                            '-o', str(executable)], check=True, capture_output=True, text=True)
            result = subprocess.run([str(executable)], check=True, capture_output=True, text=True)
            self.assertIn('PASS', result.stdout)
            print(result.stdout.strip())


if __name__ == '__main__':
    unittest.main()
