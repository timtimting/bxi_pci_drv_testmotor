#!/usr/bin/env python3
"""Reassemble legacy zero-cycle CAN frame dumps into readable motor text."""
import argparse
from pathlib import Path
import re

FRAME = re.compile(r'^\[time_us=(\d+) bus=(\d+) can_id=(0x[0-9A-Fa-f]+) len=(\d+)\] hex:((?: [0-9A-Fa-f]{2})*) text:')


def decode(text):
    if not any(FRAME.match(line) for line in text.splitlines()):
        return text.replace('\r', '')
    output = []
    for line in text.splitlines():
        match = FRAME.match(line)
        if match:
            data = bytes.fromhex(match[5])
            if len(data) != int(match[4]):
                raise ValueError('Frame length mismatch at timestamp ' + match[1])
            output.append(data.replace(b'\r', b''))
        elif (line.startswith(('==========', 'RESULT ', 'SUMMARY ', 'Failed cycle=',
                               'target_motor_turns=', '10 additional'))):
            output.append(('\n' + line + '\n').encode('utf-8'))
        # The legacy `text:` field can contain newlines. Ignore its continuation
        # lines: the hex field above already contains the complete frame bytes.
    return b''.join(output).decode('utf-8', errors='replace')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('-o', '--output', type=Path)
    args = parser.parse_args()
    output = args.output or Path(str(args.log) + '.readable.txt')
    output.write_text(decode(args.log.read_text(errors='replace')), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    main()
