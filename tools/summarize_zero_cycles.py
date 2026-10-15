#!/usr/bin/env python3
"""Summarize independent power-cycle failures by corrected position and ENCO raw angle."""
import argparse
import csv
import math
from pathlib import Path


def number(row, key):
    try:
        value = float(row.get(key, ''))
        return value if math.isfinite(value) else None
    except (ValueError, TypeError):
        return None


def analyze(paths, motor_bin=.1, raw_bin=1., gear_ratio=None):
    if not math.isfinite(motor_bin) or not math.isfinite(raw_bin) or motor_bin <= 0 or not 0 < raw_bin <= 180:
        raise ValueError('Bin widths must be positive and finite; raw bin <=180 degrees')
    if gear_ratio is not None and (not math.isfinite(gear_ratio) or gear_ratio <= 0):
        raise ValueError('gear ratio must be positive and finite')
    positions, raw_angles = {}, {}
    total = failures = fallbacks = raw_missing = confirmed_returns = repeated_returns = 0

    def add(table, key, row, reference):
        entry = table.setdefault(key, {'trials': 0, 'failures': 0, 'reproduced_returns': 0, 'max_error': 0., 'refs': []})
        entry['trials'] += 1
        if row['result'] == 'FAIL':
            entry['failures'] += 1
            entry['refs'].append(reference)
            entry['reproduced_returns'] += row.get('test_kind') == 'revisit' and row.get('revisit_hit') == 'true'
        entry['max_error'] = max(entry['max_error'], abs(number(row, 'error_motor_turns') or 0))

    for path in paths:
        with Path(path).open() as stream:
            rows = csv.DictReader(line for line in stream if not line.startswith('#') and line.strip())
            if not rows.fieldnames or 'result' not in rows.fieldnames:
                raise ValueError(str(path) + ': expected the main zero-cycle CSV, not summary/retry log')
            for row in rows:
                if row.get('result') not in ('PASS', 'FAIL'):
                    continue
                total += 1
                failures += row['result'] == 'FAIL'
                fallbacks += row.get('target_reached') == 'false'
                hit = row.get('test_kind') == 'revisit' and row.get('revisit_hit') == 'true'
                confirmed_returns += hit
                repeated_returns += hit and row['result'] == 'FAIL'
                reference = str(path) + ':cycle=' + row.get('cycle', '?')
                position = number(row, 'physical_before_motor_turns')
                if position is None:
                    if gear_ratio is None:
                        raise ValueError(str(path) + ': legacy CSV needs --gear-ratio for position conversion')
                    value = number(row, 'before_output_rad')
                    position = None if value is None else value * gear_ratio / (2*math.pi)
                if position is not None:
                    add(positions, math.floor(position/motor_bin + .5), row, reference)
                raw = number(row, 'before_enc2_raw_deg')
                if raw is None:
                    raw_missing += 1
                else:
                    # Raw angle is circular; 359.8 and -0.2 must share a bin.
                    bins = math.ceil(360/raw_bin)
                    key = math.floor((raw % 360)/raw_bin + .5) % bins
                    add(raw_angles, key, row, reference)
    return dict(total=total, failures=failures, fallbacks=fallbacks, raw_missing=raw_missing,
                confirmed_returns=confirmed_returns, repeated_returns=repeated_returns,
                positions=positions, raw_angles=raw_angles)


def render(result, motor_bin, raw_bin):
    lines = [f"上下电比较 {result['total']} 次：PASS {result['total']-result['failures']}，FAIL {result['failures']}；未到目标兜底 {result['fallbacks']} 次。",
             f"离开后返回且实际到达异常点：{result['confirmed_returns']} 次，其中失败复现 {result['repeated_returns']} 次。",
             f"ENCO 原始角度缺失 {result['raw_missing']} 次；固定点10次复测不混入本统计。"]
    for title, key, width, unit in [('原零位坐标的电机端位置', 'positions', motor_bin, '圈'),
                                    ('断电前 ENCO 原始角度', 'raw_angles', raw_bin, '°')]:
        lines.append(f'\n{title}：按 {width:g}{unit} 分箱（区间归类不代表角度完全相同）')
        groups = [(k, v) for k, v in result[key].items() if v['failures']]
        for bucket, data in sorted(groups, key=lambda x: (-x[1]['failures'], x[0])):
            lines.append(f"  中心 {bucket*width:+.6f}{unit}：失败 {data['failures']}/{data['trials']}，返回复现 {data['reproduced_returns']}，最大错位 {data['max_error']:.6f} 电机圈")
            lines.extend('    ' + reference for reference in data['refs'])
        if not groups:
            lines.append('  无可统计的失败数据。')
    return '\n'.join(lines) + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('csv', type=Path, help='One run; separate runs may have different initial zeros')
    parser.add_argument('--motor-bin-turns', type=float, default=.1)
    parser.add_argument('--enco-bin-deg', type=float, default=1.)
    parser.add_argument('--gear-ratio', type=float, help='Required only for legacy CSV without corrected positions')
    parser.add_argument('-o', '--output', type=Path)
    args = parser.parse_args()
    result = analyze([args.csv], args.motor_bin_turns, args.enco_bin_deg, args.gear_ratio)
    report = render(result, args.motor_bin_turns, args.enco_bin_deg)
    print(report, end='')
    output = args.output or Path(str(args.csv)+'.analysis.txt')
    output.write_text(report, encoding='utf-8')
    print('分析文件：' + str(output))


if __name__ == '__main__':
    main()
