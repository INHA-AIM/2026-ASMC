#!/usr/bin/env python3
"""Plot every fixed B0 repeat; standby outputs are absent, not zero Hz."""
import argparse
import json
from pathlib import Path


def main():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('summary', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = json.loads(args.summary.read_text())
    rows = [r for r in data['trials'] if not r['diagnostic'] and r['report']]
    fig, axes = plt.subplots(2, 2, figsize=(13, 9))
    styles = {'yolo': ('#3b70b4', -.24), 'front': ('#4b914a', -.08),
              'family': ('#dd8532', .08), 'resnet18': ('#9256a6', .24)}
    profiles = ['C1', 'C2', 'C3', 'C4', 'C5', 'C6']
    for name, (color, shift) in styles.items():
        points = []
        for row in rows:
            report = row['report']
            if report['phase'] != 'camera' or report['camera_debug']:
                continue
            measured = report.get('measurements', {}).get(name)
            if measured and measured.get('complete'):
                points.append((profiles.index(report['profile'])+shift+(report['repeat']-2)*.025, measured))
        axes[0, 0].scatter([x for x, _ in points], [r['hz'] for _, r in points], color=color, label=name, s=35)
        if name != 'yolo':
            axes[0, 1].scatter([x for x, _ in points], [r['first_second'] for _, r in points], color=color, label=name, s=35)
    axes[0, 0].axhline(18, color='#3b70b4', ls='--', alpha=.6, label='YOLO 18 Hz gate')
    axes[0, 0].axhline(9.5, color='gray', ls='--', alpha=.6, label='10 Hz profile 5 s gate')
    axes[0, 0].set(ylabel='Fresh output Hz (20 s mean)', title='Camera combinations: each repeat')
    axes[0, 1].axhline(9, color='gray', ls='--', alpha=.6)
    axes[0, 1].set(ylabel='Fresh outputs in first second', title='RPC-anchored transition; threshold = 9')
    for axis in axes[0]:
        axis.set_xticks(range(6))
        axis.set_xticklabels(profiles)
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    for position, bag in enumerate(('primary', 'shadow')):
        axis = axes[1, position]
        names = ['C1', 'C4', 'C5', 'C6']
        for metric, color, shift in (('processed', '#667f91', -.1), ('matched', '#dd8532', .1)):
            points = []
            for row in rows:
                report = row['report']
                if report['phase'] != 'fusion' or report['bag_id'] != bag:
                    continue
                fusion = report.get('fusion', {})
                value = len(fusion['published_processed_lidar_stamps_ns']) if metric == 'processed' else (fusion.get('totals') or {}).get('matched')
                if value is not None:
                    points.append((names.index(report['profile'])+shift+(report['repeat']-2)*.025, value))
            axis.scatter([x for x, _ in points], [y for _, y in points], color=color, label=metric, s=35)
        axis.set_xticks(range(4))
        axis.set_xticklabels(names)
        axis.set(ylabel='Whole-bag frames / matched pairs', title=bag+' LiDAR and fusion: each repeat')
        axis.grid(alpha=.2)
        axis.legend(fontsize=8)
    fig.suptitle('B0: RTX 4060 Ti 8GB, all models resident, MORAI stopped\n'
                 'Rates do not replace 5 s / gap / classification gates; no configuration adopted', fontsize=11)
    fig.tight_layout(rect=(0, 0, 1, .94))
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
