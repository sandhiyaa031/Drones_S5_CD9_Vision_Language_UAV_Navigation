#!/usr/bin/env python3
"""Generate a seeded debris release plan for spawn_study.py.

Boxes are released above the roof opening around the hovering UAV, several
in the air at once. The plan is fully determined by the seed.
"""

import argparse
import json
import random


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--seed', type=int, required=True)
    parser.add_argument('--count', type=int, default=14)
    parser.add_argument('--out', required=True)
    parser.add_argument('--min-offset', type=float, default=0.8,
                        help='minimum horizontal distance from the UAV axis')
    args = parser.parse_args()
    rng = random.Random(args.seed)
    plan = []
    while len(plan) < args.count:
        x = rng.uniform(-0.85, 1.75)
        y = rng.uniform(-0.75, 0.75)
        if (x * x + y * y) ** 0.5 < args.min_offset:
            continue
        size = round(rng.uniform(0.25, 0.5), 2)
        plan.append({
            'position': [round(x, 2), round(y, 2),
                         round(rng.uniform(6.0, 13.0), 2)],
            'velocity': [round(rng.uniform(-0.3, 0.3), 2),
                         round(rng.uniform(-0.3, 0.3), 2),
                         round(rng.uniform(-3.0, 0.0), 2)],
            'size': [size, round(rng.uniform(0.25, 0.5), 2), size],
            'mass': round(rng.uniform(0.8, 2.0), 2),
            'gap': round(rng.uniform(0.25, 1.2), 2),
            'label': f'seed {args.seed} #{len(plan)}'})
    with open(args.out, 'w') as handle:
        json.dump(plan, handle, indent=1)
    print(f'{len(plan)} releases -> {args.out}')


if __name__ == '__main__':
    main()
