#!/usr/bin/env python3
"""Compare two benchmark runs.

    tools/compare.py before.csv after.csv

Says what changed and nothing else: how many problems each run solved, which
ones changed status either way, and where the search work moved. A guard that
refuses states the search used to accept is expected to cost expansions; what
it must not do is lose problems.
"""

import csv
import pathlib
import sys


def read(path):
    with pathlib.Path(path).open() as handle:
        return {row['problem']: row for row in csv.DictReader(handle)}


def number(row, field):
    try:
        return int(row[field])
    except (KeyError, ValueError):
        return None


def main():
    if len(sys.argv) != 3:
        print(__doc__)
        return 2

    before, after = read(sys.argv[1]), read(sys.argv[2])
    shared = [name for name in before if name in after]

    solved_before = [n for n in shared if before[n]['solved'] == 'True']
    solved_after = [n for n in shared if after[n]['solved'] == 'True']

    print(f'{len(shared)} problems in both runs')
    print(f'  solved before: {len(solved_before)}')
    print(f'  solved after:  {len(solved_after)}')

    lost = sorted(set(solved_before) - set(solved_after))
    won = sorted(set(solved_after) - set(solved_before))

    if lost:
        print(f'\nlost ({len(lost)}):')
        for name in lost:
            print(f'  {name:55s} {after[name]["note"][:60]}')
    if won:
        print(f'\nwon ({len(won)}):')
        for name in won:
            print(f'  {name}')
    if not lost and not won:
        print('\nno problem changed status')

    # Where the work moved, over the problems both runs solved.
    both = sorted(set(solved_before) & set(solved_after))
    moved = []
    for name in both:
        was, now = number(before[name], 'expanded'), number(after[name], 'expanded')
        if was and now and was != now:
            moved.append((now / was, was, now, name))

    if moved:
        moved.sort(reverse=True)
        print(f'\nexpansions changed on {len(moved)} of {len(both)} solved by both:')
        for ratio, was, now, name in moved[:10]:
            print(f'  {ratio:6.2f}x  {was:>9} -> {now:<9} {name}')
        if len(moved) > 10:
            print(f'  ... and {len(moved) - 10} more')
        worse = [m for m in moved if m[0] > 1.0]
        print(f'  more expansions on {len(worse)}, fewer on {len(moved) - len(worse)}')
    else:
        print(f'\nexpansions identical on all {len(both)} solved by both')

    # A validator verdict that changed is worth more than any count.
    verdicts = [(n, before[n]['validator'], after[n]['validator'])
                for n in shared
                if before[n]['validator'] != after[n]['validator']]
    if verdicts:
        print(f'\nvalidator verdict changed on {len(verdicts)}:')
        for name, was, now in verdicts[:10]:
            print(f'  {name:55s} {was or "-"} -> {now or "-"}')

    return 1 if lost else 0


if __name__ == '__main__':
    sys.exit(main())
