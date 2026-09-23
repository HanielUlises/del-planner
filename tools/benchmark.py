#!/usr/bin/env python3
"""Run the planner over the IPC 2026 epistemic benchmarks and record what it did.

    tools/benchmark.py --benchmarks <path to ipc2026-epistemic/benchmarks> \
                       --planner build/epistemic_planner --out before.csv

Grounding is plank's and is cached under --tasks, so that two runs of this
script compare two planners over byte-identical tasks rather than over two
groundings of the same sources. A run records, per problem: whether a plan came
back, what the validator said about it, the depth it was found at, how many
nodes were expanded and generated, and the wall clock.

The point of the numbers is comparison. A guard that refuses states the search
used to accept changes the search space, and the way to see whether it changed
it for the worse is to run the suite before and after.
"""

import argparse
import csv
import json
import pathlib
import re
import subprocess
import sys
import time


TIERS = {'1-basic': 'basic.epddl',
         '2-intermediate': 'intermediate.epddl',
         '3-hard': 'hard.epddl'}

SOLUTION = re.compile(r'Solution found at depth (\d+)\s+Expanded=(\d+)\s+Generated=(\d+)')
VALIDATOR = re.compile(r'\[validator\] (OK|FAILED)(.*)')


def problems(root: pathlib.Path):
    for tier, library in TIERS.items():
        lib = root / tier / library
        if not lib.exists():
            continue
        for domain in sorted((root / tier).iterdir()):
            if not domain.is_dir():
                continue
            for problem in sorted((domain / 'problems').glob('*.epddl')):
                yield tier, domain.name, problem, domain / 'domain.epddl', lib


def ground(domain, problem, library, into: pathlib.Path):
    """Ground one problem, or return None with the reason it could not be."""
    into.mkdir(parents=True, exist_ok=True)
    existing = list(into.glob('*.json'))
    if existing:
        return existing[0]

    done = subprocess.run(
        ['plank', 'export', '-d', str(domain), '-p', str(problem),
         '-l', str(library), '-o', str(into)],
        capture_output=True, text=True, timeout=600)
    produced = list(into.glob('*.json'))
    if not produced:
        print(f'  could not ground: {done.stderr.strip()[:120]}', file=sys.stderr)
        return None
    return produced[0]


def plan(planner, task, timeout, extra):
    started = time.monotonic()
    try:
        done = subprocess.run(
            [planner, '--task', str(task), '--plan', '/dev/null',
             '--timeout', str(timeout), *extra],
            capture_output=True, text=True, timeout=timeout + 30)
        output = done.stdout + done.stderr
    except subprocess.TimeoutExpired:
        return {'solved': False, 'validator': '', 'depth': '', 'expanded': '',
                'generated': '', 'seconds': round(time.monotonic() - started, 2),
                'note': 'killed'}

    elapsed = round(time.monotonic() - started, 2)
    found = SOLUTION.search(output)
    verdict = VALIDATOR.search(output)

    return {
        'solved': bool(found),
        'validator': verdict.group(1) if verdict else '',
        'depth': found.group(1) if found else '',
        'expanded': found.group(2) if found else '',
        'generated': found.group(3) if found else '',
        'seconds': elapsed,
        'note': '' if found else output.strip().splitlines()[-1][:80] if output.strip() else '',
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--benchmarks', required=True, type=pathlib.Path)
    ap.add_argument('--planner', required=True)
    ap.add_argument('--tasks', type=pathlib.Path, default=pathlib.Path('/tmp/ipc-tasks'))
    ap.add_argument('--out', required=True, type=pathlib.Path)
    ap.add_argument('--timeout', type=int, default=30)
    ap.add_argument('--tier', action='append', choices=list(TIERS))
    ap.add_argument('planner_args', nargs='*')
    args = ap.parse_args()

    rows = []
    for tier, domain_name, problem, domain, library in problems(args.benchmarks):
        if args.tier and tier not in args.tier:
            continue

        name = f'{tier}/{domain_name}/{problem.stem}'
        task = ground(domain, problem, library, args.tasks / tier / domain_name / problem.stem)
        if task is None:
            rows.append({'problem': name, 'solved': False, 'note': 'not ground'})
            continue

        result = plan(args.planner, task, args.timeout, args.planner_args)
        result['problem'] = name
        rows.append(result)
        print(f'{name:55s} {"solved" if result["solved"] else "-":8s} '
              f'{result.get("validator", ""):3s} {result.get("expanded", ""):>10s} '
              f'{result["seconds"]:>7}s', flush=True)

    fields = ['problem', 'solved', 'validator', 'depth', 'expanded', 'generated',
              'seconds', 'note']
    with args.out.open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field, '') for field in fields})

    solved = sum(1 for row in rows if row.get('solved'))
    print(f'\n{solved} of {len(rows)} solved, written to {args.out}')


if __name__ == '__main__':
    main()
