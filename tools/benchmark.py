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
import resource
import re
import subprocess
import sys
import time


TIERS = {'1-basic': 'basic.epddl',
         '2-intermediate': 'intermediate.epddl',
         '3-hard': 'hard.epddl'}

# Each strategy announces a solution in its own words --- AO* gives a depth,
# GBFS a length, the replanner neither --- and a run that falls back from one
# strategy to another prints more than one line. What they share is the words
# "Solution found" and the counters, so the line is found by the words and
# read by the counters.
SOLUTION = re.compile(r'Solution found')
COUNTER = re.compile(r'(Expanded|Generated|Length|depth)=?\s*(\d+)')
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


def usable(path: pathlib.Path):
    """A grounded task that parses.

    A grounding killed mid-print leaves one that does not, and a cache that
    handed it back would fail every later run for a reason that has nothing to
    do with the planner."""
    try:
        with path.open() as handle:
            json.load(handle)
        return True
    except (json.JSONDecodeError, OSError):
        return False


def ground(domain, problem, library, into: pathlib.Path, seconds, gigabytes):
    """Ground one problem, or return None when it cannot be ground in budget.

    Gossip at thirteen agents asks for more memory than the machine has, and a
    problem nobody can ground is not a problem either planner gets to answer.
    Both runs read the same cache, so an instance left out is left out of the
    comparison rather than counted against one side of it."""
    into.mkdir(parents=True, exist_ok=True)
    for existing in into.glob('*.json'):
        if usable(existing):
            return existing
        existing.unlink()

    limit = gigabytes * 1024 ** 3

    def capped():
        resource.setrlimit(resource.RLIMIT_AS, (limit, limit))

    try:
        done = subprocess.run(
            ['plank', 'export', '-d', str(domain), '-p', str(problem),
             '-l', str(library), '-o', str(into)],
            capture_output=True, text=True, timeout=seconds, preexec_fn=capped)
        complaint = done.stderr.strip()[:120] or 'plank printed nothing'
    except subprocess.TimeoutExpired:
        complaint = f'grounding took longer than {seconds}s'

    for produced in into.glob('*.json'):
        if usable(produced):
            return produced
        produced.unlink()

    print(f'  could not ground: {complaint}', file=sys.stderr)
    return None


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
    found = None
    for line in output.splitlines():
        if SOLUTION.search(line):
            found = dict((key.lower(), value) for key, value in COUNTER.findall(line))
    verdict = VALIDATOR.search(output)

    tail = output.strip().splitlines()[-1][:80] if output.strip() else ''

    return {
        'solved': bool(found),
        # Empty for a linear plan, which is not validated: only a policy is.
        'validator': verdict.group(1) if verdict else '',
        'depth': (found or {}).get('depth', (found or {}).get('length', '')),
        'expanded': (found or {}).get('expanded', ''),
        'generated': (found or {}).get('generated', ''),
        'seconds': elapsed,
        'note': '' if found else tail,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--benchmarks', required=True, type=pathlib.Path)
    ap.add_argument('--planner', required=True)
    ap.add_argument('--tasks', type=pathlib.Path, default=pathlib.Path('/tmp/ipc-tasks'))
    ap.add_argument('--out', required=True, type=pathlib.Path)
    ap.add_argument('--timeout', type=int, default=30)
    ap.add_argument('--ground-timeout', type=int, default=120)
    ap.add_argument('--ground-memory', type=int, default=8,
                    help='gigabytes of address space plank may use')
    ap.add_argument('--tier', action='append', choices=list(TIERS))
    ap.add_argument('planner_args', nargs='*')
    args = ap.parse_args()

    fields = ['problem', 'solved', 'validator', 'depth', 'expanded', 'generated',
              'seconds', 'note']

    # Written as the run goes, because a suite that takes an hour is a suite
    # that will be interrupted, and half a comparison is worth having.
    handle = args.out.open('w', newline='')
    writer = csv.DictWriter(handle, fieldnames=fields)
    writer.writeheader()

    rows = []
    for tier, domain_name, problem, domain, library in problems(args.benchmarks):
        if args.tier and tier not in args.tier:
            continue

        name = f'{tier}/{domain_name}/{problem.stem}'
        task = ground(domain, problem, library,
                      args.tasks / tier / domain_name / problem.stem,
                      args.ground_timeout, args.ground_memory)
        if task is None:
            row = {'problem': name, 'solved': False, 'note': 'not ground'}
            rows.append(row)
            writer.writerow({field: row.get(field, '') for field in fields})
            handle.flush()
            continue

        result = plan(args.planner, task, args.timeout, args.planner_args)
        result['problem'] = name
        rows.append(result)
        writer.writerow({field: result.get(field, '') for field in fields})
        handle.flush()
        print(f'{name:55s} {"solved" if result["solved"] else "-":8s} '
              f'{result.get("validator", ""):3s} {result.get("expanded", ""):>10s} '
              f'{result["seconds"]:>7}s', flush=True)

    handle.close()

    solved = sum(1 for row in rows if row.get('solved'))
    print(f'\n{solved} of {len(rows)} solved, written to {args.out}')


if __name__ == '__main__':
    main()
