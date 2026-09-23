# Measuring a change to the search

Aletheia has no unit tests. What it has is a suite of problems it either solves
or does not, and the honest way to judge a change to the search or to the
semantics is to run them before and after.

The problems are the IPC 2026 epistemic benchmarks,
[ipc2026-epistemic/benchmarks](https://github.com/ipc2026-epistemic/benchmarks):
100 instances over three tiers, each tier with its own action-type library.

```bash
git clone https://github.com/ipc2026-epistemic/benchmarks.git /tmp/benchmarks

tools/benchmark.py --benchmarks /tmp/benchmarks \
                   --planner build-before/epistemic_planner \
                   --tasks /tmp/ipc-tasks --out before.csv --timeout 15

tools/benchmark.py --benchmarks /tmp/benchmarks \
                   --planner build-after/epistemic_planner \
                   --tasks /tmp/ipc-tasks --out after.csv --timeout 15

tools/compare.py before.csv after.csv
```

Grounding is plank's and is cached in `--tasks`, so the second run plans over
byte-identical tasks rather than over a second grounding of the same sources.
Build the two planners in separate directories, or the run measures whichever
binary happened to be on disk when each problem came up.

`compare.py` exits non-zero when a problem that was solved before is not solved
after, because that is the one outcome no change to the semantics is allowed to
produce quietly.

## The regression that goes with the frame guard

`tests/off-frame/run.sh` is a single task and two assertions: the policy for a
two-site goal has to act on both sites, and the south half on its own has to
stay solvable. It fails on any build without the guard and passes with it,
which is the shortest statement of what the guard is for.
