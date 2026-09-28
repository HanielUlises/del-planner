# Usage

Build, run, and configure the planner. For what it does and why, see the
[main README](../README.md); for measured results, see
[evaluation.md](evaluation.md).

## Requirements

- A C++23 compiler. GCC 11.4 is what the results were measured with.
- CMake ≥ 3.16
- [nlohmann/json](https://github.com/nlohmann/json) ≥ 3.10 (`nlohmann-json3-dev`)
- Python 3, for `serialize.py` and the benchmark scripts

## Build

```sh
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

The binary lands at `build/epistemic_planner`. Release builds enable `-O3` and
LTO, then strip; both matter for the figures in [evaluation.md](evaluation.md).

An Apptainer definition for the competition image is in `Apptainer.aletheia`.

### As a library

The same build also produces `libaletheia.a`, the planning core without
`main.cpp`, for programs that search in process.
[ePlanSys](https://github.com/ePlanSys/eplansys) is one: its ROS 2 plan
solver links it rather than keeping a copy of the sources. Install it and find
it by name:

```sh
cmake --install build --prefix <prefix>
```

```cmake
find_package(aletheia 0.2 REQUIRED)
target_link_libraries(my_target PRIVATE aletheia::aletheia)
```

Headers are included as `<aletheia/search.hpp>`. `strategy.hpp` turns the
labels the selection policy speaks in into heuristics and strategies, so a
program that honours the policy can build every choice it makes.

The library is compiled without `-march=native` and without LTO, whatever
`ALETHEIA_NATIVE` says, because both would leak into the program linking it:
`bitset.hpp` inlines a different `pext_word` under BMI2, and an archive of slim
LTO objects does not link into a program built without LTO. The binary keeps
both. `-DALETHEIA_LIBRARY=OFF` skips the library.

`package.xml` makes the checkout a colcon package, so a ROS 2 workspace builds
it from source like any other, and the installed `epistemic_planner` is on
`PATH` once the workspace is sourced.

## Running

```sh
build/epistemic_planner --task <task.json> --plan <plan.json> [options]
```

`--task` and `--plan` are required. The task is a grounded JSON task (the file
carrying a `"planning-task-info"` key). On success the plan file holds a JSON
array for a linear plan or a nested object for a conditional one; on failure it
holds `null`. An empty array means the goal already held.

### Options

| Option | Meaning |
|---|---|
| `--task <path>` | Grounded JSON task (required) |
| `--plan <path>` | Output plan file (required) |
| `--heuristic <label>` | `ug`, `ed`, `ks`, `wc`, `rpg`, `radd`, `kadd`, `kff`. Default: chosen by policy |
| `--strategy <label>` | `gbfs`, `ehc`, `aostar`, `replan`, `portfolio` (races compatible configurations in parallel). Default: chosen by policy |
| `--policy <path>` | Selection-policy JSON; replaces the built-in rules |
| `--print-policy` | Write the effective policy to stdout and exit |
| `--explain` | Report the task features and which rule decided each auto-selection |
| `--limit <n>` | Max nodes (GBFS/EHC) or max depth (AO\*); 0 = unlimited |
| `--timeout <s>` | Wall-clock limit in seconds, counted from process start and shared by every search and fallback |
| `--gbfs`, `--ehc`, `--conditional` | Aliases for `--strategy gbfs` / `ehc` / `aostar` |
| `--no-symmetry` | Disable agent-symmetry pruning (on by default) |
| `--threads <n>` | Worker threads for successor generation. Default: all cores; 1 = serial |
| `--kd45-repair` | Delete non-serial worlds after KD45 updates. Off by default, matching plank |
| `--consistent-beliefs` | Refuse any state in which an agent believes a contradiction. Off by default, matching plank; see below |
| `--no-portfolio` | Auto-selected AO\* on sensing tasks keeps the whole budget instead of handing over to replan |
| `--no-helpful` | GBFS expands every action instead of the relaxed plan's helpful actions first |
| `--signature` | Print the task's structural signature (topology, fixed-point depths, growth, symmetry) as JSON and exit |
| `--help` | Show usage |

An unknown heuristic or strategy label is an error listing the valid ones, not
a silent fallback.

### Consistent beliefs

An agent that observes an event its beliefs rule out is left with no accessible
world. In coin-in-the-box, B misses A's peek, believes A ignorant, and then
hears A announce tails. From that point `[B]φ` holds for every φ, and a goal
about what B believes is satisfied by a model that says nothing about it. The
IεPC reference plan for `coin4` reaches its goal this way, and so does the
default plan for `tests/consistent-beliefs/two-sites.json`, which never visits
the south site that half of its goal is about.

`--consistent-beliefs` refuses every product in which an agent has no
accessible world at a world reachable from the designated ones, and the
validator rejects a plan that passes through one, naming the agent. It is off
by default because plank accepts such plans. A system that executes the plan
should turn it on: a robot credited with a belief it cannot hold has not
learned anything. On the smoke suite every instance still solves; `coin4`,
`coin5` and `depot3` need longer plans, since their shortest ones depend on
a collapsed belief.

### Wrapper scripts

- `./aletheia.sh <task.json> <plan.json>` — the IεPC 2026 entry point. Builds
  the binary if missing, runs it, then flattens any conditional plan tree to a
  flat array via `serialize.py`, which the competition output format requires.
  Note that it passes `--heuristic ed` by default, so it does **not** exercise
  automatic heuristic selection unless you override it.
- `./build_test.sh` — builds and runs the whole `benchmarks/` tree, writing one
  log per instance to `smoke-logs/`. Also defaults to `--heuristic ed`.
- `./run_benchmarks.sh` — sweeps a task set across several heuristics into
  `results/`.

## Grounding large tasks

`plank export` writes every accessibility edge of the initial state and builds
the whole document in memory first. On IεPC `gos-13-all` (8 192 worlds, 13
agents) that runs past 31 GB before anything is written. `tools/ground` grounds
with plank's libraries and writes the same JSON, except that each agent's
relation is a table of distinct successor sets:

```json
"relations": {
  "A": { "sets": [["w0", "w2"], ["w1", "w3"]],
         "of":   [0, 1, 0, 1] }
}
```

`"of"` gives, for every world in the order of `"worlds"`, the index of its set;
set members may be world names or indices. The planner reads both this form and
plank's, agent by agent. On S5 and KD45 models the sets of one agent are
disjoint, so the relation is linear in the number of worlds: `gos-13-all`
grounds in about a minute to a 2.5 MB file, which the planner solves in seconds.
The initial state is also marked with `"relations-format": "successor-sets"` in
`"planning-task-info"`.

Build against a built plank checkout (the tool links its `epddl_lib` and
`del_lib` and needs Boost headers):

```sh
cmake -S tools/ground -B build-ground -DPLANK_DIR=/path/to/plank
cmake --build build-ground -j"$(nproc)"
build-ground/ground -d domain.epddl -p problem.epddl -l library.epddl -o task.json
```

`ground export -d … -p … -l … -o <dir>` takes the arguments of `plank export`
and, like it, writes `<dir>/<problem>.json`, so it can stand in for plank in
scripts that call the exporter.

## Selection policy

With neither `--heuristic` nor `--strategy` given, the planner picks both from
a rule table. The table is data, not code: it can be dumped, edited, and
supplied back without rebuilding.

```sh
build/epistemic_planner --print-policy > policy.json
# edit policy.json
build/epistemic_planner --task t.json --plan p.json --policy policy.json
```

### How rules are evaluated

Each of the two rule lists — `strategy` and `heuristic` — is evaluated
**first-match-wins**. A rule matches when *every* condition in its `when` array
holds. A rule with no conditions always matches, so it terminates the list and
acts as the default.

```json
{
  "strategy": [
    { "name": "sensing-deep-goal", "outcome": "aostar",
      "when": [
        { "feature": "sensing",          "op": "==", "value": 1 },
        { "feature": "goal_modal_depth", "op": ">=", "value": 2 },
        { "feature": "designated",       "op": "<=", "value": 32 }
      ] },
    { "name": "default", "outcome": "gbfs" }
  ]
}
```

A conjunction is one rule; a disjunction is two rules with the same `outcome`.
Operators are `<=`, `<`, `>=`, `>`, `==`, `!=`. Values are numbers — booleans
are `0` and `1`.

A file may supply `strategy`, `heuristic`, or both; whichever section is
omitted keeps its built-in rules.

### Features

Every quantity a condition can test, all read from the task before search
starts:

| Feature | Meaning |
|---|---|
| `sensing` | 1 if any action has more than one designated event |
| `max_designated_events` | max \|E_d\| over all actions |
| `worlds` | \|W\| in the initial state |
| `designated` | \|W\*\| in the initial state |
| `actions` | number of ground actions |
| `agents`, `atoms` | \|Ag\|, \|P\| |
| `goal_modal_depth` | deepest nesting of `[i]` / `C_G` / `Kw` in the goal |
| `goal_kw_only` | 1 if every top-level goal conjunct is a `Kw` formula |
| `goal_has_atom_conjunct` | 1 if the goal has a classical (non-modal) conjunct |
| `kd45` | 1 for a KD45 frame, 0 for S5 |
| `partial_obs` | 1 if any action has heterogeneous observability |
| `goal_unsat_init` | top-level goal conjuncts false in the initial state |

Outcomes must be `gbfs`, `ehc`, `aostar`, or `replan` for strategy, and `ug`, `ed`, `ks`,
`wc`, `rpg`, `radd`, `kadd`, `kff` for heuristic.

### Validation

A policy file is validated at load and the planner refuses to start if it does
not check out — an unknown feature or outcome, an unknown operator, a rule
shadowed by an earlier unconditional one, or a list with no terminal default.
Planning under a policy other than the one you asked for is worse than not
starting, so there is no fallback to the built-in table.

```
Error in selection policy: strategy rule 'typo' tests unknown feature 'wrlds';
expected one of: sensing max_designated_events worlds designated actions ...
```

### Seeing what fired

```sh
build/epistemic_planner --task benchmarks/coin4/problem_4.json \
                        --plan /dev/null --explain
```

```
[main] Features: sensing=1 max_designated_events=2 worlds=2 designated=1 ...
[main] Heuristic: knowledge-spread (auto, rule 'kw-only-goal')
[main] Strategy: AO* (auto, rule 'sensing-small-designated')
```

The thresholds in the built-in rules are tuned to the 15-instance suite in
`benchmarks/`. They are a starting point, not a claim about epistemic planning
tasks in general — retuning them for a different task distribution is the
reason the table is data.
