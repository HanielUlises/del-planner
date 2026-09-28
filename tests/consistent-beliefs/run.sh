#!/usr/bin/env bash
#
# The cases --consistent-beliefs exists for.
#
# `two-sites.json` asks for six conjunctions, three about a north site and
# three about a south one. Without the option the planner returns, and the
# validator accepts, a policy none of whose actions touch the south site: a
# private announcement leaves the south agent with no accessible world, and
# every formula about what it believes then holds. With the option the policy
# has to go there.
#
# `two-sites-south-only.json` is the same task with the north conjunctions
# removed. It always had an honest answer, and it is here so that a check which
# refuses too much is caught as well.
#
# `coin4`'s reference plan collapses B's beliefs with A's announcement. The
# goal also has an honest plan, in which B and C look for themselves.
#
#   tests/consistent-beliefs/run.sh [path to epistemic_planner]

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT=$HERE/../..
PLANNER=${1:-$ROOT/build/epistemic_planner}
PLAN=$(mktemp)
LOG=$(mktemp)
trap 'rm -f "$PLAN" "$LOG"' EXIT

fail=0

report() {
  if [[ $1 -eq 0 ]]; then
    echo "ok   $2"
  else
    echo "FAIL $2"
    fail=1
  fi
}

plan() {
  : > "$PLAN"
  "$PLANNER" --task "$1" --plan "$PLAN" --consistent-beliefs --timeout 120 "${@:2}" > "$LOG" 2>&1
}

validated() {
  [[ -s "$PLAN" ]] && grep -q '\[validator\] OK' "$LOG"
}

# 1. The policy for both sites acts on the south one.
plan "$HERE/two-sites.json" --strategy aostar --heuristic ks
south=$(grep -oE '"(goto|scan)-south[a-z-]*_[a-z]+"|"relay-south-[a-z]+_[a-z]+_[a-z]+"' "$PLAN" | wc -l)
report $(validated && [[ $south -gt 0 ]] && echo 0 || echo 1) \
  "the policy for both sites acts on the south one ($south actions)"

# 2. The south half on its own still has its answer.
plan "$HERE/two-sites-south-only.json" --strategy aostar --heuristic ks
report $(validated && echo 0 || echo 1) "the south half alone is still solved"

# 3. coin4 is solved, and the validator, which runs under the same option,
#    found no agent believing a contradiction along the way.
plan "$ROOT/benchmarks/coin4/problem_4.json"
report $(validated && echo 0 || echo 1) "coin4 is solved with every belief consistent"

exit $fail
