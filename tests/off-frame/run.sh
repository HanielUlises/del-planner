#!/usr/bin/env bash
#
# The regression the frame guard exists for.
#
# `two-sites.json` asks for six conjunctions, three about a north site and
# three about a south one. Before the guard the planner returned, and the
# validator accepted, a policy of eight actions none of which touched the south
# site: a chain of private announcements left one agent with no accessible
# world, every formula about what it knew held for want of anywhere to check,
# and half the goal was credited to a model that said nothing.
#
# `two-sites-south-only.json` is the same task with the north conjunctions
# removed. It always had an honest three-action answer, and it is here so that
# a guard which refuses too much is caught as well.
#
#   tests/off-frame/run.sh [path to epistemic_planner]

set -uo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PLANNER=${1:-$HERE/../../build/epistemic_planner}
PLAN=$(mktemp)
trap 'rm -f "$PLAN"' EXIT

fail=0

report() {
  if [[ $1 -eq 0 ]]; then
    echo "ok   $2"
  else
    echo "FAIL $2"
    fail=1
  fi
}

# 1. Both sites are surveyed, or nothing is returned. What must not happen is a
#    policy that is credited with the south conjunctions without going there.
"$PLANNER" --task "$HERE/two-sites.json" --plan "$PLAN" \
  --strategy aostar --heuristic ks --timeout 120 > /dev/null 2>&1

if [[ -s "$PLAN" ]]; then
  south=$(grep -oE '"(goto|scan)-south[a-z-]*_[a-z]+"|"relay-south-[a-z]+_[a-z]+_[a-z]+"' "$PLAN" | wc -l)
  report $([[ $south -gt 0 ]] && echo 0 || echo 1) \
    "the policy for both sites acts on the south one ($south actions)"
else
  report 0 "no policy was returned for both sites, which is not a false one"
fi

# 2. The south half on its own still has its answer.
"$PLANNER" --task "$HERE/two-sites-south-only.json" --plan "$PLAN" \
  --strategy aostar --heuristic ks --timeout 120 > /dev/null 2>&1
report $([[ -s "$PLAN" ]] && echo 0 || echo 1) \
  "the south half alone is still solved"

exit $fail
