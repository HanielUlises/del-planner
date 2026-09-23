# What the frame guard cost, 22 September 2026

Both runs are the IPC 2026 epistemic benchmarks, 15 s per instance, grounding
capped at 90 s and 8 GB. `2026-09-22-before.csv` is the planner at 3203684,
`2026-09-22-after.csv` is the same planner with the frame guard.

```
100 problems in both runs
  solved before: 40
  solved after:  40

no problem changed status
expansions identical on all 40 solved by both
```

Of the hundred instances, 46 ground inside the budget — 28 of them S5, which is
where the guard applies at all — and of those 46, forty were solved by both runs
and six by neither within 15 s. Validator verdicts are identical.

So the guard fires on no benchmark instance. That is what it was written to do:
refuse a product that left its frame, which in a well-posed task never happens,
and which in the task that prompted it happened at the second private
announcement.

The measurement's own limit is the grounding budget rather than the planner.
The whole hard tier and most of the intermediate one are missing because plank
wants more memory on this machine than it has — gossip at thirteen agents was
past two gigabytes and climbing when it was stopped. A machine with more memory
should run this again before the guard is trusted on hard instances.
