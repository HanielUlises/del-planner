# Symbolic DEL states: product update and contraction on BDDs

Research notes (2026-11-13). Code: `include/bdd.hpp`, `include/symdel.hpp`,
`src/symdel.cpp`, `tools/symcheck.cpp` (CMake target `symcheck`).

## Question

Can product update and bisimulation contraction run on *sets* of worlds
(BDDs) instead of explicit worlds, while remaining sound with respect to the
IεPC validator (plank)? Where would it change the planner's reach?

## Representation

State S = (H, θ, δ, Ω_1..Ω_n) over atoms P and hidden slots H.

- θ(x): worlds, one satisfying assignment of (x_P, x_H) per world
- δ(x): designated worlds
- Ω_i(x, y): R_i, restricted to θ × θ

Denotation ⟦S⟧: the explicit model on the assignments of θ, valuation = x_P.
Three copies (x, y, u) of every variable; u only inside the bisimulation
fixpoint. Order: hidden slots on top, then atoms with x/y/u interleaved
(hidden-on-top is 60–100× faster than hidden-at-bottom on intermediate gossip).

## Results (theory)

**Lemma 1 (model checking).** sat_S(φ) is the extension of φ in ⟦S⟧.
[i]φ = θ ∧ ¬∃y.(Ω_i(x,y) ∧ ¬φ(y)); Kw_i φ = [i]φ ∨ [i]¬φ;
C_G φ = νY. ⋀_{i∈G} [i](φ ∧ Y).
*Consequence:* plank's per-state observability choice (first type whose
condition holds at every designated world) and its applicability test
(δ ⊆ ⋁_{e∈E_d} sat(pre e)) are computed exactly.

**Lemma 2 (update).** ⟦update(S, a)⟧ ≅ ⟦S⟧ ⊗ a.
Construction: ⌈log₂|E|⌉ fresh event slots z; each atom p changed by some event
moves to a fresh slot o_p (its pre-update value); then
θ' = θ[o] ∧ ⋁_e (z=e ∧ pre_e[o]) ∧ ⋀_p (x_p ↔ ⋁_e (z=e ∧ ((o_p ∨ T_{e,p}) ∧ ¬F_{e,p})))
δ' = δ[o] ∧ θ' ∧ ⋁_{e∈E_d} z=e
Ω'_i = Ω_i[o] ∧ Q_{t_i(s)}(z, z') ∧ θ' ∧ θ'(y).
The isomorphism maps (w, e) to (w's variables with changed atoms in o, z = e,
new values of changed atoms). "Sets precede resets" as in the planner/plank.

**Lemma 3 (contraction).** ⟦contract(S)⟧ ≅ the bisimulation quotient of the part
of ⟦S⟧ reachable from δ.
1. reach = μR. δ ∨ post(R).
2. E = νE. E_0 ∧ ⋀_i (Forth_i ∧ Forth_i^T), E_0 = agreement on x_P and on δ,
   Forth_i(x,y) = ∀u.(Ω_i(x,u) → ∃u'.(Ω_i(y,u') ∧ E(u,u'))). E is maximal
   bisimilarity (classical relational fixpoint).
3. rep = least hidden assignment in each class (bisimilar worlds agree on x_P).
   Quotient edges: rep(x) ∧ rep(y) ∧ ∃u.(Ω_i(x,u) ∧ E(u,y)).
4. Hidden slots determined by the other variables on θ are projected away
   (injective projection ⇒ isomorphism).
5. The N distinct hidden vectors are re-encoded onto ⌈log₂N⌉ fresh slots
   (injective ⇒ isomorphism), then renamed to slots 0..m−1.

**Theorem (soundness w.r.t. plank).** A linear plan is accepted by the symbolic
replay iff plank accepts it. Proof: by induction ⟦S_k⟧ is bisimilar to plank's
s_k (base: S_0 encodes s_0; step: update preserves bisimilarity — paper §3 —
then Lemma 2 and Lemma 3). Applicability, observability and the goal are
formulas over designated worlds, hence bisimulation invariant (Lemma 1).
The same argument covers policies (branches restrict δ).

**Corollary.** The symbolic state space is the explicit planner's canonical
state space; the completeness and orbit-pruning theorems transfer provided
duplicate detection is exact.

**Canonicity (separate from soundness).** If contraction leaves H = ∅
(valuation-injective quotient), then (θ, δ, Ω) is a canonical form: bisimilar
states have identical BDDs, so state identity is node equality in one manager.
With H ≠ ∅ the hidden encoding depends on history: duplicate detection needs a
hash of an invariant (e.g. ∃H.δ, ∃H.(δ ∧ sat φ) for goal subformulas) plus an
exact symbolic bisimilarity check on the disjoint union (same fixpoint).
A canonical hidden encoding is open.

## Results (measurements)

Differential test (`symcheck`): every validated linear plan of the
`aletheia-hd` run (58 tasks, 4,015 states) replayed with both engines in one
process. At every step: applicability, |W|, |W_d| agree; for |W| ≤ 3000,
κ(enumerate(S)) == κ(s) — exact equality of canonical forms. **58/58 tasks,
3,765 states κ-equal, 0 discrepancies; goal agrees on all.**

BDD size vs explicit (contracted states along plans):

| family | max |W| | explicit table | BDD nodes | hidden bits |
|---|---|---|---|---|
| basic gossip, n = 3..11 | 2^n | O(n·2^n) (20,480 at n=10) | 3n+c (109 at n=11) | 0 |
| active muddy child 12 | 4,095 | 98,280 | ~440 | 0 |
| hard gossip (deceived) 12 | 40,961 | — | 5,656 | 5 |
| intermediate gossip 11 | 41,617 | — | 6,092 | 7 |
| blocks world, spy-linear, cn | 1–64 | 2–1,152 | 125–7,955 | 0 |

Time (C++, own BDD package, static order, no GC; t_exp = planner's
product_update + bisim_contract only):

- small models (blocks world: 1 world, ~1,700 atoms): symbolic 100–2000× slower
  → hybrid representation required.
- gossip ≤ 2,048 worlds: symbolic 2–5× slower; explicit is extremely fast here.
- crossover at a few thousand worlds: AMC-12 0.12 s vs 0.17 s; hard gos-12-imp
  0.58 s vs 0.42 s; basic gossip grows ~1.4× per agent symbolically vs 2× explicit.
- intermediate gos-11-all: 20.7 s vs 2.2 s, dominated by contraction (the E
  fixpoint over three variable copies).

## Where it changes the game — and where it does not

- **Does not:** every blocks-world timeout is a 1-world, 1-agent task with
  ~1,700 atoms and ~3,300 actions: classical search, not representation.
- **Does:** families whose explicit models are exponential — gossip, muddy
  children, deception. The explicit table grows as n·2^n; the BDDs grow linearly
  (basic) or stay in the low thousands (hard/intermediate, 5–7 hidden bits).
- **The initial state was the wall, and the theory loader removes it.**
  plank's grounder builds a finitary-S5 initial state by enumerating every
  valuation and comparing every pair of worlds (O(|W|²·|Ag|)); `tools/ground`
  already writes large explicit states compactly (gos-13-all), but the
  grounding itself does not scale: generated gossip with 12 agents takes 17.5 s
  and 0.9 GB, and 14 or 16 agents exceed 16 GB. See "Symbolic initial states".
- **Validation caveat:** plans for tasks plank cannot ground cannot be counted as
  validated in the competition sense; the theorem + differential evidence is
  what we can state.

## Symbolic initial states (finitary S5 theories)

`ground --theory` grounds everything except the initial state and writes the
theory plank's `finitary_s5_theory_grounder` builds it from (type-1: designated;
type-2: worlds, including C(K_i φ); type-3: per-agent agreement formulas; fixed
fact atoms). The parser reads it into `PlanningTask::theory` and materialises
`init` by grouping worlds by their type-3 truth vectors (O(|W|) per agent instead
of plank's pairwise comparison) when there are at most 2^24 candidate valuations
(`ALETHEIA_THEORY_MAX_FREE`). `Engine::from_theory` builds

  θ = fixed ∧ ⋀ type-2,  δ = θ ∧ ⋀ type-1,
  Ω_i = θ(x) ∧ θ(y) ∧ ⋀_{φ ∈ type-3(i)} (φ(x) ↔ φ(y)),

which is plank's construction read as a formula (R_i is defined by agreement,
so it is reflexive and symmetric as in plank).

Validation (`symcheck theory.json plan --plank-init explicit.json`): on all 106
finitary-S5 tasks of the benchmark set (IεPC and local), κ(plank's s0) ==
κ(materialisation) == κ(enumerate(S0)); the 47 validated plans among them replay
correctly from the theory (980 states κ-equal).

Scaling, generated gossip (plan: all tell_i_j, n(n−1) actions; goal checked):

| n | worlds | plank ground | ground --theory | symbolic replay | explicit replay |
|---|---|---|---|---|---|
| 10 | 1,024 | 0.5 s | 0.01 s | 0.013 s | 0.008 s |
| 12 | 4,096 | 17.5 s, 0.9 GB | 0.02 s | 0.025 s | 0.052 s |
| 14 | 16,384 | > 16 GB | 0.02 s | 0.043 s | 0.34 s |
| 16 | 65,536 | > 16 GB | 0.02 s | 0.09 s | 2.3 s |
| 20 | 10^6 | — | 0.04 s | 0.18 s | 67 s, 1.3 GB |
| 32 | 4.3·10^9 | — | 0.16 s | 0.96 s | — |
| 64 | 1.8·10^19 | — | 0.95 s | 16.9 s, 0.44 GB | — |

Up to n = 20 both engines agree on |W| and |W_d| at every step and on the
goal; plank's validator accepts the plan for n = 10 and gives no verdict for
n = 12 within 900 s (it grounds, then validates on the explicit model), so
beyond n = 10 the theorem and the differential test are the evidence.

## Search on symbolic states (2026-11-26)

Code: `include/symsearch.hpp`, `src/symsearch.cpp` (hybrid GBFS),
`include/symheur.hpp`, `src/symheur.cpp` (heuristics on BDDs), additions to
`include/bdd.hpp` and `src/symdel.cpp`; flag `--symbolic` (docs/usage.md);
tests `tests/symbolic/symbolic_test.cpp` (CMake `-DALETHEIA_TESTS=ON`, target
`symbolic_test`).

### Engine changes

- **Garbage collection.** `Manager::gc(roots)` is a non-moving mark-and-sweep:
  unreachable nodes go to a free list that `mk` reuses, so a Ref the caller
  keeps as a root stays valid. The unique table is rehashed and the computed
  cache cleared. The search collects between expansions once the nodes
  created since the last collection outnumber those it kept; roots are the
  open symbolic nodes and the H ≠ ∅ states kept for duplicate checks.
- **Cache.** The computed cache starts at 2^16 entries and grows with the
  unique table up to 2^23 (it was a fixed 2^22, 80 MB per manager). The engine
  also keeps `[i]φ` across states, keyed by (Ω_i, ext(φ), θ): a successor
  shares most relations with its parent, so most modal subformulas are
  lookups (goal tests and kadd's fact flags 2–4× faster on gossip).
- **Skipping the fixpoint.** Contraction now projects away the hidden slots
  that the other variables determine *before* the bisimulation fixpoint. A
  slot survives only if two worlds differ in it alone, so H = ∅ afterwards iff
  the model is valuation-injective. Bisimilar worlds share their valuation,
  so then the quotient is the model itself and the fixpoint is skipped. This
  is a test on the state, not on the action, and it subsumes "the update
  provably preserves valuation-injectivity" (public announcements, gossip
  calls, sensing of observed atoms: Theorem obs-closure of the notes).
- **Fixpoint renamings.** E is symmetric, so E(u,x) = E(x,u) is an
  order-preserving relabelling, and x ↔ y transposition is `swap_adjacent`,
  one linear pass (copies are interleaved per variable). Neither changed
  the fixpoint's time measurably. On intermediate gos-11-all the replay is
  9.2 s symbolic vs 1.2 s explicit (20.7 s before), 92% of it in the fixpoint's
  relational products. Computing the transposed condition directly, with two
  more products per agent, was slower (11.1 s) and was reverted.
- **Not done: complement edges.** Operation counts on that replay: negation is
  10% of recursive calls (relational products 22%, renamings 50%), so
  constant-time negation would not change the picture. What would is a
  different fixpoint (signature-based partition refinement over block
  variables, as in symbolic bisimulation minimisation) instead of a relation
  over two copies.

### Identity of states

- H = ∅: (θ, δ, Ω_i) is canonical (any bisimulation between two
  valuation-injective, δ-reachable models is the identity on valuations,
  the argument of Theorem obs-canonical without the observational
  restriction on relations). Duplicate detection is a 128-bit structural
  digest of the four BDDs, computed bottom-up over nodes, so closed states
  need not stay alive.
- H ≠ ∅: equal digests still mean equal states (same encoding). Otherwise
  candidates are bucketed by an invariant: digests of ∃H.θ, ∃H.δ, ∃H.Ω_i and
  counts (worlds, designated worlds, edges per agent, worlds per atom). Within
  a bucket the test is exact: the maximal bisimulation of the disjoint union
  (a tag slot, the same fixpoint as contraction) must match every world of
  each side. Projections alone put all 42 successors of intermediate
  gos-07's first expansion into one bucket (861 checks); with counts no check
  ran on that task.
- Symmetry: a swap fixes a state with H = ∅ iff the renamed BDDs are the same
  nodes; with H ≠ ∅, iff the renamed state has the same invariant and is
  bisimilar. With both, the symbolic search generates exactly what the
  explicit one does (intermediate gos-07: 207 successors; without the H ≠ ∅
  test 862).

### Hybrid representation

A contracted state is symbolic iff it has more than T worlds (default 4 096)
and is valuation-injective, or more than T_H worlds (default 2^20). Both are
bisimulation invariants, so one state always lands on the same side and
duplicate detection never compares across representations. Crossing over:
explicit → symbolic by encoding the contracted model (balanced disjunction of
world cubes); symbolic → explicit by enumerating θ, one enumeration per
distinct successor-set BDD, then `bisim_contract` for the canonical numbering
(26 s → 0.035 s for the two conversions on gos-13-all compared with the
first version). The second condition exists because H ≠ ∅ states are where the
symbolic engine is slow (intermediate gos-09 with T alone: > 300 s, against
1 s explicit).

### Heuristics

- kadd/kff read a state only through "fact φ holds at some designated world",
  δ ∧ sat(φ) ≠ ⊥. `KnowledgeRelaxationHeuristic::estimate(flags)` takes those
  flags from either representation, so values are equal on both (tested on
  502 states).
- h^Δ on BDDs: possibility sets T, F over x; the possible cuts of agent i as
  one relation, cut_i(x,y) = Ω_i ∧ ⋁_{a,e} genT_e(x) ∧ ⋁_t ⋀_{f∈Q_t(e)}
  genF_f(y); boxes and C_G as relational products; the layer update of
  `hdelta.cpp` otherwise unchanged. Refinements R1–R4 are omitted. Each one
  only removes possible cuts or eliminations, so the symbolic sets are
  supersets: symbolic h^Δ ≤ explicit h^Δ, still admissible, and ∞ still a
  proof. Measured equal on all 161 test states. It is the root dead-end test
  for symbolic roots (gos-13-all: explicit root test 13 s, symbolic < 0.1 s).


## Contraction and symmetry when H ≠ ∅ (2026-11-27)

Branch `symbolic/sigref`. The H ≠ ∅ states (private and semi-private
announcements, deception) were where the symbolic engine lost: replaying
intermediate gos-11-all took 9.2 s symbolically against 1.2 s explicitly, 92%
in the bisimulation fixpoint.

### What the fixpoint was doing

In intermediate gossip, contraction never merges a world: 72 of 72
contractions on gos-09-all's plan and 110 of 110 on gos-11-all's return the
model unchanged (hard gos-10-imp: 8 of 9). The fixpoint's two rounds compute
E_1 and then confirm it; the cost is proving minimality. The BDDs involved
are small (θ ≈ 200 nodes, Ω_i ≈ 600, E ≤ 9 000); the time is in relational
products over three copies, whose recursion grows with the product of the
operand sizes.

### Depth-one certificate

Pairs of distinct worlds with equal valuation and designation are candidates
for merging. Two of them are not bisimilar if, for some agent i, their sets of
pairs (successor valuation, successor designated?) differ. These sets are
S_i(x, y_P) = ∃y_H. Ω_i(x,y) ∧ [δ](y): the successors' hidden bits are
quantified first, so the comparison is between functions of a world and a
valuation (x, y and the atoms of u), not between worlds of three copies. If no
candidate pair survives, the maximal bisimulation is the identity and the
model is its own quotient (`Engine::depth_one_pairs`, `contract`). Otherwise
the fixpoint starts from identity ∪ (surviving pairs). That is sound, since
this relation contains the maximal bisimulation.

Replays (symcheck, same machine, loaded): gos-07 1.79 → 0.45 s, gos-09
9.2 → 1.9 s, gos-11 37 → 6.1 s; all contractions certified, canonical forms
still equal to the explicit ones wherever checked.

### Symmetry for H ≠ ∅

With contraction cheap, the swap test dominated search (121 of 146 s on
intermediate gos-11-all), because each test ran a full bisimulation fixpoint
on a disjoint union. All 371 tests on gos-09 were positive: proving real
symmetries was the cost. Three changes:

1. **Inheritance.** If σ fixes P and σ(a) = a, then σ fixes P ⊗ a:
   σ(P ⊗ a) ≅ σ(P) ⊗ σ(a) = σ(P) ⊗ a ≅ P ⊗ a, since renaming commutes with
   product update and update preserves bisimilarity. Nodes carry the swaps
   proved for them; successors inherit those that fix the action, untested,
   and the stabiliser applies them before testing anything else
   (`stabiliser_by(..., known)`). Half of the swaps on intermediate gossip.
2. **Count filter.** Edges per agent and worlds per atom, once per state.
3. **Hidden relabelling.** Depth-one candidates between σ(s) and s, read on
   the hidden bits alone, give a relation π on hidden values; if it is a
   bijection and renaming σ(s)'s hidden bits by π yields s's four BDDs, the
   two are isomorphic. Exact when it succeeds; otherwise the invariant and the
   union fixpoint decide as before. It succeeded on every positive test
   (174/174 gos-09, 338/338 gos-11), so no union fixpoint ran. It also
   short-cuts `equivalent()` for duplicate detection.

### Results

All-symbolic GBFS+kadd, one thread, against explicit GBFS+kadd, one thread
(intermediate gossip, n agents, initial state as a finitary S5 theory; n = 11
is IεPC gos-11-all):

| family | n | max worlds | explicit | symbolic |
|---|---|---|---|---|
| intermediate | 11 (IεPC) | 42 637 | 35.9 s, 1.0 GB | 13.6 s, 170 MB |
| intermediate | 13 | 233 801 | 340 s, 9.2 GB | 59.7 s, 335 MB |
| intermediate | 15 | 1.2·10^6 | out of memory (14 GB) | 114 s, 337 MB |
| intermediate | 17 | 6.3·10^6 | — | 292 s, 502 MB |
| hard (KD45, lies) | 12 (IεPC) | 40 961 | 4.4 s, 251 MB | 1.6 s, 170 MB |
| hard | 14 | 192 513 | 45 s, 1.7 GB | 2.4 s, 172 MB |
| hard | 16 | 884 737 | 250 s, 11.4 GB | 4.8 s, 173 MB |
| hard | 20 | 1.8·10^7 | — | 9.5 s, 340 MB |
| hard | 24 | 3.4·10^8 | — | 21 s, 355 MB |
| hard | 32 | 1.2·10^11 | — | 77 s, 507 MB |
| hard, all deceived | 12 (IεPC) | | 0.50 s | 0.14 s |
| hard, all deceived | 14 | | 4.0 s | 0.21 s |
| hard, all deceived | 16 | | 35 s, 207 MB | 0.27 s, 29 MB |
| hard, all deceived | 20 | | 1 129 s, 3.7 GB | 0.85 s, 52 MB |

Instances beyond the competition's are generated from the IεPC problem files
(agent lists extended; `~/Documents/ICAPS-2027/exploration/symbolic/hidden/`)
and grounded with `ground --theory`. Plans up to intermediate n = 15 and
hard n = 16 replay correctly in the explicit engine (n = 15: 274 s explicit,
19 s symbolic); all of them replay in the
symbolic one.

Against deep on deep's own gossip formulation (private tells, mA*; goal *one*:
agent 0 believes every secret, deep's form; *all*: every agent does),
translated with `ma2json.py`. Limits: 600 s, 12 GB (RLIMIT_AS), one thread.
Ours is GBFS+kadd, explicit or `--symbolic` with the per-run choice
(all-symbolic from n = 10, where the initial state has 2^10 worlds; below
that no state crosses the thresholds and the run is explicit), with kadd
splitting the goal as described in the next section. deep is not run beyond
n = 8. Times are wall clock including loading (n = 16: 65 536 initial worlds
in JSON, about 60 s of the symbolic "one" run; its search takes 0.5 s).

| n | goal | deep portfolio | deep HFS+C_PG | ours explicit | ours `--symbolic` |
|---|---|---|---|---|---|
| 5 | one | 23.9 s, 2.3 GB, 14 steps | 11.4 s, 314 MB, 14 steps | 0.004 s, 4 steps | 0.005 s, 4 steps |
| 5 | all | 34.7 s, 3.1 GB | 17.0 s, 420 MB | 0.02 s | 0.02 s |
| 6 | one | out of memory | 184 s, 4.6 GB, 22 steps | 0.005 s, 5 steps | 0.006 s, 5 steps |
| 6 | all | out of memory | 205 s, 5.2 GB | 0.07 s | 0.07 s |
| 7 | one | out of memory | timeout | 0.02 s | 0.01 s |
| 7 | all | out of memory | out of memory | 0.31 s | 0.36 s |
| 8 | one | out of memory | out of memory | 0.02 s | 0.02 s |
| 8 | all | out of memory | out of memory | 1.4 s, 157 MB | 1.6 s, 158 MB |
| 10 | one | — | — | 0.10 s | 0.07 s |
| 10 | all | — | — | 30 s, 2.6 GB | 19 s, 171 MB |
| 12 | one | — | — | 0.78 s | 0.30 s |
| 12 | all | — | — | out of memory | 61 s, 338 MB |
| 14 | one | — | — | 9.2 s, 463 MB | 3.8 s, 104 MB |
| 14 | all | — | — | out of memory | 215 s, 372 MB |
| 16 | one | — | — | 121 s, 2.8 GB | 63 s, 212 MB |
| 16 | all | — | — | out of memory | 561 s, 576 MB |

Beyond n = 16, `ma2json.py --theory` writes the initial state as a finitary S5
theory (type 1 the `initially` literals, type 2 propositional common
knowledge, type 3 the C(B(i,f) | B(i,¬f)) statements), which is never
materialised. At n = 10 it gives the same plans and expansions as the explicit
file. Symbolic, *one*: n = 20 0.5 s, 24 0.9 s, 32 3.1 s, 48 22 s, 64 75 s
(840 MB; up to 6·10^20 worlds), all with n − 1 tells; *all* times out from
n = 20. These plans are replayed in the symbolic engine only
(`symcheck --no-explicit`; n = 20 also explicitly): `symcheck` reports
`goal_explicit` equal to the symbolic result when the explicit engine does
not run, so `fixup.py`'s `valid_explicit_replay` is not an explicit check
there (n = 24: the explicit replay ran out of memory, recorded as false).

Plans: *one* n − 1 tells, *all* n(n − 1). Up to n = 8 they are validated by
deep's plan execution (`deep -e`). From n = 10, `deep -e` itself throws
`bad_alloc` at 12 GB (n = 10: after 348 s), so those plans are replayed in our
explicit engine instead (`symcheck --enum 0`; all reach the goal). With the
kadd of before the split, *one* took 251 s explicit / 60 s symbolic at n = 10
(74 steps), symbolic 271 s at n = 12, and timed out at n = 14 in both
representations. Data: `exploration/symbolic/deepcmp/results_fixed.jsonl`
(old kadd, deep) and `results_sym3.jsonl` (split kadd); `run.py`, then
`fixup.py`.

### kadd: boxes over conjunctions (2026-11-27)

deep's goal is one formula, [0](s_0 ∧ … ∧ s_n). kadd made it a single fact,
which its relaxation never reaches, so every state had h = 1000 (the
dead-conjunct penalty) until the goal held. GBFS ran on helpful actions and
tie-breaking alone, and the default `--dead-ends suspect` ran h^Δ on every
node (77–85 % of the time, never pruning). kadd now splits
[i](φ ∧ ψ) into [i]φ ∧ [i]ψ, valid in every Kripke model, both among goal
conjuncts and inside compiled formulas (test
`kadd_splits_boxes_over_conjunctions`, fixture `deep-gossip-05-one.json`).

Suite (GBFS+kadd, one thread, 300 s, dead ends off; `gbfs-kadd-exp3` /
`auto3` against `exp2` / `auto`): 14 of 142 tasks contain such a box (coin
in the box, intermediate and hard blocks world). Valid plans 124 → 122 in
both configurations:

- intermediate bw-4-05-2: 12.9 s, 2 892 steps → 0.03 s, 3 steps;
  bw-3-06-2: 28 → 21 steps; hard bw-3-5-2-clumsy: 169 → 87 steps; coin 4:
  8 → 6 steps (coin 2: 3 → 4);
- lost: intermediate bw-4-07-3 (165 s → timeout) and hard bw-4-5-2-clumsy
  (73 s → timeout; the helpful-action search runs dry and the retry without
  pruning does not finish).

Splitting only the goal gives the same results on these tasks. Both lost
goals require a false belief: bw-4-5-2-clumsy wants [C]on(b3, b1) and
[D](on(b3, b4) ∧ …), bw-4-07-3 wants [D](on(b5, b6) ∧ on(b5, b7)). The
delete-free relaxation cannot see that the conjuncts conflict; the old kadd
ignored one of them (it was a dead fact) and stumbled on a plan. Neither
`--no-helpful` nor a reserve queue for the held-back non-preferred actions
(instead of restarting) recovers them in 300 s. Every plan returned is valid.

`--dead-ends suspect` ran h^Δ on nodes with kadd ≥ 1000, the per-conjunct
penalty; with many conjuncts (gossip *all*, n(n − 1)) a finite estimate
passes it. kadd/kff now record the dead conjuncts of their last estimate
(`last_dead_conjuncts`), which the suspect test reads; other heuristics keep
the threshold.

Fixed (2026-11-27): symbolic h^Δ on gossip *all* n = 64 (`--dead-ends all`,
or suspect before the fix above) threw `bdd: order violated in mk` after
about 400 s. Each `permute()`/`swap_adjacent()` call caches under its own op
code, kPermBase + 2·epoch (+1 if monotone), in 32 bits. After about 2^31
calls (one h^Δ `build_cuts` renames ≈ 64 agents × 4 032 actions × 3 events
functions) the code wrapped onto the fixed codes: the renaming x_p → y_p was
cached as kNeg, so neg(x_p) returned y_p and the atom's false-set
θ ∧ ¬x_p contained a y variable. The manager now drops the cache and restarts
the epoch before the code can overflow (`next_perm_op`; test
`permutation_op_codes_never_collide`, which failed before). Runs below 2^31
renamings were unaffected; every plan reported above was also replayed in a
fresh process, whose epoch stays far from the wrap.

The symbolic per-node h^Δ check now stops at the search deadline
(`h_delta_until`; a check cut short prunes nothing). Before, one check could
outlast `--timeout` by an hour (gossip *all*, 64 agents, `--dead-ends all`,
1 500 s limit); now a 120 s limit ends after 153 s, mostly loading. The
explicit `hdelta::prunes` still runs without a deadline.

Also tried: evaluating fact formulas only on the designated worlds (pointed
model checking, [i]ψ on U needs ψ only on R_i(U)). The facts are equal but it
was not faster (n = 16: 0.06 s against 0.02 s for the whole search's
heuristic); reverted.

Hard gos-12-imp-deceived: 2.5 s symbolic against 4.4 s explicit (each
includes about 1.8 s of loading). Intermediate gos-09-all, the largest size
where explicit still wins: 3.7 s against 2.6 s.

### Representation policy

Converting a large *non-injective* explicit state to BDDs numbers its worlds
arbitrarily (rank within a valuation). The resulting BDDs are large, and
the run never recovers (intermediate gos-09 with a single threshold of 4 096:
> 600 s). States produced by symbolic updates have structured hidden bits
(event codes, old values). `--symbolic` therefore decides once per run: if
the contracted initial state has at least 1 024 worlds (or the theory is not
materialised), every state is symbolic; otherwise the hybrid of the previous
section. On the suite, tasks with ≥ 1 024 initial worlds were already faster
all-symbolic before these changes (geometric mean 0.71× explicit time over
10 tasks). Explicit thresholds (`--symbolic-threshold`,
`--symbolic-hidden-threshold`) disable the per-run choice.

### First improvement (2026-11-27)

On gossip GBFS is perfectly guided (expanded = plan length), yet every
expansion builds and contracts all applicable successors: 28 921
contractions for 240 expansions on deep's gossip *all*, n = 16, 337 of 500 s.
The depth-one certificate needs nearly every agent there (22 of 24 passes at
n = 12; a move-to-front agent order saved 3 %), so the lever is the number of
successors, not the certificate.

`--first-improvement` (implies `--symbolic`; `Options::first_improvement`):
a symbolic node stops at the first successor whose h is below its own and
goes back to the open list with `resume` = that action + 1, keeping its
state and proven swaps. Explicit nodes keep ordinary GBFS, so it is opt-in and
the hybrid tests are unchanged. Same plan lengths on every task below; plans
replayed in a fresh process.

| task | GBFS | first improvement |
|---|---|---|
| deep gossip *all* n = 12 | 61 s, 8 779 contractions | 3.4 s, 133 |
| deep gossip *all* n = 16 | 561 s | 51–84 s (search 36 s) |
| IεPC intermediate gos-15 | 114 s | 42 s |
| IεPC intermediate gos-17 | 292 s | 116 s |
| IεPC intermediate gos-19 (3.1·10^7 worlds) | — | 252 s |
| IεPC intermediate gos-21 (1.5·10^8 worlds) | — | 485 s |
| IεPC hard gos-24-imp | 21 s | 8.5 s |

Beyond n = 16 on deep's gossip (*every agent*, 1 800 s, n(n−1) tells; the
explicit replay runs out of memory from n = 20, so these plans were replayed
with `symcheck --no-explicit`, goal reached at every step):

| n | GBFS | first improvement | + two-copy certificate |
|---|---|---|---|
| 20 | — | 167 s, 513 MB | 131 s, 357 MB |
| 24 | — | 931 s, 807 MB (1.6·10^9 worlds) | 481 s, 379 MB |
| 28 | — | TO (516 of 756 expansions in 600 s) | 1 113 s, 545 MB (3.5·10^10 worlds) |
| 32 | — | TO | TO (949 of 992 expansions) |

gos-19/21 are generated like gos-13..17 (agents list extended, `ground
--theory`; regenerating gos-17 reproduces its file byte for byte) and replay
in the symbolic engine (342 and 420 steps).

### Two copies for the certificate (2026-11-27)

Sampled stacks of the n = 24 search: 26 of 32 samples in `contract`, 22 in
`depth_one_pairs`; contraction was 452 of 600 s. Every expansion there makes
one successor (first improvement), so the certificate is the cost per step.
Instrumented on the n = 16 replay, 7 331 agent passes for 240 certificates
(30–32 of 32 passes each: each agent separates about 12% of the pairs, so
no order empties D early), and the xor `S(x,u) ⊕ S(y,u)` over three copies
cost three times the conjunction with D.

Two changes, same relation D:

- Pairs of D share their valuation, so D is kept as D'(x_P, x_H, y_H) and the
  y_P slot holds the successor's valuation: `S(x, y_P)` against S with x_H
  renamed to y_H. Two copies of the atoms instead of three.
- `Manager::and_exists_xor(d, f, g, vars)` = ∃vars. d ∧ (f ⊕ g) in one
  recursion, so the xor is never built where d is false. The quantified
  suffix at a node is fixed by its level, so within one call the cache key is
  (d, f, g) under a fresh op code (`next_perm_op`).

Certificate time on the n = 16 replay 13.5 → 10.3 s (two copies) → 5.8 s
(fused); replay of n = 20 59.5 → 29.9 s. Checked against the old computation
(both run, relations compared for equality) on every contraction of the 18
hidden-gossip replays, including the hard ones where D is non-empty and the
fixpoint runs: no difference.

### Negative results

- **Certificate on per-world signatures** (number the sub-BDDs below the x
  levels, then test that no x_P, l has two hidden witnesses, exactly, by a
  pass computing ∃x_H and "at least two x_H"): over 600 s against 18.8 s on
  the n = 16 replay. The signatures need the atoms' copy z below every x
  variable, which breaks the x/y interleaving that keeps relations small; the
  same cause as signature refinement below. Also re-measured: one fused pass
  for all agents with the agent code below the copies (over 600 s against
  30 s on n = 20), as recorded below for gos-09.

- **Signature refinement** (blocks as numbers on bits below the x levels, as
  in BDD-based bisimulation minimisation) is correct but not faster here.
  With block numbers as the new hidden encoding, relations grew 15×
  (7 603 vs ≈ 500 nodes) because the old encoding's structure is lost. With
  representatives instead it is even on gos-05 and 2× slower on gos-09. E stays
  small in these models, so avoiding it buys nothing. Kept behind
  `SYM_SIGNATURE=1`.
- **Random parity fingerprints** of successor sets (parity quantification with
  random functions) almost never separated all pairs. Low-degree
  functions sum to 0 over subcubes of higher dimension, and knowledge sets are
  subcubes; with thousands of pairs, some pair collides on its few bits
  anyway. The exact comparison was as fast; removed.
- **One product for all agents** (agent code bits below the copies): 175 s
  instead of 1.4 s on gos-09's certificate; reverted.
- **Bug found by the sweep:** without `--symbolic`, a theory too large to
  materialise left an empty initial model, and GBFS reported the empty plan.
  The planner now refuses such a task.
