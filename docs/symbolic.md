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

Against deep on deep's own gossip formulation (private tells, mA*), translated
with `ma2json.py`, plans validated by deep's plan execution: n = 5, deep
HFS+C_PG 11.4 s, ours 0.04 s; n = 6, deep 226 s (portfolio: no plan), ours
0.2 s (`exploration/symbolic/deepcmp/`).

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

### Negative results

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
