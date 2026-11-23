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

## Next steps (in order)

1. Contraction speed: skip the E fixpoint when the update provably preserves
   valuation-injectivity; complement edges; better cache; GC (needed for search).
2. Search integration: hybrid nodes (explicit below a world threshold, symbolic
   above; conversions exist in both directions), duplicate detection as above,
   symmetry Γ_s by variable renaming + node equality when H = ∅.
3. Heuristics: kadd's "holds at some designated world" is ∃-quantification;
   h^Δ's possibility pairs need a symbolic formulation.
