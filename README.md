# Aletheia

**Epistemic planner** for the International Epistemic Planning Competition (IεPC 2026), Tracks Basic and Intermediate.  
Built at **UNAM–FI** (Artificial Intelligence Microsoft Lab) / **IPN–ESCOM**.

[![Release](https://github.com/HanielUlises/Aletheia/actions/workflows/release.yml/badge.svg)](https://github.com/HanielUlises/Aletheia/actions/workflows/release.yml)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![C++23](https://img.shields.io/badge/C%2B%2B-23-blue.svg)](https://en.cppreference.com/w/cpp/23)
[![ICAPS 2026](https://img.shields.io/badge/ICAPS-2026%20Workshop-orange.svg)](https://www.icaps-conference.org/)

This document describes the design. Building and running the planner is covered
in [docs/usage.md](docs/usage.md); measured results are in
[docs/evaluation.md](docs/evaluation.md).

---

## Abstract

Aletheia is a planner for Dynamic Epistemic Logic (DEL) planning tasks over $S5_n$ and $KD45_n$ frames. Its search space is not a set of propositional valuations but a set of *pointed Kripke models*, each of which must be updated, minimised and compared in full at every node. That makes the planner's performance a question of how a Kripke model is represented, how modal formulas are evaluated over it, and how two models are recognised as the same epistemic situation.

This document describes the design of the current implementation. Three decisions dominate it:

1. **Accessibility is a table of successor sets.**  
   Each agent maps every world to one of a model's distinct successor sets, stored once as a sorted world list. Epistemic models repeat successor sets heavily, so a model costs $O(\lvert Ag\rvert\cdot\lvert W\rvert)$ rather than $\lvert Ag\rvert\cdot\lvert W\rvert^2$ bits, and modal operators are decided once per set.

2. **Formulas are evaluated as extensions, not pointwise.**  
   For each subformula the planner computes the set of worlds at which it holds, bottom-up over the whole model, memoised on hash-consed formula identity. Common knowledge becomes a single greatest fixpoint instead of one graph search per world.

3. **Contraction produces a canonical form.**  
   Bisimulation contraction assigns world indices in an order determined by the model's structure alone, so bisimilar states serialise identically and duplicate detection reduces to comparing 128-bit fingerprints.

Against the previous implementation on the same benchmark set, these changes reduce the largest solved instance from **151 s and 21 GB** of resident memory to **40 ms and 7.3 MB**, and improve AND-OR iteration throughput on an unsolved instance by a factor of **19.8×**.

---

## 1. The problem

Classical planning assumes a fully observable world: the agent knows exactly which propositions hold. Epistemic planning drops that assumption. The agent operates over a Kripke structure — a set of possible worlds with accessibility relations encoding what each agent considers possible — and pursues goals that may be intrinsically modal: not “the door is open” but “agent $A$ *knows* the door is open”, or “neither $A$ nor $B$ *knows whether* the coin is heads”.

An epistemic planning task is a tuple

```math
\Pi = \langle \mathcal{M}_0,\ \mathcal{A},\ \varphi_g \rangle
```

where $\mathcal{M}_0$ is the initial multi-pointed Kripke model, $\mathcal{A}$ a set of event models, and $\varphi_g$ a modal goal formula. A solution is a sequence (or, under partial observability, a branching policy) of actions whose product updates carry $\mathcal{M}_0$ to a model satisfying $\varphi_g$.

The cost structure differs sharply from classical planning. A classical state is a bit vector and successor generation is a set difference. An epistemic state is a labelled graph; successor generation is a graph product that can square the state's size, and every heuristic evaluation is a model-checking problem over that graph. The planner therefore spends its time in three places — the product update, bisimulation contraction, and modal model checking — and the design below is organised around those three.

---

## 2. Preliminaries

### 2.1 Epistemic states

Fix a finite set of atoms $P$ and agents $Ag$. An **epistemic state** is a multi-pointed Kripke model

```math
\mathcal{M} = (W,\ \{R_i\}_{i \in Ag},\ V,\ W^*)
```

with $W$ a finite set of worlds, $R_i \subseteq W \times W$ agent $i$'s accessibility relation, $V : W \to 2^P$ a valuation, and $\emptyset \neq W^* \subseteq W$ the *designated* worlds — those the planner considers actual.

The language is the modal fragment

```math
\varphi ::= \top \mid \bot \mid p \mid \neg\varphi \mid \varphi \wedge \varphi \mid \varphi \vee \varphi \mid [i]\varphi \mid C_G\varphi \mid \mathit{Kw}_i\varphi
```

with the standard semantics, $\mathit{Kw}_i\varphi \equiv [i]\varphi \vee [i]\neg\varphi$ (“$i$ knows whether $\varphi$”), and

```math
\mathcal{M} \models \varphi \quad\text{iff}\quad \mathcal{M}, w \models \varphi \ \text{ for every } w \in W^*.
```

Frames are $S5_n$ (knowledge) or $KD45_n$ (belief); the latter requires every $R_i$ to be serial, which the product update does not preserve and must therefore repair.

### 2.2 Product update

An action is an event model $\mathcal{E} = (E, \{R^E_i\}, \mathit{pre}, \mathit{post}, E_d)$. The product update is

```math
\begin{aligned}
W' &= \{ (w,e) \mid w \in W,\ e \in E,\ \mathcal{M},w \models \mathit{pre}(e) \} \\
R'_i &= \{ ((w,e),(v,f)) \mid (w,v) \in R_i \ \wedge\ (e,f) \in R^E_i \} \\
V'(w,e) &= \mathit{post}(e)\ \text{applied to}\ V(w) \\
{W'}^* &= \{ (w,e) \mid w \in W^*,\ e \in E_d \}
\end{aligned}
```

Postconditions are conditional: an atom flips at $(w,e)$ only if its guard holds at $w$ *in the pre-update model*. Observability is conditional too: each agent carries an ordered list of (guard, event relation) cases, and the first case whose guard holds at $w$ supplies $R^E_i$ there. This is what lets a single action be public for one agent, private for another, and conditional on the state for a third.

Without contraction, $|W|$ can double at every step. With it, the reachable state space is finite up to bisimilarity, and contraction is what makes the search terminate on the benchmark domains at all.

### 2.3 Bisimulation

Worlds $w, v$ of a multi-pointed model are **bisimilar** when

1. $V(w) = V(v)$,
2. $w \in W^* \iff v \in W^*$,
3. for every $i \in Ag$, every $R_i$-successor of $w$ has a bisimilar $R_i$-successor of $v$, and symmetrically.

Bisimilar worlds satisfy exactly the same formulas, so quotienting by bisimilarity preserves the truth of every goal and precondition. Condition (2) is not required for that preservation — bisimilar worlds agree on all formulas whether or not they agree on designation — but it *is* required for the quotient to determine $W^*$, and hence for the canonical form of §5 to be a sound identity test on planning situations. Aletheia includes it, accepting a possibly coarser contraction in exchange.

---

## 3. Representation

### 3.1 The model as a set table

An epistemic state is stored as

| Array        | Shape | Contents |
|--------------|-------|----------|
| `valuation`  | $`\lvert W\rvert \times \lceil \lvert P\rvert/64\rceil`$ words | $V$ as a bit matrix, row per world |
| `set_of`     | $`\lvert Ag\rvert \times \lvert W\rvert`$ ids | $R_i(w)$ as an index into the set table |
| `set_begin`, `members` | one offset per set, one entry per member | the distinct successor sets, each a sorted world list |
| `designated` | $`\lceil \lvert W\rvert/64\rceil`$ words | $`W^*`$ |

Epistemic models repeat successor sets heavily. On K45 frames — transitive and Euclidean, which covers S5 and KD45 — two successor sets of one agent are either equal or disjoint, so an agent's sets hold at most $\lvert W\rvert$ entries together. Both operations that create models preserve the frame: the product update with K45 event relations (every IεPC observability type — fully, partially, oblivious, deceived — is one) and the bisimulation quotient. The table therefore stays linear in $\lvert W\rvert$ along every search, where a bit matrix costs $\lvert Ag\rvert \cdot \lvert W\rvert^2$ bits: 553 MB for a single 19 210-world state of IεPC hard gossip.

Operations on a model read the table once per distinct set:

- $p$ holds at $w$: one bit test.
- $[i]\varphi$: a set qualifies when all its members are in $\mathit{sat}(\varphi)$, decided once per set; each world then reads its set's flag.
- $C_G\varphi$: a backward worklist from the $\neg\varphi$ worlds, visiting each set and each (agent, world) pair once.
- copy a model: five `memcpy`s; hash a model: one linear scan.

Contracted states intern sets by content in canonical order (§5), so two bisimilar states still produce identical arrays.

The representation before this one was a bit matrix per agent, and before that `std::unordered_set<uint32_t>` per world and per $(\mathit{agent}, \mathit{world})$ row. The matrix removed pointer chasing (on `gossip1`, 21 GB down to 7.3 MB); the set table removes the quadratic term, which is what limits large gossip tasks with private and deceptive announcements: IεPC intermediate `gos-11-all` went from 12 expansions in 120 s to a solution in 54 s, and hard `gos-12-imp-deceived` from 7 expansions in 120 s to a solution in 7 s.

### 3.2 Hash-consed formulas

Formula constructors intern into a process-wide registry, so structurally identical formulas are the same object and carry the same dense integer identifier. Interning costs $O(\text{arity})$ rather than $O(|\varphi|)$, because children are already interned and can be compared by address.

The payoff is not memory but memoisation. A task's action preconditions, postcondition guards, observability guards and goal conjuncts share a great many subformulas. Because identity is structural, the model checker's memo table is indexed by a dense formula id and shared across every syntactic occurrence anywhere in the task.

---

## 4. Model checking by satisfaction sets

### 4.1 The method

For every subformula the planner computes its **extension**

```math
\mathit{sat}(\varphi) = \{ w \in W \mid \mathcal{M}, w \models \varphi \} \subseteq W
```

bottom-up, as a bit set. The clauses are

```math
\begin{aligned}
\mathit{sat}(p) &= \{ w \mid p \in V(w) \} \\
\mathit{sat}(\neg\varphi) &= W \setminus \mathit{sat}(\varphi) \\
\mathit{sat}(\varphi \wedge \psi) &= \mathit{sat}(\varphi) \cap \mathit{sat}(\psi) \\
\mathit{sat}([i]\varphi) &= \{ w \mid R_i(w) \subseteq \mathit{sat}(\varphi) \} \\
\mathit{sat}(\mathit{Kw}_i\varphi) &= \mathit{sat}([i]\varphi) \ \cup\ \mathit{sat}([i]\neg\varphi) \\
\mathit{sat}(C_G\varphi) &= \nu X.\ \mathit{sat}(\varphi) \cap \{ w \mid R_G(w) \subseteq X \}
\end{aligned}
```

and goal satisfaction is one subset test: $W^* \subseteq \mathit{sat}(\varphi_g)$.

Three of these clauses replace something structurally worse:

- **Box.** $\mathit{sat}([i]\varphi)$ is decided once per distinct successor set of agent $i$; each world then reads its set's flag.
- **Knowing-whether.** Both modal tests are derived from a single pass over each set against one extension of $\varphi$.
- **Common knowledge.** $C_G\varphi$ is computed as a single fixpoint over the whole model rather than one BFS per world: its complement is propagated backwards from the $\neg\varphi$ worlds, marking each successor set and each (agent, world) pair once.

### 4.2 Complexity

Let $n = |W|$, $\omega = 64$, and $\sigma_i$ the total size of agent $i$'s distinct successor sets ($\sigma_i \le n$ on K45 frames, $n^2$ in the worst case).

| Subformula | Cost |
|------------|------|
| $p$ | $O(n)$ |
| $`\neg,\ \wedge,\ \vee`$ | $O(n/\omega)$ per child |
| $`[i]\varphi`$, $`\mathit{Kw}_i\varphi`$ | $`O(n + \sigma_i)`$ |
| $`C_G\varphi`$ | $`O(\lvert G\rvert \cdot n + \sum_{i \in G} \sigma_i)`$ |

A formula of size $\lvert\varphi\rvert$ therefore costs $O(\lvert\varphi\rvert \cdot (n + \sigma))$ per model on K45 frames — linear in the number of worlds — evaluated once and memoised.

---

## 5. Contraction and canonical form

### 5.1 Why a canonical form

Duplicate detection is the difference between a search space of thousands of states and one of millions. Two Kripke models that represent the same epistemic situation will generally have different world numberings. Aletheia therefore contracts *and* canonically labels in one pass, and identifies states by a 128-bit fingerprint of the resulting byte image.

### 5.2 The algorithm

Contraction proceeds in three stages:

1. **Reachability restriction** — worlds not reachable from $W^*$ are removed.
2. **Ordered partition refinement** — starting from the partition induced by designation and valuation, each round sorts worlds by a key and assigns class identifiers in sorted order.
3. **Quotient** — class $c$ becomes world $c$ of the result, and its successor sets — the classes of its representative's successors — are interned by content in (agent, class) order, which numbers the sets canonically as well.

Each refinement round computes, for every distinct successor set $S$, the sorted list $N(S)$ of classes of its members, ranks the distinct lists by (size, contents), and keys each world by its class and the ranks of its agents' sets. The work per round is linear in the table rather than quadratic in $\lvert W\rvert$.

Neither round orders its worlds by sorting them outright, because in both the key carries less information than $n \log n$ comparisons extract. Round 0 has one key per distinct valuation, and a model holds far fewer of those than worlds — a product update with no ontic effect reproduces every valuation once per event — so the worlds are hashed into groups and only the $d$ distinct keys are sorted. Round $k$ opens its key with the world's previous class, and refinement only splits within a class, so the order is the classes in sequence and then the ranks inside each; the class is a dense id and counting-sorts, leaving the comparison sort one run per class, runs that shrink to singletons as the partition nears its fixpoint. Both produce exactly the order the full sort would, so the canonical form is unchanged. On Gossip with 8 agents the two sorts fall from 22.5% of the planner's instruction count to 0.8%, for a 1.31× reduction overall.

### 5.3 Canonicity

The resulting world numbering depends only on the isomorphism class of the input. Two bisimilar states produce byte-identical output, so fingerprint equality is exactly bisimilarity (collision probability of a 128-bit digest is negligible).

---

## 6. The product update in practice

Three costs dominated the previous implementation and are removed here.

**Guards are evaluated once per model, not once per pair.** Preconditions, postcondition guards and observability guards were tested pointwise, once per $(w,e)$ or per $(w,e,i)$ triple. They are now single extensions, and every later test is a bit lookup. Observability guards are the sharpest case: they depend only on $w$, never on $e$, so evaluating them inside the pair loop repeated each one $\lvert E\rvert$ times over.

**The $(w,e) \to$ world table is a flat array.** It was an `unordered_map` keyed on a packed 64-bit pair, probed from the innermost loop of the relation construction — once per $(i, w, e, v, f)$ quintuple. The index space is dense and small, so a vector with a sentinel is both smaller and free of hashing.

**Successor sets are built once per (set, event row).** The successor set of $(w,e)$ depends only on $R_i(w)$ and on the event row $R^E_i(e)$, so the update builds it once for each distinct pair and points every matching world at it. Worlds are numbered in one block per event, and walking $f \in R^E_i(e)$ and $v \in R_i(w)$ in order visits $\mathit{idx}(v,f)$ in increasing order, so sets come out sorted without sorting.

**Sensing branches share the model.** For a sensing action, the branch for event $e_k$ differs from its siblings only in $`{W'}^*_k = \lbrace (w,e_k) \mid w \in W^* \rbrace`$. All branches are derived from one shared update, which also keeps their world indices mutually coherent — running the update once per event would compact indices independently and leave each branch's designated set referring to different worlds. Branch order is sorted by event index so that the emitted conditional plan is deterministic.

**KD45 repair.** $R_i$ is not serial after a product update: $(w,e)$ is non-serial for $i$ whenever $R_i(w) = \emptyset$ or $R^E_i(e) = \emptyset$, and removal cascades because a removed world may have been another's only successor. The surviving set is the greatest fixpoint of "every agent's row, restricted to survivors, is non-empty", computed by repeated sweeps that flag each successor set once; survivors are then compacted and the pair table patched through the remapping.

**Prune reasons are typed.** The update declines to produce a successor for three distinct reasons — the action was inapplicable, the pre-contraction bound $\lvert W\rvert \cdot \lvert E\rvert$ fired, or seriality repair emptied $W^*$ — and only the first is a property of the domain. These were previously collapsed into a bare `std::nullopt`, making it impossible to distinguish a genuinely dead branch from one the planner chose to prune. They are now carried in the result type and counted separately in the statistics.

---

## 7. Search

Three algorithms share a common substrate.

**Nodes live in an arena; open lists hold indices.** The open list stores $(h, g, \text{index})$ triples of twelve bytes, so heap operations move integers. The previous implementation stored nodes by value inside a `std::priority_queue` and read the top with `Node node = open.top()`, deep-copying an entire Kripke model on every expansion and again on every sift.

**Plans are parent links.** Each node records its parent's index and the action that reached it; the action sequence is reconstructed once on success. The previous code copied the whole prefix `vector<string>` into every generated successor, making plan storage alone $O(\text{nodes} \times \text{depth})$.

**Closed lists hold fingerprints.** Sixteen bytes and two integer comparisons per membership test, against a stored model and a graph walk before.

### 7.1 Greedy best-first search

Standard GBFS over contracted states, with the duplicate check performed *before* heuristic evaluation — the heuristic is the most expensive operation per successor and a fingerprint lookup is nearly free. Ties in $h$ are broken toward the deeper node, the usual greedy dive, which matters here because epistemic plateaus are wide.

### 7.2 Enforced hill climbing

Greedy descent to any $h$-improving successor; on a plateau, a breadth-first search for the nearest strictly better state. Both phases share one node arena and one visited set, which is what prevents the descent from re-entering a region the escape has already crossed and vice versa.

### 7.3 Iterative-deepening AND-OR search

For sensing actions the planner must produce a branching policy, one subplan per outcome. The search is a depth-bounded AND-OR DFS wrapped in iterative deepening, with two memo tables that **persist across the deepening iterations**:

- **Solved subtrees**, keyed by fingerprint and tagged with their height. A cached solution is reused only when its height fits the remaining budget, so the depth bound the current iteration is enforcing is never violated. Reuse turns the output from a plan tree into a plan DAG.
- **Refuted states**, keyed by fingerprint and tagged with the greatest depth at which failure was proven. Failure at depth $d$ implies failure at every $d' \le d$.

The previous implementation rebuilt its memo table at every depth, so each iteration re-expanded from scratch everything its predecessor had already refuted. The measured effect is a factor of 19.8 in iteration throughput ([evaluation](docs/evaluation.md)).

Persistence requires care, because two kinds of failure are not properties of the state alone. A branch cut because its state is already on the current DFS path, and a branch cut because the deadline expired, are path- and time-dependent respectively; the same state reached by another path, or a moment earlier, may well be solvable. Such failures are marked *tainted* and propagate that mark upward; only untainted failures enter the memo. Solutions are always sound to cache, since a policy from a state depends on nothing but the state.

Action ranking also changed. The previous code ranked actions by running `product_update`, discarded the resulting state, and then ran `product_update_split` again on whichever action it committed to — computing the expensive part twice. The split is now computed once and carried into the recursion. Ranking uses the *worst* branch rather than the heuristic of the merged product: an AND node is solved only when every branch is solved, so the binding constraint is the hardest outcome, whereas the merged product's designated set is the union over designated events and describes no branch in particular.

---

## 8. Heuristics

Four goal-decomposition estimates are available, selected automatically from
task structure by a rule table over features of the task — frame, goal shape,
model size — that is data rather than control flow, and can be replaced without
rebuilding ([usage](docs/usage.md)). The same table selects the search
algorithm of §7.

| | estimate |
|---|---|
| `wc` | $`\lvert W^*\rvert`$ — uncertainty as raw world count |
| `ug` | number of unsatisfied top-level goal conjuncts |
| `ed` | *epistemic distance*: for an unsatisfied $`[i]\varphi`$, the fraction of worlds $i$ considers possible that are counterexamples to $\varphi$; nested modalities are handled by projecting $`W^*`$ through $R_i$ and recursing |
| `ks` | *knowledge spread*: the same measure specialised to conjunctions of $`\mathit{Kw}`$ goals across agents, where it tracks knowledge propagating through the agent graph |
| `kadd` | *knowledge relaxation*: additive cost over a delete-free relaxation whose facts are literals and knowledge literals $`[j]\ell`$; an event teaches agent $`j`$ the literals shared by every event $`j`$ cannot tell it apart from (`src/knowledge_relaxation.cpp`) |
| `kff` | same relaxation; counts the operators of the relaxed plan extracted from the cost fixpoint instead of summing goal costs (FF-style) |

`ed` and `ks` improve on `ug` by giving a real-valued gradient where `ug` sees only 0 or 1 per conjunct. Both cut their counterexample scan off after a fixed number of accessible worlds to bound cost on wide models.

That cutoff had a consequence worth recording. The previous implementation walked `std::unordered_set` to enumerate accessible worlds, so *which* worlds fell inside the sample — and therefore the heuristic value, and therefore the plan — depended on hash iteration order. The planner was not reproducible. Bit sets are traversed in ascending index order, so the same state now always yields the same estimate.

All four are now computed from the state's satisfaction cache, so the goal is evaluated once per state and shared across conjuncts. The previous code called a full recursive `satisfies` per conjunct on top of a per-heuristic $(\text{formula}, \text{world})$ memo that could not outlive a single call.

### 8.1 A relaxation, and why it does not pay here

None of the four solves a relaxed problem, so none of them estimates a *distance*. Two further heuristics do, via a **relaxed announcement closure**.

The classical delete-relaxation does not transfer. In classical planning one drops delete effects because progress is monotone growth of a fact set. In DEL the actions that establish knowledge carry no ontic effect at all: they make progress by *eliminating* possibilities. The monotone quantity is therefore the model, shrinking rather than growing, and two things shrink:

```math
W_{k+1} \ =\ W_k \cap \bigcap \, \lbrace \mathit{sat}(\mathit{pre}(e)) \ \mid\ a \in \mathcal{A},\ e \in E_d(a) \rbrace
```

```math
R_i \ \leftarrow\ R_i \setminus \big( \mathit{sat}(\mathit{pre}(e)) \times \mathit{sat}(\mathit{pre}(f)) \big) \quad \text{whenever } i \text{ distinguishes } e \text{ from } f
```

The second term is the essential one. Under a private announcement no world is eliminated at all — an agent who observes which event occurred simply loses the edges between worlds the two events separate. A relaxation that only prunes worlds reaches its fixpoint at layer zero on Gossip and Grapevine and reports nothing.

The layer at which each goal conjunct first becomes true is then a step count, aggregated as `rpg` (max, a lower bound) or `radd` (sum, assuming conjunct independence). Cost is $`O(L \cdot \lvert \mathcal{A}\rvert \cdot \lvert E\rvert \cdot \lvert\varphi\rvert \cdot n^2/\omega)`$ with $L \le \lvert W\rvert$ layers.

**Measured, it does not help on this suite, and the reason is structural.** Epistemic action preconditions are typically *anti*-monotone: `tell_A_B` in Gossip requires $\mathit{secret}_A \wedge \neg \mathit{Kw}_B(\mathit{secret}_A)$, so establishing knowledge *disables* the actions that establish it. The delete-relaxation's founding assumption — that achieving a fact never removes an option — is violated by construction. On Gossip the closure fires every action in one layer and returns a constant.

Where preconditions are monotone the closure does discriminate. On `coin4` it separates the initial state from its successor, $2.0 \to 1.0$, where `ug`, `ed` and `ks` are all flat at $1.0$. That is real information, and it is why both remain available.

Neither is used by the automatic selector. `radd` is roughly neutral on the suite (within ±13 expansions of `ed` everywhere) at two to three times the per-node cost; `rpg`'s max aggregation collapses conjunctive goals and regresses `grapevine1` from 5 expansions to 279. Treat `rpg` as an admissible lower bound rather than a search guide.

### 8.2 Search guidance is not this planner's bottleneck

The suite provides no headroom for a better heuristic, and it is worth stating why rather than leaving it implicit. On Gossip scaled from 5 to 7 agents, every heuristic — including plain `ug` — expands exactly the plan length, with no backtracking whatsoever:

| instance | worlds | actions | plan | expansions (`ug` / `ed` / `ks` / `radd`) | time |
|---|---:|---:|---:|---:|---:|
| gossip-5 | 32 | 20 | 20 | 20 / 20 / 20 / 20 | 0.05 s |
| gossip-6 | 64 | 30 | 30 | 30 / 30 / 30 / 30 | 0.6 s |
| gossip-7 | 128 | 42 | 42 | 42 / 42 / 42 / 42 | 7.5 s |

Search cost grows two orders of magnitude across these instances while the number of expansions stays exactly optimal. The time is spent evaluating nodes — product update and contraction over a 128-world model — not choosing between them. The same holds for the conditional instances: `coin4`'s 629 expansions are almost entirely iterative-deepening re-expansion (its memo ends at 6 solved / 20 refuted entries, and only one action is applicable at the root), not misranking.

The conclusion is that further heuristic work should be deferred behind per-node cost and behind replacing iterative deepening in the AND-OR search (§10). A heuristic can only pay once search is actually branching.

---

## 9. Correctness notes

Two defects in the previous implementation are worth recording, since both were silent.

**Bisimulation merged worlds with different valuations.** The initial partition assigned class identifiers by *hash* of the sorted atom set, with no fallback comparison. A hash collision therefore placed two worlds with different valuations in the same class — and refinement could never separate them, because the refinement signature consists of class identifiers and never re-reads valuations. The result was an unsound contraction: a model claiming worlds are indistinguishable that a formula can in fact tell apart. The initial partition now compares valuation words exactly.

**Knowing-whether was evaluated twice.** $\mathit{Kw}_i\varphi$ tested `holds_at(inner, v)` and then `!holds_at(inner, v)` as separate calls per accessible world. Not incorrect, but a factor of two compounding through every nesting level.

Separately, the hash combiner throughout was a Boost-style `hash_combine` applied to `std::hash<uint32_t>`, which libstdc++ implements as the identity — so the inputs to the combiner had no avalanche at all, in a system where bisimulation class assignment and closed-list behaviour both depend on collision rates. All hashing now goes through a splitmix64 finaliser.

The planner remains **incomplete by design** in one respect: the pre-contraction bound $\lvert W\rvert \cdot \lvert E\rvert$ prunes branches outright rather than deferring them. This is sound in the sense that it never produces an invalid plan, but a task whose only solution passes through a wide intermediate model will be reported unsolvable. The bound is checked against the pessimistic count, before precondition filtering, so it can reject branches that would in fact have contracted to a handful of worlds.


### 9.1 Proving unsolvability

Iterative deepening cannot conclude "no solution" by itself: a failed iteration
is ambiguous between a genuinely dead space and one the depth bound truncated.
The previous test compared expansion counts between iterations, which the
persistent memo defeats — memo hits keep each iteration cheap while still
expanding more than the root, so the test never fired and the planner deepened
forever. `amc1` reached depth $2.75 \times 10^5$; a two-world instance of
Consecutive Numbers reached depth $1.59 \times 10^6$.

The search now tracks whether any branch was abandoned *because the budget ran
out* — that is, whether any call reached depth zero, or the deadline expired.
If an iteration completes without that flag being raised, the bound never bound
anything: every acyclic path through the AND-OR space was walked to a dead end,
and a larger budget explores exactly the same space. That is a proof of
unsolvability.

Two details make it sound. Failures served from the memo replay their own
truncation flag, conservatively — a failure that was truncated at depth $d$ is
also truncated at any smaller depth, so propagating the flag can only delay the
proof, never fabricate one. And the ancestor cut needs no special handling: it
fires when a state already lies on the current path, which is equally true at
any larger budget, so a cut branch is not a truncated one.

The effect on the benchmark suite is that both instances the planner cannot
solve are now *proven* unsolvable in 3 ms rather than timing out. Both were
right to fail: their reference plans are `null`, and GBFS independently
exhausts the same spaces. `amc1` is degenerate as encoded — `pre(e-ask-pos)`
holds at no world and `pre(e-ask-neg)` at all 32, so every action satisfies
$\mathcal{M} \otimes \mathcal{E} \cong \mathcal{M}$ and the reachable space is
the initial state alone. Muddy Children without the initiating public
announcement is a fixpoint.

### 9.2 Conformant applicability and the Consecutive Numbers family

Consecutive Numbers is worth recording in full, because it shows how much the
ontic/sensing distinction decides.

The domain declares its announcement with a single event, whose precondition is
that the speaker does *not* know the other's number. One event makes it an ontic
action, so conformant applicability requires the precondition to hold at every
designated world (§2.2, §7). In cn-5 it does not:

```math
\mathit{sat}(\mathit{pre}(\texttt{ann-A-B})) = \lbrace w_0, w_1, w_2, w_3 \rbrace, \qquad W^* = \lbrace w_3, w_4 \rbrace
```

A knows B's number in one actual world and not in the other, so the action is
never applicable, the other announcement prunes nothing, and the task reaches a
fixpoint immediately. That verdict is correct: there is no conformant plan.

But the puzzle's announcement is not an ontic act — it is an *observation*. The
other agent hears which answer was given, so the action is public sensing with
two events, exactly as Muddy Children encodes `ask`. Declaring both events makes
the planner branch on the answer:

| | as shipped (one event) | public sensing (two events) |
|---|---|---|
| cn-1 | goal already true | goal already true |
| cn-2 | unsolvable | unsolvable (proven, 3 ms) |
| cn-3 | depth 1 | depth 1, 3 expansions |
| cn-4 | unsolvable | depth 1, 3 expansions |
| **cn-5** | **unsolvable** | **depth 2, 13 expansions** |
| cn-6 … cn-13 | unsolvable | depth 1 or 2, ≤13 expansions |

The cn-5 policy is the puzzle's own argument: ask A whether it knows B's number;
if yes, B has learned that A knows, and the goal $[B][A]\,\mathit{has}(B,n_4)$
holds; if no, ask B, whose answer then settles it. The validator replays both
branches.

Every instance from cn-4 upward flips from unsolvable to solvable in one or two
steps, and the odd/even alternation in depth is the parity of the chain. The
planner's behaviour was right in both encodings; only the encoding's reading of
what an announcement *is* changed.


---

## 10. Limitations and further work

**The world cap is a completeness hole.** It is checked against $\lvert W\rvert \cdot \lvert E\rvert$ before precondition filtering. The surviving count is now cheap to compute exactly — one population count per event over the precondition extension — so the bound should be moved onto the real count, or onto post-contraction size.

**The relaxation is blocked by anti-monotone preconditions.** §8.1: `rpg`/`radd` relax the model rather than the fact set, which is the right direction for DEL, but epistemic preconditions of the form $\neg \mathit{Kw}_i \varphi$ mean that establishing knowledge disables the actions that establish it. A relaxation that survives this needs to track *which* announcements remain available, not just what is known — closer to a landmark decomposition over (agent, proposition) pairs than to a planning graph.

**Refinement is not asymptotically optimal.** §5.2: obtaining both Paige–Tarjan's $O(m \log n)$ and a canonical labelling would require canonicalising the fixpoint partition in a separate pass.

**Iterative deepening is the wrong outer loop for AND-OR search.** LAO\*, with an explicit AND-OR graph and value iteration over it, avoids re-descending solved regions entirely. The persistent memo recovers part of that benefit but not the ordering.

**Unsolvability is proven, not merely timed out — but only for AND-OR search.** §9.1. GBFS and EHC still report exhaustion without distinguishing "no plan exists" from "the world cap pruned the only route", because the cap prunes branches silently. Typed prune reasons (§6) now make that distinguishable in principle; the search does not yet act on it.

**Successor generation is single-threaded.** The loop over applicable actions is embarrassingly parallel, and refinement rounds parallelise per world. The satisfaction cache would need to become thread-local or immutable first.

---

## 11. Implementation notes

The planner is a self-contained C++23 binary. Beyond the algorithmic content above, the following language facilities carry weight in the design:

- `std::span` throughout the bit-set layer, so a single allocation can back an entire model and every set-valued object is a view into it rather than an owner.
- `<bit>` — `std::countr_zero` for set-bit iteration, `std::popcount` for cardinality.
- `std::expected` for typed prune reasons, with a minimal portable fallback where libstdc++ predates it.
- Concepts on the bit-set callbacks (`std::invocable`, `std::predicate`), so misuse is a constraint failure rather than a template error.
- `constexpr` on the whole bit-set layer and on the hash finaliser.

Notably absent: `std::mdspan`, which is the natural spelling for the $\lvert W\rvert \times \lvert E\rvert$ pair table and the per-agent adjacency matrix, and `std::flat_set` for the small sparse sets that remain. Neither is available in any libstdc++ on the target toolchain; both are worth adopting when they are.

---

## References

- Baltag, Moss & Solecki. *The Logic of Public Announcements, Common Knowledge, and Private Suspicions*. TARK 1998.
- van Ditmarsch, van der Hoek & Kooi. *Dynamic Epistemic Logic*. Springer, 2007.
- Bolander & Andersen. *Epistemic Planning for Single- and Multi-Agent Systems*. Journal of Applied Non-Classical Logics, 2011.
- Paige & Tarjan. *Three Partition Refinement Algorithms*. SIAM Journal on Computing, 1987.
- Hoffmann & Nebel. *The FF Planning System*. JAIR, 2001.
- Hansen & Zilberstein. *LAO\*: A Heuristic Search Algorithm that Finds Solutions with Loops*. Artificial Intelligence, 2001.

---

## License

Apache License 2.0
