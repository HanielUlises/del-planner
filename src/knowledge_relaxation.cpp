#include "knowledge_relaxation.hpp"

#include <algorithm>
#include <map>

namespace {

constexpr std::int32_t kInf  = 1 << 29;
constexpr float        kDead = 1000.f;   // per goal conjunct unreachable in the relaxation

std::int32_t add_sat(std::int32_t a, std::int32_t b) { return std::min(a + b, kInf); }

bool is_literal(const Formula& f) {
    return f.kind == FormulaKind::Atom ||
           (f.kind == FormulaKind::Not && f.children[0]->kind == FormulaKind::Atom);
}

AtomIdx atom_of(const Formula& lit) {
    return lit.kind == FormulaKind::Atom ? lit.atom : lit.children[0]->atom;
}

void literal_conjuncts(const FormulaPtr& f, std::vector<FormulaPtr>& out) {
    if (f->kind == FormulaKind::And) {
        for (const auto& c : f->children) literal_conjuncts(c, out);
    } else if (is_literal(*f)) {
        out.push_back(f);
    }
}

// Literals guaranteed to hold after event e: unoverwritten precondition
// conjuncts plus unconditional postconditions. Sorted by formula id.
std::vector<FormulaPtr> after_literals(const Event& ev) {
    std::vector<FormulaPtr> lits;
    literal_conjuncts(ev.precondition, lits);
    std::erase_if(lits, [&](const FormulaPtr& l) {
        const AtomIdx p = atom_of(*l);
        return ev.post_true.count(p) || ev.post_false.count(p);
    });
    for (const auto& [p, cond] : ev.post_true)
        if (cond->kind == FormulaKind::Top && !ev.post_false.count(p))
            lits.push_back(Formula::make_atom(p));
    for (const auto& [p, cond] : ev.post_false)
        if (cond->kind == FormulaKind::Top && !ev.post_true.count(p))
            lits.push_back(Formula::make_not(Formula::make_atom(p)));
    std::sort(lits.begin(), lits.end(), [](auto& x, auto& y) { return x->id < y->id; });
    lits.erase(std::unique(lits.begin(), lits.end()), lits.end());
    return lits;
}

} // namespace

std::uint32_t KnowledgeRelaxationHeuristic::node(Kind k, std::vector<std::uint32_t> children) {
    Req r;
    r.kind  = k;
    r.begin = static_cast<std::uint32_t>(req_children_.size());
    req_children_.insert(req_children_.end(), children.begin(), children.end());
    r.end   = static_cast<std::uint32_t>(req_children_.size());
    reqs_.push_back(r);
    return static_cast<std::uint32_t>(reqs_.size() - 1);
}

std::uint32_t KnowledgeRelaxationHeuristic::fact(const FormulaPtr& f) {
    if (auto it = fact_of_.find(f->id); it != fact_of_.end()) return it->second;

    const auto idx = static_cast<std::uint32_t>(facts_.size());
    fact_of_[f->id] = idx;
    facts_.push_back(f);
    derived_.push_back(-1);

    if (f->kind == FormulaKind::Belief && f->children[0]->kind == FormulaKind::Kw &&
        is_literal(*f->children[0]->children[0])) {
        const AgentIdx   i = f->agent, j = f->children[0]->agent;
        const FormulaPtr& p = f->children[0]->children[0];
        const FormulaPtr  q = p->kind == FormulaKind::Not ? p->children[0] : Formula::make_not(p);
        const std::uint32_t pos = fact(Formula::make_belief(i, Formula::make_belief(j, p)));
        const std::uint32_t neg = fact(Formula::make_belief(i, Formula::make_belief(j, q)));
        Req a{Kind::Fact, pos, 0, 0}, b{Kind::Fact, neg, 0, 0};
        reqs_.push_back(a);
        const auto ra = static_cast<std::uint32_t>(reqs_.size() - 1);
        reqs_.push_back(b);
        const auto rb = static_cast<std::uint32_t>(reqs_.size() - 1);
        derived_[idx] = static_cast<std::int32_t>(node(Kind::Or, {ra, rb}));
    }
    if (f->kind == FormulaKind::Kw && is_literal(*f->children[0])) {
        const FormulaPtr& p = f->children[0];
        const std::uint32_t pos = fact(Formula::make_belief(f->agent, p));
        const std::uint32_t neg = fact(Formula::make_belief(
            f->agent, p->kind == FormulaKind::Not ? p->children[0] : Formula::make_not(p)));
        Req a{Kind::Fact, pos, 0, 0}, b{Kind::Fact, neg, 0, 0};
        reqs_.push_back(a);
        const auto ra = static_cast<std::uint32_t>(reqs_.size() - 1);
        reqs_.push_back(b);
        const auto rb = static_cast<std::uint32_t>(reqs_.size() - 1);
        derived_[idx] = static_cast<std::int32_t>(node(Kind::Or, {ra, rb}));
    }
    return idx;
}

std::uint32_t KnowledgeRelaxationHeuristic::compile(const FormulaPtr& f, bool negated) {
    const std::uint64_t key = (std::uint64_t(f->id) << 1) | (negated ? 1 : 0);
    if (auto it = compiled_.find(key); it != compiled_.end()) return it->second;

    std::uint32_t out = 0;
    switch (f->kind) {
        case FormulaKind::Top:
            out = node(negated ? Kind::False : Kind::True, {});
            break;
        case FormulaKind::Bot:
            out = node(negated ? Kind::True : Kind::False, {});
            break;
        case FormulaKind::Not:
            out = compile(f->children[0], !negated);
            break;
        case FormulaKind::Belief:
            if (f->children[0]->kind == FormulaKind::And) {
                // [i](φ ∧ ψ) ≡ [i]φ ∧ [i]ψ in every Kripke model: one fact per
                // conjunct, so that the relaxation can reach them one at a time.
                std::vector<FormulaPtr> bs;
                for (const auto& c : f->children[0]->children) bs.push_back(Formula::make_belief(f->agent, c));
                out = compile(Formula::make_and(std::move(bs)), negated);
                break;
            }
            [[fallthrough]];
        default: {   // atoms and modal formulas are facts
            const std::uint32_t fi = fact(negated ? Formula::make_not(f) : f);
            Req r{Kind::Fact, fi, 0, 0};
            reqs_.push_back(r);
            out = static_cast<std::uint32_t>(reqs_.size() - 1);
            break;
        }
        case FormulaKind::And:
        case FormulaKind::Or: {
            std::vector<std::uint32_t> cs;
            for (const auto& c : f->children) cs.push_back(compile(c, negated));
            const bool conj = (f->kind == FormulaKind::And) != negated;
            out = node(conj ? Kind::And : Kind::Or, std::move(cs));
            break;
        }
    }
    compiled_[key] = out;
    return out;
}

void KnowledgeRelaxationHeuristic::add_op(std::uint32_t pre, const std::vector<std::uint32_t>& adds) {
    if (adds.empty()) return;
    Op op{pre, static_cast<std::uint32_t>(op_adds_.size()), 0, current_action_};
    op_adds_.insert(op_adds_.end(), adds.begin(), adds.end());
    op.end = static_cast<std::uint32_t>(op_adds_.size());
    ops_.push_back(op);
}

KnowledgeRelaxationHeuristic::KnowledgeRelaxationHeuristic(const PlanningTask& task, Estimate estimate)
    : estimate_(estimate) {
    // Goal conjuncts, with [i](φ ∧ ψ) split into [i]φ and [i]ψ: kDead counts
    // unreachable conjuncts, so one box over a conjunction would make every
    // state look like a dead end until the goal holds.
    std::vector<FormulaPtr> conj;
    auto split = [&](auto& self, const FormulaPtr& f) -> void {
        if (f->kind == FormulaKind::And) { for (const auto& c : f->children) self(self, c); return; }
        if (f->kind == FormulaKind::Belief && f->children[0]->kind == FormulaKind::And) {
            for (const auto& c : f->children[0]->children) self(self, Formula::make_belief(f->agent, c));
            return;
        }
        conj.push_back(f);
    };
    split(split, task.goal);
    for (const auto& c : conj) goal_.push_back(compile(c, false));

    // First pass: compile every precondition and observability condition, so
    // the nested facts [i][j]ℓ the task mentions are known before operators
    // are built.
    for (const Action& a : task.actions) {
        for (EventIdx e : a.designated_events)
            if (e < a.events.size()) compile(a.events[e].precondition, false);
        for (const auto& cases : a.obs_cases)
            for (const ObsCase& c : cases) compile(c.condition, false);
    }
    std::map<std::pair<AgentIdx, AgentIdx>, std::vector<std::uint32_t>> nested;   // (i, j) → literal ids

    // C_G ψ for ψ a literal or a disjunction of literals: the literals any of
    // which, once commonly observed, establishes the fact.
    struct Common { FormulaPtr fact; std::vector<AgentIdx> group; std::vector<std::uint32_t> lits; };
    std::vector<Common> commons;
    for (const FormulaPtr& f : facts_) {
        if (f->kind != FormulaKind::Common) continue;
        const FormulaPtr& psi = f->children[0];
        Common c{f, f->group, {}};
        if (is_literal(*psi)) c.lits.push_back(psi->id);
        else if (psi->kind == FormulaKind::Or &&
                 std::all_of(psi->children.begin(), psi->children.end(),
                             [](const FormulaPtr& x) { return is_literal(*x); }))
            for (const auto& x : psi->children) c.lits.push_back(x->id);
        if (!c.lits.empty()) commons.push_back(std::move(c));
    }

    for (const FormulaPtr& f : facts_)
        if (f->kind == FormulaKind::Belief && f->children[0]->kind == FormulaKind::Belief &&
            is_literal(*f->children[0]->children[0]))
            nested[{f->agent, f->children[0]->agent}].push_back(f->children[0]->children[0]->id);

    for (const Action& a : task.actions) {
        current_action_ = static_cast<ActionIdx>(&a - task.actions.data());
        const std::size_t ne = a.events.size();
        std::vector<std::vector<FormulaPtr>> after(ne);
        for (std::size_t e = 0; e < ne; ++e) after[e] = after_literals(a.events[e]);

        std::vector<bits::Word> all_events(bits::words_for(ne), 0);
        bits::fill_all(all_events, ne);

        // Literals holding after every event in `row`.
        const auto common_after = [&](const std::vector<EventIdx>& row) {
            std::vector<FormulaPtr> common;
            bool first = true;
            for (EventIdx f : row) {
                if (f >= ne) continue;
                if (first) { common = after[f]; first = false; continue; }
                std::vector<FormulaPtr> keep;
                std::set_intersection(common.begin(), common.end(), after[f].begin(), after[f].end(),
                                      std::back_inserter(keep),
                                      [](auto& x, auto& y) { return x->id < y->id; });
                common.swap(keep);
            }
            return common;
        };
        const auto events_in = [&](bits::ConstWordSpan row) {
            std::vector<EventIdx> out;
            bits::for_each(row, [&](std::uint32_t f) { out.push_back(f); });
            return out;
        };
        // (condition requirement, event row) per observability case of agent j.
        const auto obs_of = [&](AgentIdx j, EventIdx e) {
            std::vector<std::pair<std::uint32_t, std::vector<EventIdx>>> out;
            if (j >= a.obs_cases.size() || a.obs_cases[j].empty())
                out.emplace_back(node(Kind::True, {}), events_in(a.default_obs.event_row(e)));
            else
                for (const ObsCase& c : a.obs_cases[j])
                    out.emplace_back(compile(c.condition, false), events_in(c.event_row(e)));
            return out;
        };

        for (EventIdx e : a.designated_events) {
            if (e >= ne) continue;
            const Event& ev = a.events[e];
            const std::uint32_t pre = compile(ev.precondition, false);

            std::vector<std::uint32_t> ontic;
            for (const auto& [p, cond] : ev.post_true) {
                const std::uint32_t lit = fact(Formula::make_atom(p));
                if (cond->kind == FormulaKind::Top) ontic.push_back(lit);
                else add_op(node(Kind::And, {pre, compile(cond, false)}), {lit});
            }
            for (const auto& [p, cond] : ev.post_false) {
                const std::uint32_t lit = fact(Formula::make_not(Formula::make_atom(p)));
                if (cond->kind == FormulaKind::Top) ontic.push_back(lit);
                else add_op(node(Kind::And, {pre, compile(cond, false)}), {lit});
            }
            add_op(pre, ontic);

            for (AgentIdx j = 0; j < task.num_agents(); ++j) {
                for (const auto& [cond, row] : obs_of(j, e)) {
                    std::vector<std::uint32_t> gains;
                    for (const auto& l : common_after(row))
                        gains.push_back(fact(Formula::make_belief(j, l)));
                    add_op(node(Kind::And, {pre, cond}), gains);
                }
            }

            // C_G ψ: every agent in G has a case that sees e alone, and a literal
            // of ψ holds after e.
            for (const Common& cm : commons) {
                const bool learnt = std::any_of(after[e].begin(), after[e].end(), [&](const FormulaPtr& l) {
                    return std::find(cm.lits.begin(), cm.lits.end(), l->id) != cm.lits.end();
                });
                if (!learnt) continue;
                std::vector<std::uint32_t> conds{pre};
                bool all_see = true;
                for (AgentIdx g : cm.group) {
                    std::vector<std::uint32_t> alts;
                    for (const auto& [cond, row] : obs_of(g, e))
                        if (row.size() == 1 && row[0] == e) alts.push_back(cond);
                    if (alts.empty()) { all_see = false; break; }
                    conds.push_back(alts.size() == 1 ? alts[0] : node(Kind::Or, alts));
                }
                if (all_see) add_op(node(Kind::And, conds), {fact(cm.fact)});
            }

            // [i][j]ℓ: in every event i cannot tell from e, j sees only events
            // after which ℓ holds. Built only for the triples the task mentions.
            for (const auto& [ij, lits] : nested) {
                const auto [i, j] = ij;
                for (const auto& [ci, row_i] : obs_of(i, e)) {
                    if (row_i.empty()) continue;
                    for (std::size_t k = 0; k < (j < a.obs_cases.size() && !a.obs_cases[j].empty()
                                                     ? a.obs_cases[j].size() : 1); ++k) {
                        std::vector<FormulaPtr> common;
                        bool first = true, empty_row = false;
                        std::uint32_t cj = 0;
                        for (EventIdx f : row_i) {
                            if (f >= ne) continue;
                            const auto cases = obs_of(j, f);
                            cj = cases[k].first;
                            if (cases[k].second.empty()) { empty_row = true; break; }
                            auto c = common_after(cases[k].second);
                            if (first) { common.swap(c); first = false; continue; }
                            std::vector<FormulaPtr> keep;
                            std::set_intersection(common.begin(), common.end(), c.begin(), c.end(),
                                                  std::back_inserter(keep),
                                                  [](auto& x, auto& y) { return x->id < y->id; });
                            common.swap(keep);
                        }
                        if (first || empty_row) continue;
                        std::vector<std::uint32_t> gains;
                        for (const auto& l : common)
                            if (std::find(lits.begin(), lits.end(), l->id) != lits.end())
                                gains.push_back(fact(Formula::make_belief(i, Formula::make_belief(j, l))));
                        add_op(node(Kind::And, {pre, ci, cj}), gains);
                    }
                }
            }
        }
    }
    prune();
}

// Keeps only facts, requirements and operators that can contribute to the goal.
void KnowledgeRelaxationHeuristic::prune() {
    std::vector<char> req_needed(reqs_.size(), 0), fact_needed(facts_.size(), 0);
    std::vector<std::uint32_t> stack;

    const auto mark = [&](std::uint32_t r) {
        stack.push_back(r);
        while (!stack.empty()) {
            const std::uint32_t x = stack.back();
            stack.pop_back();
            if (req_needed[x]) continue;
            req_needed[x] = 1;
            const Req& q = reqs_[x];
            if (q.kind == Kind::Fact && !fact_needed[q.fact]) {
                fact_needed[q.fact] = 1;
                if (derived_[q.fact] >= 0) stack.push_back(static_cast<std::uint32_t>(derived_[q.fact]));
            }
            for (std::uint32_t i = q.begin; i < q.end; ++i) stack.push_back(req_children_[i]);
        }
    };

    for (std::uint32_t r : goal_) mark(r);
    std::vector<char> op_needed(ops_.size(), 0);
    for (bool changed = true; changed;) {
        changed = false;
        for (std::size_t o = 0; o < ops_.size(); ++o) {
            if (op_needed[o]) continue;
            for (std::uint32_t i = ops_[o].begin; i < ops_[o].end; ++i)
                if (fact_needed[op_adds_[i]]) { op_needed[o] = 1; break; }
            if (op_needed[o]) { mark(ops_[o].pre); changed = true; }
        }
    }

    std::vector<Op> ops;
    std::vector<std::uint32_t> adds;
    for (std::size_t o = 0; o < ops_.size(); ++o) {
        if (!op_needed[o]) continue;
        Op op{ops_[o].pre, static_cast<std::uint32_t>(adds.size()), 0, ops_[o].action};
        for (std::uint32_t i = ops_[o].begin; i < ops_[o].end; ++i)
            if (fact_needed[op_adds_[i]]) adds.push_back(op_adds_[i]);
        op.end = static_cast<std::uint32_t>(adds.size());
        ops.push_back(op);
    }
    ops_.swap(ops);
    op_adds_.swap(adds);

    // Unneeded requirements become constants so evaluation can skip them;
    // unneeded facts are never evaluated against the state.
    for (std::size_t r = 0; r < reqs_.size(); ++r)
        if (!req_needed[r]) reqs_[r] = Req{Kind::False, 0, 0, 0};
    for (std::size_t f = 0; f < facts_.size(); ++f)
        if (!fact_needed[f]) { facts_[f] = nullptr; derived_[f] = -1; }
}

std::vector<std::uint8_t> KnowledgeRelaxationHeuristic::initial_facts(const EpistemicState& s) const {
    std::vector<std::uint8_t> init(facts_.size(), 0);
    const auto des = s.designated_bits();
    for (std::size_t f = 0; f < facts_.size(); ++f)
        if (facts_[f] && bits::intersects(des, s.sat(*facts_[f]))) init[f] = 1;
    return init;
}

std::uint32_t KnowledgeRelaxationHeuristic::costs(const EpistemicState& s, std::vector<std::int32_t>& fc,
                                         std::vector<std::int32_t>& rc,
                                         std::vector<std::int32_t>* supporter) const {
    return costs(initial_facts(s), fc, rc, supporter);
}

std::uint32_t KnowledgeRelaxationHeuristic::costs(const std::vector<std::uint8_t>& init,
                                         std::vector<std::int32_t>& fc,
                                         std::vector<std::int32_t>& rc,
                                         std::vector<std::int32_t>* supporter) const {
    fc.assign(facts_.size(), kInf);
    rc.assign(reqs_.size(), kInf);
    if (supporter) supporter->assign(facts_.size(), -1);

    for (std::size_t f = 0; f < facts_.size(); ++f)
        if (init[f]) fc[f] = 0;

    const auto eval = [&] {
        for (std::size_t r = 0; r < reqs_.size(); ++r) {
            const Req& q = reqs_[r];
            switch (q.kind) {
                case Kind::True:  rc[r] = 0; break;
                case Kind::False: rc[r] = kInf; break;
                case Kind::Fact:  rc[r] = fc[q.fact]; break;
                case Kind::And: {
                    std::int32_t c = 0;
                    for (std::uint32_t i = q.begin; i < q.end && c < kInf; ++i)
                        c = add_sat(c, rc[req_children_[i]]);
                    rc[r] = c;
                    break;
                }
                case Kind::Or: {
                    std::int32_t c = kInf;
                    for (std::uint32_t i = q.begin; i < q.end; ++i) c = std::min(c, rc[req_children_[i]]);
                    rc[r] = c;
                    break;
                }
            }
        }
    };

    // Children precede parents in reqs_, so one ordered pass per round is exact
    // for the current fact costs; rounds propagate operator effects.
    std::uint32_t rounds = 0;
    for (int round = 0; round < 1024; ++round, ++rounds) {
        eval();
        bool changed = false;
        for (std::size_t o = 0; o < ops_.size(); ++o) {
            const Op& op = ops_[o];
            const std::int32_t c = rc[op.pre];
            if (c >= kInf) continue;
            for (std::uint32_t i = op.begin; i < op.end; ++i)
                if (c + 1 < fc[op_adds_[i]]) {
                    fc[op_adds_[i]] = c + 1;
                    if (supporter) (*supporter)[op_adds_[i]] = static_cast<std::int32_t>(o);
                    changed = true;
                }
        }
        for (std::size_t f = 0; f < facts_.size(); ++f)
            if (derived_[f] >= 0 && rc[derived_[f]] < fc[f]) {
                fc[f] = rc[derived_[f]];
                if (supporter) (*supporter)[f] = -1;
                changed = true;
            }
        if (!changed) break;
    }
    if (supporter) eval();
    return rounds;
}

std::uint32_t KnowledgeRelaxationHeuristic::extract(const std::vector<std::int32_t>& rc,
                                                    const std::vector<std::int32_t>& sup,
                                                    std::vector<ActionIdx>* helpful) const {
    thread_local std::vector<char> used, seen_req, seen_fact;
    thread_local std::vector<std::uint32_t> stack;
    used.assign(ops_.size(), 0);
    seen_req.assign(reqs_.size(), 0);
    seen_fact.assign(facts_.size(), 0);

    // Every conjunct, the cheapest disjunct, and for each unmet fact its
    // supporting operator.
    std::uint32_t n = 0;
    stack.assign(goal_.begin(), goal_.end());
    while (!stack.empty()) {
        const std::uint32_t r = stack.back();
        stack.pop_back();
        if (seen_req[r] || rc[r] == 0 || rc[r] >= kInf) continue;
        seen_req[r] = 1;
        const Req& q = reqs_[r];
        switch (q.kind) {
            case Kind::And:
                for (std::uint32_t i = q.begin; i < q.end; ++i) stack.push_back(req_children_[i]);
                break;
            case Kind::Or: {
                std::uint32_t best = req_children_[q.begin];
                for (std::uint32_t i = q.begin; i < q.end; ++i)
                    if (rc[req_children_[i]] < rc[best]) best = req_children_[i];
                stack.push_back(best);
                break;
            }
            case Kind::Fact: {
                const std::uint32_t f = q.fact;
                if (seen_fact[f]) break;
                seen_fact[f] = 1;
                if (sup[f] >= 0) {
                    const Op& op = ops_[sup[f]];
                    if (!used[sup[f]]) {
                        used[sup[f]] = 1;
                        ++n;
                        if (helpful && rc[op.pre] == 0) helpful->push_back(op.action);
                    }
                    stack.push_back(op.pre);
                } else if (derived_[f] >= 0) {
                    stack.push_back(static_cast<std::uint32_t>(derived_[f]));
                }
                break;
            }
            default: break;
        }
    }
    return n;
}

KnowledgeRelaxationHeuristic::Analysis
KnowledgeRelaxationHeuristic::analyse(const EpistemicState& s) const {
    std::vector<std::int32_t> fc, rc, sup;
    Analysis a;
    a.rounds = costs(s, fc, rc, &sup);
    for (std::uint32_t r : goal_) {
        if (rc[r] >= kInf) ++a.dead_goals;
        else a.depth = std::max<std::uint32_t>(a.depth, static_cast<std::uint32_t>(rc[r]));
    }
    a.relaxed_plan = extract(rc, sup, nullptr);
    for (const auto& f : facts_) a.facts += f != nullptr;
    a.operators = static_cast<std::uint32_t>(ops_.size());
    return a;
}

float KnowledgeRelaxationHeuristic::operator()(const EpistemicState& s,
                                               const PlanningTask&) const {
    return estimate(initial_facts(s));
}

float KnowledgeRelaxationHeuristic::estimate(const std::vector<std::uint8_t>& init) const {
    thread_local std::vector<std::int32_t> fc, rc, sup;
    float h = 0.f;
    int dead = 0;
    if (estimate_ == Estimate::Add) {
        costs(init, fc, rc, nullptr);
        for (std::uint32_t r : goal_) { h += rc[r] >= kInf ? kDead : float(rc[r]); dead += rc[r] >= kInf; }
        last_dead_conjuncts = dead;
        return h;
    }
    costs(init, fc, rc, &sup);
    for (std::uint32_t r : goal_) { h += rc[r] >= kInf ? kDead : 0.f; dead += rc[r] >= kInf; }
    last_dead_conjuncts = dead;
    return h + float(extract(rc, sup, nullptr));
}

bool KnowledgeRelaxationHeuristic::preferred(const EpistemicState& s, const PlanningTask&,
                                             std::vector<ActionIdx>& out) const {
    return preferred_from(initial_facts(s), out);
}

bool KnowledgeRelaxationHeuristic::preferred_from(const std::vector<std::uint8_t>& init,
                                                  std::vector<ActionIdx>& out) const {
    thread_local std::vector<std::int32_t> fc, rc, sup;
    costs(init, fc, rc, &sup);
    out.clear();
    extract(rc, sup, &out);
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return true;
}
