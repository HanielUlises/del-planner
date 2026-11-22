#include "temporal.hpp"

#include "bisimulation.hpp"
#include "product_update.hpp"

#include <algorithm>
#include <map>

namespace temporal {

std::vector<std::pair<EventIdx, EpistemicState>>
outcomes(const PlanningTask& task, const EpistemicState& s, const Action& a,
         const WorldCapPolicy& cap)
{
    std::vector<std::pair<EventIdx, EpistemicState>> out;
    if (!a.applicable(s)) return out;

    auto branches = product_update_split(s, a, task.seriality(), cap);
    out.reserve(branches.size());
    for (auto& [event, state] : branches) {
        // A branch whose designated set is empty is an outcome that cannot
        // occur from here, not a state anyone would reach.
        if (state.num_designated() == 0) continue;
        out.emplace_back(event, bisim_contract(std::move(state)));
    }
    return out;
}

const char* verdict_name(Verdict v) noexcept
{
    switch (v) {
        case Verdict::Commute:         return "commute";
        case Verdict::Inapplicable:    return "inapplicable";
        case Verdict::DisabledByFirst: return "disabled-by-first";
        case Verdict::OutcomesDiffer:  return "outcomes-differ";
        case Verdict::ResultsDiffer:   return "results-differ";
    }
    return "unknown";
}

namespace {

std::string event_name(const Action& a, EventIdx e)
{
    if (e < a.events.size() && !a.events[e].name.empty()) return a.events[e].name;
    return "e" + std::to_string(e);
}

} // namespace

std::string Commutation::explain(const PlanningTask& task) const
{
    const Action& f = task.actions[first];
    const Action& s = task.actions[second];
    switch (verdict) {
        case Verdict::Commute:
            return f.name + " and " + s.name + " commute (" + std::to_string(pairs) +
                   " outcome pair(s), " + std::to_string(updates) + " update(s))";
        case Verdict::Inapplicable:
            return f.name + " is not applicable here, so it cannot overlap " + s.name;
        case Verdict::DisabledByFirst:
            return s.name + " is not applicable after " + f.name + " -> " +
                   event_name(f, first_event);
        case Verdict::OutcomesDiffer:
            return "after " + f.name + " -> " + event_name(f, first_event) + ", " + s.name +
                   " -> " + event_name(s, second_event) +
                   " is reachable in this order and not in the other";
        case Verdict::ResultsDiffer:
            return f.name + " -> " + event_name(f, first_event) + " then " + s.name + " -> " +
                   event_name(s, second_event) +
                   " is not bisimilar to the other order";
    }
    return "unknown";
}

Commutation commutes(const PlanningTask& task, const EpistemicState& s, ActionIdx a,
                     ActionIdx b, const WorldCapPolicy& cap)
{
    Commutation result;
    result.first  = a;
    result.second = b;

    const Action& act_a = task.actions[a];
    const Action& act_b = task.actions[b];

    const auto out_a = outcomes(task, s, act_a, cap);
    const auto out_b = outcomes(task, s, act_b, cap);
    result.updates += 2;

    if (out_a.empty() || out_b.empty()) {
        result.verdict = Verdict::Inapplicable;
        if (out_a.empty()) {
            result.first = a; result.second = b;
        } else {
            result.first = b; result.second = a;
        }
        return result;
    }

    // Keyed by (outcome of a, outcome of b) in both maps, whichever ran first.
    using Pair = std::pair<EventIdx, EventIdx>;
    std::map<Pair, Fingerprint> a_then_b;
    std::map<Pair, Fingerprint> b_then_a;

    for (const auto& [e, after_a] : out_a) {
        const auto next = outcomes(task, after_a, act_b, cap);
        ++result.updates;
        if (next.empty()) {
            result.verdict     = Verdict::DisabledByFirst;
            result.first       = a;
            result.second      = b;
            result.first_event = e;
            return result;
        }
        for (const auto& [f, state] : next) a_then_b[{e, f}] = state.fingerprint();
    }

    for (const auto& [f, after_b] : out_b) {
        const auto next = outcomes(task, after_b, act_a, cap);
        ++result.updates;
        if (next.empty()) {
            result.verdict     = Verdict::DisabledByFirst;
            result.first       = b;
            result.second      = a;
            result.first_event = f;
            return result;
        }
        for (const auto& [e, state] : next) b_then_a[{e, f}] = state.fingerprint();
    }

    // The same outcome pairs must be possible whichever ends first: a pair
    // possible in one order only is a branch the other order cannot take.
    for (const auto& [key, fp] : a_then_b) {
        if (!b_then_a.count(key)) {
            result.verdict      = Verdict::OutcomesDiffer;
            result.first        = a;
            result.second       = b;
            result.first_event  = key.first;
            result.second_event = key.second;
            return result;
        }
    }
    for (const auto& [key, fp] : b_then_a) {
        if (!a_then_b.count(key)) {
            result.verdict      = Verdict::OutcomesDiffer;
            result.first        = b;
            result.second       = a;
            result.first_event  = key.second;
            result.second_event = key.first;
            return result;
        }
    }

    result.pairs = a_then_b.size();
    for (const auto& [key, fp] : a_then_b) {
        if (!(b_then_a.at(key) == fp)) {
            result.verdict      = Verdict::ResultsDiffer;
            result.first        = a;
            result.second       = b;
            result.first_event  = key.first;
            result.second_event = key.second;
            return result;
        }
    }

    result.verdict = Verdict::Commute;
    return result;
}

Commutation commutes(const PlanningTask& task, const EpistemicState& s, ActionIdx a,
                     ActionIdx b)
{
    return commutes(task, s, a, b, make_world_cap_policy(task.partial_obs));
}

namespace {

void atoms_of(const FormulaPtr& f, std::vector<AtomIdx>& out)
{
    if (!f) return;
    if (f->kind == FormulaKind::Atom) out.push_back(f->atom);
    for (const auto& child : f->children) atoms_of(child, out);
}

void sort_unique(std::vector<AtomIdx>& v)
{
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
}

} // namespace

AtomFootprint footprint(const Action& a)
{
    AtomFootprint fp;
    for (const auto& event : a.events) {
        atoms_of(event.precondition, fp.reads);
        for (const auto& [atom, guard] : event.post_true) {
            fp.writes.push_back(atom);
            atoms_of(guard, fp.reads);
        }
        for (const auto& [atom, guard] : event.post_false) {
            fp.writes.push_back(atom);
            atoms_of(guard, fp.reads);
        }
    }
    for (const auto& cases : a.obs_cases)
        for (const auto& c : cases) atoms_of(c.condition, fp.reads);
    atoms_of(a.default_obs.condition, fp.reads);

    sort_unique(fp.reads);
    sort_unique(fp.writes);
    return fp;
}

bool syntactically_independent(const Action& a, const Action& b)
{
    const auto fa = footprint(a);
    const auto fb = footprint(b);

    const auto meets = [](const std::vector<AtomIdx>& x, const std::vector<AtomIdx>& y) {
        auto i = x.begin();
        auto j = y.begin();
        while (i != x.end() && j != y.end()) {
            if (*i == *j) return true;
            if (*i < *j) ++i; else ++j;
        }
        return false;
    };

    return !meets(fa.writes, fb.reads) && !meets(fa.writes, fb.writes) &&
           !meets(fb.writes, fa.reads);
}

} // namespace temporal
