#include "temporal_search.hpp"

#include "bisimulation.hpp"
#include "heuristic.hpp"
#include "world_cap_policy.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <fstream>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace temporal {

// ── Durations ───────────────────────────────────────────────────────────────

Durations Durations::uniform(const PlanningTask& task, double d)
{
    return Durations{std::vector<double>(task.actions.size(), d)};
}

Durations load_durations(const PlanningTask& task, const std::string& path, double fallback)
{
    if (!(fallback > 0.0)) throw std::invalid_argument("fallback duration must be positive");

    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot open durations file " + path);
    const nlohmann::json doc = nlohmann::json::parse(in);

    Durations out = Durations::uniform(task, fallback);

    const nlohmann::json* table = &doc;
    if (doc.contains("durations")) {
        table = &doc.at("durations");
    } else if (doc.contains("planning-task-info")) {
        return out;     // a task file without durations: every action untimed
    }
    if (!table->is_object()) throw std::runtime_error(path + ": durations must be an object");

    for (const auto& [name, value] : table->items()) {
        const auto it = task.action_index.find(name);
        if (it == task.action_index.end())
            throw std::runtime_error(path + ": no action named " + name + " in the task");

        double d = 0.0;
        if (value.is_number()) {
            d = value.get<double>();
        } else if (value.is_object() && value.contains("duration")) {
            d = value.at("duration").get<double>();
        } else if (value.is_object() || value.is_string()) {
            continue;   // a mapping entry without a duration keeps the fallback
        } else {
            throw std::runtime_error(path + ": " + name + " has no usable duration");
        }
        if (!(d > 0.0))
            throw std::runtime_error(path + ": " + name + " must take positive time");
        out.of[it->second] = d;
    }
    return out;
}

// ── Single-outcome updates and coherence ────────────────────────────────────

namespace {

WorldCapPolicy cap_of(const PlanningTask& task)
{
    return make_world_cap_policy(task.partial_obs);
}

// The model after `a` ends in `s`, when that is a single outcome.
std::optional<EpistemicState> end_of(const PlanningTask& task, const EpistemicState& s,
                                     ActionIdx a)
{
    auto outs = outcomes(task, s, task.actions[a], cap_of(task));
    if (outs.size() != 1) return std::nullopt;
    return std::move(outs.front().second);
}

bool single_outcome(const PlanningTask& task, const EpistemicState& s, ActionIdx a)
{
    return outcomes(task, s, task.actions[a], cap_of(task)).size() == 1;
}

// For each subset of `running` (bit i = running[i] has ended), the model once
// those have ended, in index order. Empty optional where an end fails. Under
// coherence the order does not matter, so one order per subset suffices.
std::vector<std::optional<EpistemicState>>
intermediates(const PlanningTask& task, const EpistemicState& s,
              const std::vector<ActionIdx>& running)
{
    const std::size_t n = running.size();
    std::vector<std::optional<EpistemicState>> at(std::size_t{1} << n);
    at[0] = s;
    for (std::size_t mask = 1; mask < at.size(); ++mask) {
        // Built from the subset without its highest member.
        std::size_t high = 0;
        for (std::size_t i = 0; i < n; ++i)
            if (mask & (std::size_t{1} << i)) high = i;
        const auto& base = at[mask & ~(std::size_t{1} << high)];
        if (base) at[mask] = end_of(task, *base, running[high]);
    }
    return at;
}

} // namespace

bool coherent(const PlanningTask& task, const EpistemicState& s,
              const std::vector<ActionIdx>& running)
{
    const auto at = intermediates(task, s, running);
    const std::size_t n = running.size();
    for (std::size_t mask = 0; mask < at.size(); ++mask) {
        if (!at[mask]) return false;
        for (std::size_t i = 0; i < n; ++i) {
            if (mask & (std::size_t{1} << i)) continue;
            if (!single_outcome(task, *at[mask], running[i])) return false;
            for (std::size_t j = i + 1; j < n; ++j) {
                if (mask & (std::size_t{1} << j)) continue;
                if (!commutes(task, *at[mask], running[i], running[j], cap_of(task)).commute())
                    return false;
            }
        }
    }
    return true;
}

namespace {

// May `b` start in `s` while `running` runs, given that `running` is coherent
// in `s`? Only the pairs involving b need checking: coherence of the rest is
// kept by the commutations checked here (b with every remaining action, in
// every intermediate state).
bool may_start(const PlanningTask& task, const EpistemicState& s,
               const std::vector<ActionIdx>& running,
               const std::vector<std::optional<EpistemicState>>& at, ActionIdx b,
               SearchStats& stats)
{
    ++stats.start_checks;
    const std::size_t n = running.size();
    for (std::size_t mask = 0; mask < at.size(); ++mask) {
        if (!at[mask] || !single_outcome(task, *at[mask], b)) return false;
        for (std::size_t i = 0; i < n; ++i) {
            if (mask & (std::size_t{1} << i)) continue;
            ++stats.commutation_checks;
            if (!commutes(task, *at[mask], b, running[i], cap_of(task)).commute()) return false;
        }
    }
    (void)s;
    return true;
}

struct Running {
    ActionIdx action;
    double    end;
};

// Times are compared and hashed after rounding to a microsecond, so that sums
// of decimal durations that differ in the last bit are one time.
std::int64_t ticks(double t) { return static_cast<std::int64_t>(std::llround(t * 1e6)); }

struct Node {
    EpistemicState       model;
    std::vector<Running> running;    // sorted by action index
    double               t{0.0};
    std::size_t          parent{std::numeric_limits<std::size_t>::max()};
    std::optional<Step>  started;     // the start move that produced this node
};

struct Key {
    Fingerprint                                     fp;
    std::vector<std::pair<ActionIdx, std::int64_t>> remaining;   // (action, end - t)

    friend bool operator==(const Key&, const Key&) = default;
};

struct KeyHash {
    std::size_t operator()(const Key& k) const noexcept
    {
        std::size_t h = FingerprintHash{}(k.fp);
        for (const auto& [a, r] : k.remaining)
            h ^= (std::hash<std::uint64_t>{}((std::uint64_t(a) << 40) ^ std::uint64_t(r)) +
                  0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2));
        return h;
    }
};

Key key_of(const Node& n)
{
    Key k{n.model.fingerprint(), {}};
    k.remaining.reserve(n.running.size());
    for (const auto& r : n.running) k.remaining.emplace_back(r.action, ticks(r.end - n.t));
    return k;
}

std::vector<ActionIdx> actions_of(const std::vector<Running>& running)
{
    std::vector<ActionIdx> out;
    out.reserve(running.size());
    for (const auto& r : running) out.push_back(r.action);
    return out;
}

} // namespace

std::optional<TemporalPlan>
search_linear(const PlanningTask& task, const Durations& durations,
              const SearchOptions& options, SearchStats* stats_out)
{
    SearchStats stats;
    const auto started_at = std::chrono::steady_clock::now();

    double min_duration = std::numeric_limits<double>::infinity();
    for (const double d : durations.of) min_duration = std::min(min_duration, d);

    // Admissible: the goal needs nothing running, so at least the latest end;
    // and with nothing running and the goal false, at least one more action.
    const auto estimate = [&](const Node& n) {
        if (!n.running.empty()) {
            double latest = n.t;
            for (const auto& r : n.running) latest = std::max(latest, r.end);
            return latest - n.t;
        }
        return n.model.satisfies(*task.goal) ? 0.0 : min_duration;
    };

    std::vector<Node> nodes;
    std::unordered_map<Key, double, KeyHash> best_g;

    using Entry = std::pair<double, std::size_t>;   // (f, node index); min-heap
    std::priority_queue<Entry, std::vector<Entry>, std::greater<>> open;

    const auto push = [&](Node node) {
        ++stats.generated;
        Key key = key_of(node);
        const auto it = best_g.find(key);
        if (it != best_g.end() && it->second <= node.t + 1e-9) {
            ++stats.duplicates;
            return;
        }
        best_g[std::move(key)] = node.t;
        const double f = node.t + options.weight * estimate(node);
        nodes.push_back(std::move(node));
        open.emplace(f, nodes.size() - 1);
    };

    Node root;
    root.model = bisim_contract(task.init);
    push(std::move(root));

    std::optional<std::size_t> goal;
    while (!open.empty()) {
        if (std::chrono::steady_clock::now() >= options.deadline) break;
        if (options.max_expansions && stats.expanded >= options.max_expansions) break;

        const std::size_t idx = open.top().second;
        open.pop();

        // A stale entry: the same key was reached sooner since it was queued.
        {
            const auto it = best_g.find(key_of(nodes[idx]));
            if (it != best_g.end() && it->second < nodes[idx].t - 1e-9) continue;
        }

        if (nodes[idx].running.empty() && nodes[idx].model.satisfies(*task.goal)) {
            goal = idx;
            break;
        }
        ++stats.expanded;

        // Copies: pushing may reallocate `nodes`.
        const EpistemicState model   = nodes[idx].model;
        const std::vector<Running> running = nodes[idx].running;
        const double t = nodes[idx].t;

        // start b
        if (running.size() < options.max_concurrency) {
            const std::vector<ActionIdx> run = actions_of(running);
            const auto at = intermediates(task, model, run);
            for (ActionIdx b = 0; b < task.actions.size(); ++b) {
                if (std::find(run.begin(), run.end(), b) != run.end()) continue;
                if (!task.actions[b].applicable(model)) continue;
                if (!may_start(task, model, run, at, b, stats)) {
                    ++stats.start_rejected;
                    continue;
                }
                Node next;
                next.model   = model;
                next.running = running;
                next.running.push_back({b, t + durations[b]});
                std::sort(next.running.begin(), next.running.end(),
                          [](const Running& x, const Running& y) { return x.action < y.action; });
                next.t       = t;
                next.parent  = idx;
                next.started = Step{b, t, durations[b]};
                push(std::move(next));
            }
        }

        // advance
        if (!running.empty()) {
            double t_next = std::numeric_limits<double>::infinity();
            for (const auto& r : running) t_next = std::min(t_next, r.end);

            Node next;
            next.model = model;
            next.t     = t_next;
            bool ok    = true;
            for (const auto& r : running) {     // index order; coherence makes it irrelevant
                if (ticks(r.end) == ticks(t_next)) {
                    auto after = end_of(task, next.model, r.action);
                    if (!after) { ok = false; break; }
                    next.model = std::move(*after);
                } else {
                    next.running.push_back(r);
                }
            }
            if (ok) {
                next.parent = idx;
                push(std::move(next));
            }
        }
    }

    stats.exhausted   = open.empty() && !goal;
    stats.elapsed_sec = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                      started_at).count();
    if (stats_out) *stats_out = stats;
    if (!goal) return std::nullopt;

    TemporalPlan plan;
    plan.makespan = nodes[*goal].t;
    for (std::size_t i = *goal; i != std::numeric_limits<std::size_t>::max(); i = nodes[i].parent)
        if (nodes[i].started) plan.steps.push_back(*nodes[i].started);
    std::sort(plan.steps.begin(), plan.steps.end(), [](const Step& x, const Step& y) {
        if (ticks(x.start) != ticks(y.start)) return x.start < y.start;
        return x.action < y.action;
    });
    return plan;
}

bool validate_linear(const PlanningTask& task, const Durations& durations,
                     const TemporalPlan& plan, std::string* why)
{
    const auto fail = [&](const std::string& message) {
        if (why) *why = message;
        return false;
    };

    std::vector<Step> steps = plan.steps;
    std::sort(steps.begin(), steps.end(), [](const Step& x, const Step& y) {
        return x.start < y.start;
    });

    EpistemicState model = bisim_contract(task.init);
    std::vector<Running> running;
    double makespan = 0.0;

    // Ends at time t are applied before starts at t: an action that starts as
    // another ends sees the ended one's update.
    const auto end_until = [&](double t) -> bool {
        while (true) {
            double t_next = std::numeric_limits<double>::infinity();
            for (const auto& r : running) t_next = std::min(t_next, r.end);
            if (running.empty()) return true;
            if (std::isfinite(t) && ticks(t_next) > ticks(t)) return true;
            std::vector<Running> rest;
            for (const auto& r : running) {
                if (ticks(r.end) == ticks(t_next)) {
                    auto after = end_of(task, model, r.action);
                    if (!after) {
                        if (why) *why = task.actions[r.action].name + " has no single outcome at its end";
                        return false;
                    }
                    model = std::move(*after);
                } else {
                    rest.push_back(r);
                }
            }
            running = std::move(rest);
        }
    };

    for (const auto& step : steps) {
        if (std::abs(step.duration - durations[step.action]) > 1e-9)
            return fail(task.actions[step.action].name + " has the wrong duration");
        if (!end_until(step.start)) return false;
        if (!task.actions[step.action].applicable(model))
            return fail(task.actions[step.action].name + " is not applicable at its start");
        running.push_back({step.action, step.end()});
        if (!coherent(task, model, actions_of(running)))
            return fail("starting " + task.actions[step.action].name +
                        " makes the running set incoherent");
        makespan = std::max(makespan, step.end());
    }
    if (!end_until(std::numeric_limits<double>::infinity())) return false;
    if (!model.satisfies(*task.goal)) return fail("the goal does not hold at the end");
    if (ticks(makespan) != ticks(plan.makespan)) return fail("the makespan is wrong");
    return true;
}


// ── Branching ───────────────────────────────────────────────────────────────

namespace {

using Observed = std::vector<std::pair<ActionIdx, EventIdx>>;

// For each subset of `running` ended, every state the model can be in: one
// per combination of the ended actions' outcomes. nullopt where some ended
// action has no outcome at all.
std::optional<std::vector<std::vector<EpistemicState>>>
branch_intermediates(const PlanningTask& task, const EpistemicState& s,
                     const std::vector<ActionIdx>& running)
{
    const std::size_t n = running.size();
    std::vector<std::vector<EpistemicState>> at(std::size_t{1} << n);
    at[0] = {s};
    for (std::size_t mask = 1; mask < at.size(); ++mask) {
        std::size_t high = 0;
        for (std::size_t i = 0; i < n; ++i)
            if (mask & (std::size_t{1} << i)) high = i;
        for (const auto& base : at[mask & ~(std::size_t{1} << high)]) {
            auto outs = outcomes(task, base, task.actions[running[high]], cap_of(task));
            if (outs.empty()) return std::nullopt;
            for (auto& [e, st] : outs) at[mask].push_back(std::move(st));
        }
    }
    return at;
}

bool coherent_branching(const PlanningTask& task, const EpistemicState& s,
                        const std::vector<ActionIdx>& running)
{
    const auto at = branch_intermediates(task, s, running);
    if (!at) return false;
    const std::size_t n = running.size();
    for (std::size_t mask = 0; mask < at->size(); ++mask) {
        for (const auto& st : (*at)[mask]) {
            for (std::size_t i = 0; i < n; ++i) {
                if (mask & (std::size_t{1} << i)) continue;
                if (outcomes(task, st, task.actions[running[i]], cap_of(task)).empty())
                    return false;
                for (std::size_t j = i + 1; j < n; ++j) {
                    if (mask & (std::size_t{1} << j)) continue;
                    if (!commutes(task, st, running[i], running[j], cap_of(task)).commute())
                        return false;
                }
            }
        }
    }
    return true;
}

bool may_start_branching(const PlanningTask& task,
                         const std::vector<std::vector<EpistemicState>>& at,
                         const std::vector<ActionIdx>& running, ActionIdx b, SearchStats& stats)
{
    ++stats.start_checks;
    const std::size_t n = running.size();
    for (std::size_t mask = 0; mask < at.size(); ++mask) {
        for (const auto& st : at[mask]) {
            if (outcomes(task, st, task.actions[b], cap_of(task)).empty()) return false;
            for (std::size_t i = 0; i < n; ++i) {
                if (mask & (std::size_t{1} << i)) continue;
                ++stats.commutation_checks;
                if (!commutes(task, st, b, running[i], cap_of(task)).commute()) return false;
            }
        }
    }
    return true;
}

struct BNode {
    EpistemicState       model;
    std::vector<Running> running;     // by action index
    double               t{0.0};
};

Key key_of(const BNode& n)
{
    Key k{n.model.fingerprint(), {}};
    for (const auto& r : n.running) k.remaining.emplace_back(r.action, ticks(r.end - n.t));
    return k;
}

BNode started(const BNode& n, ActionIdx b, double duration)
{
    BNode next{n.model, n.running, n.t};
    next.running.push_back({b, n.t + duration});
    std::sort(next.running.begin(), next.running.end(),
              [](const Running& x, const Running& y) { return x.action < y.action; });
    return next;
}

// Advance to the earliest end: one child per combination of the observed
// outcomes of the actions ending then. nullopt if an end has no outcome.
std::optional<std::vector<std::pair<Observed, BNode>>> advanced(const PlanningTask& task,
                                                                 const BNode& n)
{
    double t_next = std::numeric_limits<double>::infinity();
    for (const auto& r : n.running) t_next = std::min(t_next, r.end);

    std::vector<Running> rest;
    std::vector<ActionIdx> ending;
    for (const auto& r : n.running) {
        if (ticks(r.end) == ticks(t_next)) ending.push_back(r.action);
        else rest.push_back(r);
    }

    std::vector<std::pair<Observed, EpistemicState>> states{{{}, n.model}};
    for (const ActionIdx a : ending) {
        std::vector<std::pair<Observed, EpistemicState>> next;
        for (auto& [obs, st] : states) {
            auto outs = outcomes(task, st, task.actions[a], cap_of(task));
            if (outs.empty()) return std::nullopt;
            for (auto& [e, after] : outs) {
                Observed o = obs;
                if (outs.size() > 1) o.emplace_back(a, e);
                next.emplace_back(std::move(o), std::move(after));
            }
        }
        states = std::move(next);
    }

    std::vector<std::pair<Observed, BNode>> out;
    out.reserve(states.size());
    for (auto& [obs, st] : states) out.emplace_back(std::move(obs), BNode{std::move(st), rest, t_next});
    return out;
}

struct OutOfTime {};

class PolicySearch {
public:
    PolicySearch(const PlanningTask& task, const Durations& d, const SearchOptions& o,
                 SearchStats& stats)
    : task_(task), d_(d), o_(o), stats_(stats)
    {
        for (const double x : d_.of) {
            min_duration_ = std::min(min_duration_, x);
            max_duration_ = std::max(max_duration_, x);
        }
    }

    struct Result {
        bool   solved;
        double value;     // the exact least cost if solved, else a lower bound above the bound
    };

    Result solve(const BNode& n, double bound)
    {
        if (std::chrono::steady_clock::now() >= o_.deadline) throw OutOfTime{};
        if (o_.max_expansions && stats_.expanded >= o_.max_expansions) throw OutOfTime{};

        const Key key = key_of(n);
        if (const auto it = memo_.find(key); it != memo_.end()) {
            const Memo& m = it->second;
            if (m.solved) return {m.cost <= bound + kEps, m.cost};
            if (m.lower > bound + kEps) return {false, m.lower};
        }

        if (n.running.empty() && n.model.satisfies(*task_.goal)) {
            memo_[key] = Memo{true, 0.0, 0.0, kGoal};
            return {true, 0.0};
        }

        const double h = estimate(n);
        if (h > bound + kEps) return {false, h};
        ++stats_.expanded;

        double best = kInf;
        int    move = kNone;
        double lower = kInf;

        // start b: no time passes. Which starts are allowed is decided first,
        // and timed on its own, before any of them is searched.
        if (n.running.size() < o_.max_concurrency) {
            const auto run = actions_of(n.running);
            std::vector<ActionIdx> allowed;
            const auto checking = std::chrono::steady_clock::now();
            if (o_.overlap == SearchOptions::Overlap::Syntactic) {
                for (ActionIdx b = 0; b < task_.actions.size(); ++b) {
                    if (std::find(run.begin(), run.end(), b) != run.end()) continue;
                    if (!task_.actions[b].applicable(n.model)) continue;
                    ++stats_.start_checks;
                    const bool independent = std::all_of(run.begin(), run.end(), [&](ActionIdx r) {
                        return syntactically_independent(task_.actions[b], task_.actions[r]);
                    });
                    if (!independent) {
                        ++stats_.start_rejected;
                        continue;
                    }
                    allowed.push_back(b);
                }
            } else if (const auto at = branch_intermediates(task_, n.model, run)) {
                for (ActionIdx b = 0; b < task_.actions.size(); ++b) {
                    if (std::find(run.begin(), run.end(), b) != run.end()) continue;
                    if (!task_.actions[b].applicable(n.model)) continue;
                    if (!may_start_branching(task_, *at, run, b, stats_)) {
                        ++stats_.start_rejected;
                        continue;
                    }
                    allowed.push_back(b);
                }
            }
            stats_.start_check_sec += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - checking).count();

            for (const ActionIdx b : allowed) {
                ++stats_.generated;
                const Result r = solve(started(n, b, d_[b]), std::min(bound, best));
                if (r.solved && r.value < best - kEps) {
                    best = r.value;
                    move = static_cast<int>(b);
                } else if (!r.solved) {
                    lower = std::min(lower, r.value);
                }
            }
        }

        // advance: an AND over what the ending actions observe.
        if (!n.running.empty()) {
            if (const auto children = advanced(task_, n)) {
                const double dt = children->front().second.t - n.t;
                const double budget = std::min(bound, best) - dt;
                double worst = 0.0;
                bool   all   = true;
                for (const auto& [obs, child] : *children) {
                    ++stats_.generated;
                    const Result r = solve(child, budget);
                    if (!r.solved) {
                        all = false;
                        lower = std::min(lower, dt + r.value);
                        break;
                    }
                    worst = std::max(worst, r.value);
                }
                if (all && dt + worst < best - kEps) {
                    best = dt + worst;
                    move = kAdvance;
                }
            }
        }

        if (best <= bound + kEps) {
            memo_[key] = Memo{true, best, best, move};
            return {true, best};
        }
        Memo& m = memo_[key];
        m.solved = false;
        m.lower  = std::max(m.lower, lower);
        return {false, lower};
    }

    std::shared_ptr<const Epoch> build(BNode n, std::size_t& leaves) const
    {
        auto epoch = std::make_shared<Epoch>();
        epoch->t = n.t;
        while (true) {
            const Memo& m = memo_.at(key_of(n));
            if (m.move == kGoal) {
                epoch->advance_to = n.t;
                ++leaves;
                return epoch;
            }
            if (m.move >= 0) {
                const auto b = static_cast<ActionIdx>(m.move);
                epoch->starts.push_back(Step{b, n.t, d_[b]});
                n = started(n, b, d_[b]);
                continue;
            }
            const auto children = advanced(task_, n);
            epoch->advance_to = children->front().second.t;
            for (const auto& [obs, child] : *children)
                epoch->branches.push_back(Branch{obs, build(child, leaves)});
            std::sort(epoch->starts.begin(), epoch->starts.end(),
                      [](const Step& x, const Step& y) { return x.action < y.action; });
            return epoch;
        }
    }

    double estimate(const BNode& n) const
    {
        double admissible = 0.0;
        if (!n.running.empty()) {
            double latest = n.t;
            for (const auto& r : n.running) latest = std::max(latest, r.end);
            admissible = latest - n.t;
        } else if (!n.model.satisfies(*task_.goal)) {
            admissible = min_duration_;
        }
        if (o_.estimate == SearchOptions::Estimate::Admissible) return admissible;

        // h_cp is a fixpoint over the model, so it is cached by the same key
        // the results are memoised under.
        const Key key = key_of(n);
        auto it = critical_path_.find(key);
        if (it == critical_path_.end())
            it = critical_path_.emplace(key, critical_path(n)).first;
        return std::max(admissible, it->second);
    }

    // The relaxed closure's layers, each weighed by the shortest duration
    // among the actions that contribute to it; a running action counts with
    // its remaining time. A closure that never resolves the goal costs one
    // longest action past what it did resolve.
    double critical_path(const BNode& n) const
    {
        const auto layers = relaxed_closure_layers(n.model, task_);
        if (!layers) return max_duration_;
        double total = 0.0;
        for (const auto& layer : *layers) {
            double cheapest = kInf;
            for (const ActionIdx a : layer) {
                double d = d_[a];
                for (const auto& r : n.running)
                    if (r.action == a) d = r.end - n.t;
                cheapest = std::min(cheapest, d);
            }
            if (std::isfinite(cheapest)) total += cheapest;
        }
        return total;
    }

private:
    static constexpr double kEps  = 1e-9;
    static constexpr double kInf  = std::numeric_limits<double>::infinity();
    static constexpr int    kNone = -3, kGoal = -2, kAdvance = -1;

    struct Memo {
        bool   solved{false};
        double cost{kInf};
        double lower{0.0};
        int    move{kNone};
    };

    const PlanningTask&  task_;
    const Durations&     d_;
    const SearchOptions& o_;
    SearchStats&         stats_;
    double               min_duration_{kInf};
    double               max_duration_{0.0};
    std::unordered_map<Key, Memo, KeyHash> memo_;
    mutable std::unordered_map<Key, double, KeyHash> critical_path_;
};

} // namespace

std::optional<TemporalPolicy>
search_policy(const PlanningTask& task, const Durations& durations, const SearchOptions& options,
              SearchStats* stats_out)
{
    SearchStats stats;
    const auto started_at = std::chrono::steady_clock::now();
    PolicySearch search(task, durations, options, stats);

    BNode root{bisim_contract(task.init), {}, 0.0};
    std::optional<TemporalPolicy> policy;
    try {
        double bound = search.estimate(root);
        while (true) {
            const auto r = search.solve(root, bound);
            if (r.solved) {
                TemporalPolicy p;
                p.makespan = r.value;
                p.root     = search.build(root, p.leaves);
                policy     = std::move(p);
                break;
            }
            if (!std::isfinite(r.value)) {
                stats.exhausted = true;
                break;
            }
            bound = r.value;
        }
    } catch (const OutOfTime&) {
    }

    stats.elapsed_sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
    if (stats_out) *stats_out = stats;
    return policy;
}

bool validate_policy(const PlanningTask& task, const Durations& durations,
                     const TemporalPolicy& policy, std::string* why)
{
    const auto fail = [&](const std::string& message) {
        if (why) *why = message;
        return false;
    };
    if (!policy.root) return fail("the policy is empty");

    double worst = 0.0;
    const std::function<bool(const Epoch&, BNode)> check = [&](const Epoch& e, BNode n) -> bool {
        if (ticks(e.t) != ticks(n.t)) return fail("an epoch is at the wrong time");
        for (const auto& step : e.starts) {
            const std::string& name = task.actions[step.action].name;
            if (ticks(step.start) != ticks(n.t)) return fail(name + " starts off its epoch");
            if (std::abs(step.duration - durations[step.action]) > 1e-9)
                return fail(name + " has the wrong duration");
            if (!task.actions[step.action].applicable(n.model))
                return fail(name + " is not applicable at its start");
            n = started(n, step.action, step.duration);
            if (!coherent_branching(task, n.model, actions_of(n.running)))
                return fail("starting " + name + " makes the running set incoherent");
        }
        if (e.branches.empty()) {
            if (!n.running.empty()) return fail("a leaf has actions still running");
            if (!n.model.satisfies(*task.goal)) return fail("the goal does not hold at a leaf");
            worst = std::max(worst, n.t);
            return true;
        }
        if (n.running.empty()) return fail("an advance with nothing running");
        const auto children = advanced(task, n);
        if (!children) return fail("an action ending has no outcome");
        if (ticks(children->front().second.t) != ticks(e.advance_to))
            return fail("an advance goes to the wrong time");
        if (children->size() != e.branches.size())
            return fail("an advance has " + std::to_string(children->size()) +
                        " possible outcome combinations but " +
                        std::to_string(e.branches.size()) + " branches");
        for (const auto& [obs, child] : *children) {
            const auto it = std::find_if(e.branches.begin(), e.branches.end(),
                                         [&](const Branch& b) { return b.observed == obs; });
            if (it == e.branches.end() || !it->next)
                return fail("an outcome combination has no branch");
            if (!check(*it->next, child)) return false;
        }
        return true;
    };

    if (!check(*policy.root, BNode{bisim_contract(task.init), {}, 0.0})) return false;
    if (ticks(worst) != ticks(policy.makespan)) return fail("the worst-case makespan is wrong");
    return true;
}

} // namespace temporal
