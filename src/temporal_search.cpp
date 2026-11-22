#include "temporal_search.hpp"

#include "bisimulation.hpp"
#include "world_cap_policy.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
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

} // namespace temporal
