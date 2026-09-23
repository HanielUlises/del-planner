#include "parser.hpp"

#include <algorithm>
#include "formula.hpp"
#include "heuristic.hpp"
#include "json.hpp"
#include <charconv>
#include <string_view>
#include <stdexcept>
#include <iostream>


/**
 * @brief Parse a logical formula from its JSON representation.
 *
 * Supports atomic propositions, boolean constants, logical connectives
 * (negation, conjunction, disjunction), and modal operators (box, diamond).
 *
 * @param j JSON object encoding the formula.
 * @param atom_idx Mapping from atom names to indices.
 * @param agent_idx Mapping from agent names to indices.
 * @return Parsed formula as a FormulaPtr.
 *
 * @throws std::runtime_error if the formula is malformed or references unknown symbols.
 */
static FormulaPtr parse_formula(
    const json::Value& j,
    const std::unordered_map<std::string,AtomIdx>&  atom_idx,
    const std::unordered_map<std::string,AgentIdx>& agent_idx)
{
    if (j.is_string()) {
        std::string s = j.get<std::string>();
        if (s == "true")  return Formula::make_top();
        if (s == "false") return Formula::make_bot();

        auto it = atom_idx.find(s);
        if (it == atom_idx.end())
            throw std::runtime_error("Unknown atom: " + s);

        return Formula::make_atom(it->second);
    }

    if (!j.is_object())
        throw std::runtime_error("Expected formula object or string");

    if (j.contains("connective")) {
        std::string conn = j.at("connective").get<std::string>();

        if (conn == "not") {
            return Formula::make_not(
                parse_formula(j.at("formula"), atom_idx, agent_idx));
        }

        if (conn == "and") {
            std::vector<FormulaPtr> children;
            for (const auto c : j.at("formulas"))
                children.push_back(parse_formula(c, atom_idx, agent_idx));
            return Formula::make_and(std::move(children));
        }

        if (conn == "or") {
            std::vector<FormulaPtr> children;
            for (const auto c : j.at("formulas"))
                children.push_back(parse_formula(c, atom_idx, agent_idx));
            return Formula::make_or(std::move(children));
        }

        // imply: p -> q  ≡  ¬p ∨ q
        if (conn == "imply") {
            auto lhs = parse_formula(j.at("formulas")[0], atom_idx, agent_idx);
            auto rhs = parse_formula(j.at("formulas")[1], atom_idx, agent_idx);
            return Formula::make_or({Formula::make_not(lhs), rhs});
        }

        // forall over agents: universally quantified conjunction
        // plank expands these at ground time so this is a fallback for
        // any residual quantified formulas in the JSON
        if (conn == "forall") {
            if (j.contains("formulas")) {
                std::vector<FormulaPtr> children;
                for (const auto c : j.at("formulas"))
                    children.push_back(parse_formula(c, atom_idx, agent_idx));
                return Formula::make_and(std::move(children));
            }
            return parse_formula(j.at("formula"), atom_idx, agent_idx);
        }

        throw std::runtime_error("Unknown connective: " + conn);
    }

    if (j.contains("modality-name")) {
        std::string mname = j.at("modality-name").get<std::string>();
        const auto midx = j.at("modality-index");

        FormulaPtr child =
            parse_formula(j.at("formula"), atom_idx, agent_idx);

        // Group modalities follow plank's model checker: [G]φ and <G>φ quantify
        // over each agent of G separately, [G]φ ≡ ∧_{i∈G} [i]φ and
        // <G>φ ≡ ∧_{i∈G} <i>φ. Common knowledge is the separate "C.box".
        auto group_of = [&](const char* what) {
            std::vector<AgentIdx> grp;
            for (const auto a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error(std::string("Unknown agent in ") + what + ": " + aname);
                grp.push_back(it->second);
            }
            if (grp.empty())
                throw std::runtime_error(std::string("Empty modality index in ") + what);
            return grp;
        };

        if (mname == "box") {
            std::vector<FormulaPtr> conjuncts;
            for (AgentIdx ag : group_of("box"))
                conjuncts.push_back(Formula::make_belief(ag, child));
            return conjuncts.size() == 1 ? conjuncts[0] : Formula::make_and(std::move(conjuncts));
        }

        if (mname == "diamond") {
            std::vector<FormulaPtr> conjuncts;
            for (AgentIdx ag : group_of("diamond"))
                conjuncts.push_back(Formula::make_not(
                    Formula::make_belief(ag, Formula::make_not(child))));
            return conjuncts.size() == 1 ? conjuncts[0] : Formula::make_and(std::move(conjuncts));
        }

        // [Kw.i]φ  ≡  [i]φ ∨ [i]¬φ  (knowing-whether)
        // [Kw.G]φ  ≡  ∧_{i∈G} ([i]φ ∨ [i]¬φ)  (group knowing-whether)
        if (mname == "Kw.box") {
            if (midx.size() == 1) {
                std::string aname = midx[0].get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent: " + aname);
                return Formula::make_kw(it->second, child);
            }
            if (midx.size() > 1) {
                std::vector<FormulaPtr> conjuncts;
                for (const auto a : midx) {
                    std::string aname = a.get<std::string>();
                    auto it = agent_idx.find(aname);
                    if (it == agent_idx.end())
                        throw std::runtime_error("Unknown agent in Kw.box group: " + aname);
                    conjuncts.push_back(Formula::make_kw(it->second, child));
                }
                return Formula::make_and(std::move(conjuncts));
            }
        }

        // <Kw.i>φ  ≡  ¬([i]φ ∨ [i]¬φ)  (not knowing-whether)
        // <Kw.G>φ  ≡  ∧_{i∈G} ¬([i]φ ∨ [i]¬φ), as in plank: every agent of G is
        // uncertain, not merely some agent.
        if (mname == "Kw.diamond") {
            std::vector<FormulaPtr> conjuncts;
            for (AgentIdx ag : group_of("Kw.diamond"))
                conjuncts.push_back(Formula::make_not(Formula::make_kw(ag, child)));
            return conjuncts.size() == 1 ? conjuncts[0] : Formula::make_and(std::move(conjuncts));
        }

        // C.box — common knowledge/belief over a group
        // plank emits "C.box" with modality-index listing all group agents
        if (mname == "C.box") {
            std::vector<AgentIdx> grp;
            for (const auto a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent in C.box group: " + aname);
                grp.push_back(it->second);
            }
            return Formula::make_common(std::move(grp), child);
        }

        // C.diamond — dual: ¬C.box¬φ
        if (mname == "C.diamond") {
            std::vector<AgentIdx> grp;
            for (const auto a : midx) {
                std::string aname = a.get<std::string>();
                auto it = agent_idx.find(aname);
                if (it == agent_idx.end())
                    throw std::runtime_error("Unknown agent in C.diamond group: " + aname);
                grp.push_back(it->second);
            }
            return Formula::make_not(
                Formula::make_common(std::move(grp), Formula::make_not(child)));
        }

        throw std::runtime_error("Unknown modality: " + mname);
    }

    throw std::runtime_error("Unrecognised formula shape");
}

/**
 * @brief Extract and parse a formula, unwrapping a "formula" field if present.
 *
 * Some JSON structures wrap formulas inside a "formula" key. This function
 * transparently handles both wrapped and direct representations.
 *
 * @param j JSON object containing the formula or wrapper.
 * @param atom_idx Mapping from atom names to indices.
 * @param agent_idx Mapping from agent names to indices.
 * @return Parsed formula as a FormulaPtr.
 */
static FormulaPtr unwrap_formula(
    const json::Value& j,
    const std::unordered_map<std::string,AtomIdx>&  atom_idx,
    const std::unordered_map<std::string,AgentIdx>& agent_idx)
{
    if (j.contains("formula"))
        return parse_formula(j.at("formula"), atom_idx, agent_idx);

    return parse_formula(j, atom_idx, agent_idx);
}

/**
 * @brief Load a planning task from a JSON file.
 *
 * The input JSON is expected to follow a structured format including:
 * - Language definition (atoms and agents)
 * - Requirements (used to detect KD45 vs S5 frame constraints)
 * - Initial epistemic state (worlds, valuations, accessibility relations)
 * - Actions with events, preconditions, effects, and observability conditions
 * - Goal formula
 *
 * The kd45 flag is set when the requirements contain ":kd45" or ":belief",
 * indicating that all accessibility relations must be serial (KD45n frame).
 * Product update will enforce seriality on the resulting state when this flag
 * is true, preventing vacuous-truth errors from worlds with empty R_i rows.
 *
 * @param json_path Path to the JSON file.
 * @return Fully constructed PlanningTask instance.
 *
 * @throws std::runtime_error if the file cannot be read or parsing fails.
 */
PlanningTask load_task(const std::string& json_path) {
    const json::Document doc = json::Document::parse_file(json_path);
    const json::Value    j   = doc.root();

    PlanningTask task;

    // Read requirements to detect KD45 (doxastic/belief) vs S5 (knowledge) frame.
    // plank exports requirements as a list of strings under "requirements".
    // KD45 is indicated by ":kd45" or ":belief"; S5 by ":s5" or ":knowledge".
    // If requirements are absent or neither flag is found, default to S5 (conservative).
    task.kd45 = false;
    if (j.contains("planning-task-info")) {
        const auto pti = j.at("planning-task-info");
        if (pti.contains("requirements")) {
            for (const auto req : pti.at("requirements")) {
                std::string r = req.get<std::string>();
                if (r == ":kd45" || r == ":KD45-frames" || r == ":belief" || r == ":doxastic") {
                    task.kd45 = true;
                    break;
                }
            }
        }
    }


    if (!task.kd45 && j.contains("requirements")) {
        for (const auto req : j.at("requirements")) {
            std::string r = req.get<std::string>();
            if (r == ":kd45" || r == ":KD45-frames" || r == ":belief" || r == ":doxastic") {
                task.kd45 = true;
                break;
            }
        }
    }

    std::cerr << "[parser] Frame: " << (task.kd45 ? "KD45 (belief)" : "S5 (knowledge)") << "\n";

    for (const auto a : j.at("language").at("atoms")) {
        std::string name = a.get<std::string>();
        task.atom_index[name] = static_cast<AtomIdx>(task.atom_names.size());
        task.atom_names.push_back(name);
    }

    for (const auto a : j.at("language").at("agents")) {
        std::string name = a.get<std::string>();
        task.agent_index[name] = static_cast<AgentIdx>(task.agent_names.size());
        task.agent_names.push_back(name);
    }

    size_t na = task.num_agents();

    const auto is = j.at("initial-state");

    // Views into the document; it outlives every lookup below.
    std::unordered_map<std::string_view, WorldIdx> world_idx;
    bool worlds_numbered = true;   // names are exactly w0, w1, … in order
    {
        WorldIdx idx = 0;
        const auto worlds = is.at("worlds");
        world_idx.reserve(worlds.size());
        for (const auto w : worlds) {
            const std::string_view name = w.str();
            worlds_numbered = worlds_numbered && name == "w" + std::to_string(idx);
            world_idx[name] = idx++;
        }
    }
    // kNoWorld if unknown.
    const auto world_of = [&](std::string_view name) -> WorldIdx {
        if (worlds_numbered && name.size() > 1 && name[0] == 'w') {
            WorldIdx k = 0;
            const auto r = std::from_chars(name.data() + 1, name.data() + name.size(), k);
            if (r.ec == std::errc{} && r.ptr == name.data() + name.size() &&
                k < world_idx.size() && (name.size() == 2 || name[1] != '0'))
                return k;
        }
        const auto it = world_idx.find(name);
        return it == world_idx.end() ? kNoWorld : it->second;
    };
    std::unordered_map<std::string_view, AtomIdx> atom_sv;
    atom_sv.reserve(task.atom_names.size());
    for (AtomIdx p = 0; p < task.atom_names.size(); ++p)
        atom_sv[task.atom_names[p]] = p;

    size_t nw = world_idx.size();

    // The model is allocated up front as three flat bit arrays; every field
    // below writes bits into them rather than growing per-world containers.
    task.init.allocate(static_cast<std::uint32_t>(nw),
                       static_cast<std::uint32_t>(task.num_atoms()),
                       static_cast<std::uint32_t>(na));

    for (auto& [wname, atoms] : is.at("labels").items()) {
        const WorldIdx w = world_of(wname);
        if (w == kNoWorld) continue;

        for (const auto a : atoms) {
            auto ait = atom_sv.find(a.str());
            if (ait != atom_sv.end())
                task.init.set_atom(w, ait->second);
        }
    }

    for (const auto d : is.at("designated")) {
        const WorldIdx w = world_of(d.str());
        if (w != kNoWorld)
            task.init.set_designated(w);
    }

    {
        SetInterner interner(task.init);
        std::vector<WorldIdx> succ;
        for (auto& [agent_name, rows] : is.at("relations").items()) {
            auto ait = task.agent_index.find(std::string(agent_name));
            if (ait == task.agent_index.end()) continue;

            AgentIdx ag = ait->second;

            // Compact form written by tools/ground: distinct successor sets and,
            // for each world in the order of "worlds", the index of its set.
            if (rows.is_object() && rows.contains("sets") && rows.contains("of")) {
                std::vector<std::uint32_t> ids;
                for (const auto set : rows.at("sets")) {
                    succ.clear();
                    for (const auto t : set) {
                        const WorldIdx dst = t.is_number() ? static_cast<WorldIdx>(t.number())
                                                           : world_of(t.str());
                        if (dst != kNoWorld && dst < nw) succ.push_back(dst);
                    }
                    std::sort(succ.begin(), succ.end());
                    succ.erase(std::unique(succ.begin(), succ.end()), succ.end());
                    ids.push_back(interner.intern(succ));
                }
                const auto of = rows.at("of");
                if (of.size() != nw)
                    throw std::runtime_error("relations of agent " + std::string(agent_name) +
                                             ": \"of\" has " + std::to_string(of.size()) +
                                             " entries for " + std::to_string(nw) + " worlds");
                for (std::size_t w = 0; w < nw; ++w) {
                    const auto k = static_cast<std::size_t>(of[w].number());
                    if (k >= ids.size())
                        throw std::runtime_error("relations of agent " + std::string(agent_name) +
                                                 ": set index out of range");
                    task.init.set_of[std::size_t(ag) * nw + w] = ids[k];
                }
                continue;
            }

            for (auto& [src_wname, targets] : rows.items()) {
                const WorldIdx src = world_of(src_wname);
                if (src == kNoWorld) continue;

                succ.clear();
                for (const auto t : targets) {
                    const WorldIdx dst = world_of(t.str());
                    if (dst != kNoWorld) succ.push_back(dst);
                }
                std::sort(succ.begin(), succ.end());
                succ.erase(std::unique(succ.begin(), succ.end()), succ.end());

                // A world listed twice for one agent keeps the union of its rows.
                const auto cur = task.init.succ(ag, src);
                if (!cur.empty()) {
                    std::vector<WorldIdx> merged;
                    std::set_union(cur.begin(), cur.end(), succ.begin(), succ.end(),
                                   std::back_inserter(merged));
                    succ.swap(merged);
                }
                task.init.set_of[std::size_t(ag) * task.init.num_worlds + src] = interner.intern(succ);
            }
        }
        task.init.invalidate();
    }

    for (auto& [action_name, a_j] : j.at("actions").items()) {
        Action act;
        act.name       = std::string(action_name);
        act.num_agents = na;

        std::unordered_map<std::string, EventIdx> event_idx;

        for (const auto e : a_j.at("events")) {
            std::string ename = e.get<std::string>();
            EventIdx eid = static_cast<EventIdx>(act.events.size());

            event_idx[ename] = eid;

            Event ev;
            ev.id   = eid;
            ev.name = ename;
            ev.is_nil = (ename == "nil");
            ev.precondition = Formula::make_top();

            act.events.push_back(std::move(ev));
        }

        size_t ne = act.events.size();

        if (a_j.contains("preconditions")) {
            for (auto& [ename, pre_j] : a_j.at("preconditions").items()) {
                auto it = event_idx.find(std::string(ename));
                if (it == event_idx.end()) continue;

                act.events[it->second].precondition =
                    unwrap_formula(pre_j, task.atom_index, task.agent_index);
            }
        }

        if (a_j.contains("effects")) {
            for (auto& [ename, eff_j] : a_j.at("effects").items()) {
                auto it = event_idx.find(std::string(ename));
                if (it == event_idx.end() || eff_j.is_null()) continue;

                Event& ev = act.events[it->second];

                for (auto& [atom_name, val_j] : eff_j.items()) {
                    auto ait = task.atom_index.find(std::string(atom_name));
                    if (ait == task.atom_index.end()) continue;

                    AtomIdx atom = ait->second;

                    FormulaPtr cond =
                        unwrap_formula(val_j, task.atom_index, task.agent_index);

                    if (cond->kind == FormulaKind::Top) {
                        ev.post_true[atom] = Formula::make_top();
                    } else if (cond->kind == FormulaKind::Bot) {
                        ev.post_false[atom] = Formula::make_top();
                    } else {
                        ev.post_true[atom]  = cond;
                        ev.post_false[atom] = Formula::make_not(cond);
                    }
                }
            }
        }

        for (const auto d : a_j.at("designated")) {
            std::string ename = d.get<std::string>();
            auto it = event_idx.find(std::string(ename));
            if (it != event_idx.end())
                act.designated_events.insert(it->second);
        }

        // Build obs_type_rel: obs_type_name -> event relation
        std::unordered_map<std::string,
            std::vector<std::unordered_set<EventIdx>>> obs_type_rel;

        std::string first_obs_type;
        if (a_j.contains("relations")) {
            for (auto& [obs_type, rel_j] : a_j.at("relations").members()) {
                if (first_obs_type.empty()) first_obs_type = std::string(obs_type);
                std::vector<std::unordered_set<EventIdx>> rel(ne);

                for (auto& [src_ename, targets] : rel_j.items()) {
                    auto sit = event_idx.find(std::string(src_ename));
                    if (sit == event_idx.end()) continue;

                    for (const auto t : targets) {
                        std::string tname = t.get<std::string>();
                        auto tit = event_idx.find(tname);
                        if (tit != event_idx.end())
                            rel[sit->second].insert(tit->second);
                    }
                }

                obs_type_rel[std::string(obs_type)] = std::move(rel);
            }
        }

        // Collect all observability cases per agent
        act.obs_cases.resize(na);

        if (a_j.contains("observability-conditions")) {
            for (auto& [agent_name, obs_j] : a_j.at("observability-conditions").items()) {
                auto ait = task.agent_index.find(std::string(agent_name));
                if (ait == task.agent_index.end()) continue;

                AgentIdx ag = ait->second;

                // Document order: plank takes the first condition that holds.
                for (auto& [obs_type, cond_j] : obs_j.members()) {
                    auto rit = obs_type_rel.find(std::string(obs_type));
                    if (rit == obs_type_rel.end()) continue;

                    ObsCase oc;
                    oc.condition = unwrap_formula(cond_j, task.atom_index, task.agent_index);
                    oc.relation  = rit->second;
                    oc.finalize(ne);
                    act.obs_cases[ag].push_back(std::move(oc));
                }
            }
        }

        if (auto rit = obs_type_rel.find(first_obs_type); rit != obs_type_rel.end()) {
            act.default_obs.condition = Formula::make_top();
            act.default_obs.relation  = rit->second;
        } else {
            act.default_obs.condition = Formula::make_top();
            act.default_obs.relation.assign(ne, {});
            for (EventIdx e = 0; e < ne; ++e)
                for (EventIdx f = 0; f < ne; ++f) act.default_obs.relation[e].insert(f);
        }
        act.default_obs.finalize(ne);

        task.action_index[act.name] =
            static_cast<ActionIdx>(task.actions.size());

        task.actions.push_back(std::move(act));
    }

    task.goal =
        unwrap_formula(j.at("goal"), task.atom_index, task.agent_index);

    // Detect partial observability.
    //
    // A domain has partial observability iff at least one action has agents
    // with heterogeneous observability — some Fully observable, others
    // Oblivious or conditional. In the parsed representation this shows up as
    // obs_cases[ag][0].relation differing between agents: Fully has the
    // identity relation (e -> {e} for all events), Oblivious maps every event
    // to {nil} (the single non-designated event), and conditional cases have
    // world-dependent rows.
    //
    // Comparing obs_cases sizes across agents is insufficient — Gossip assigns
    // exactly one ObsCase per agent (all have size 1) but with structurally
    // different relations. We instead compare the actual relation vectors of
    // the first ObsCase across agents: if any two agents disagree the domain
    // has partial observability.
    task.partial_obs = false;
    for (auto& act : task.actions) {
        if (act.obs_cases.size() < 2) continue;

        size_t ref_ag = act.obs_cases.size();
        for (size_t ag = 0; ag < act.obs_cases.size(); ag++) {
            if (!act.obs_cases[ag].empty()) { ref_ag = ag; break; }
        }
        if (ref_ag == act.obs_cases.size()) continue;

        const auto& ref_rel = act.obs_cases[ref_ag][0].relation;
        for (size_t ag = ref_ag + 1; ag < act.obs_cases.size(); ag++) {
            if (act.obs_cases[ag].empty()) continue;
            const auto& ag_rel = act.obs_cases[ag][0].relation;
            if (ag_rel.size() != ref_rel.size()) {
                task.partial_obs = true;
                break;
            }
            bool differs = false;
            for (size_t ei = 0; ei < ref_rel.size() && !differs; ei++)
                if (ag_rel[ei] != ref_rel[ei])
                    differs = true;
            if (differs) {
                task.partial_obs = true;
                break;
            }
        }
        if (task.partial_obs) break;
    }

    // Detect Kw-only goal.
    //
    // A goal is Kw-only if it is a single Kw formula or a conjunction where
    // every top-level conjunct is a Kw formula (FormulaKind::Kw, or an Or of
    // two Belief formulas that the parser expands Kw into). We check the
    // top-level structure only — deeper nesting is handled by the heuristic.
    // has_atom_conjunct (defined in heuristic.hpp) returns true iff the formula
    // has any bare atom at the top level, so goal_kw_only = !has_atom_conjunct.
    task.goal_kw_only = task.goal && !has_atom_conjunct(*task.goal);

    std::cerr << "[parser] Loaded: "
              << task.num_atoms()   << " atoms, "
              << task.num_agents()  << " agents, "
              << task.init.num_worlds << " worlds ("
              << task.init.num_designated() << " designated), "
              << task.num_actions() << " actions"
              << "  partial_obs=" << task.partial_obs
              << "  goal_kw_only=" << task.goal_kw_only
              << "\n";

    return task;
}