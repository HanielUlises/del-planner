#pragma once
// A small reduced ordered BDD package: unique table, lossy computed cache,
// static variable order (variable index = level). No complement edges and no
// garbage collection: a Manager lives for one task.
//
// Operations: ITE, AND, OR, XOR, NOT, cofactors, EXISTS, AND-EXISTS (relational
// product), variable permutation (structural when order-preserving), model
// counting, enumeration (for tests) and node counting.
#include <algorithm>
#include <cassert>
#include <functional>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace bdd {

using Ref = std::uint32_t;
inline constexpr Ref kFalse = 0, kTrue = 1;
inline constexpr std::uint32_t kTerm = 0xFFFFFFFFu;

struct Node { std::uint32_t var; Ref lo, hi; };

class Manager {
public:
    explicit Manager(std::uint32_t nvars, unsigned cache_log2 = 22, unsigned table_log2 = 20)
        : nvars_(nvars) {
        nodes_.reserve(std::size_t(1) << table_log2);
        nodes_.push_back({kTerm, kFalse, kFalse});
        nodes_.push_back({kTerm, kTrue, kTrue});
        table_.assign(std::size_t(1) << table_log2, kNone);
        cache_.assign(std::size_t(1) << cache_log2, Entry{0, 0, 0, 0, 0});
        cmask_ = cache_.size() - 1;
    }

    [[nodiscard]] std::uint32_t num_vars() const noexcept { return nvars_; }
    [[nodiscard]] std::size_t size() const noexcept { return nodes_.size(); }
    [[nodiscard]] std::uint32_t top(Ref f) const noexcept { return nodes_[f].var; }
    [[nodiscard]] Ref lo(Ref f) const noexcept { return nodes_[f].lo; }
    [[nodiscard]] Ref hi(Ref f) const noexcept { return nodes_[f].hi; }

    Ref var(std::uint32_t v)  { return mk(v, kFalse, kTrue); }
    Ref nvar(std::uint32_t v) { return mk(v, kTrue, kFalse); }

    Ref mk(std::uint32_t v, Ref l, Ref h) {
        if (l == h) return l;
        if (v >= top(l) || v >= top(h)) throw std::logic_error("bdd: order violated in mk");
        std::size_t i = hash3(v, l, h) & (table_.size() - 1);
        while (true) {
            const Ref r = table_[i];
            if (r == kNone) break;
            const Node& n = nodes_[r];
            if (n.var == v && n.lo == l && n.hi == h) return r;
            i = (i + 1) & (table_.size() - 1);
        }
        const Ref r = static_cast<Ref>(nodes_.size());
        if (r == kNone) throw std::runtime_error("bdd: node table full");
        nodes_.push_back({v, l, h});
        table_[i] = r;
        if (nodes_.size() * 2 > table_.size()) grow();
        return r;
    }

    // ------------------------------------------------------------ boolean ops
    Ref neg(Ref f) {
        if (f <= kTrue) return f ^ 1u;
        Ref r;
        if (lookup(kNeg, f, 0, 0, r)) return r;
        const Node n = nodes_[f];
        r = mk(n.var, neg(n.lo), neg(n.hi));
        store(kNeg, f, 0, 0, r);
        return r;
    }

    Ref band(Ref f, Ref g) {
        if (f == kFalse || g == kFalse) return kFalse;
        if (f == kTrue) return g;
        if (g == kTrue || f == g) return f;
        if (f > g) std::swap(f, g);
        Ref r;
        if (lookup(kAnd, f, g, 0, r)) return r;
        const std::uint32_t v = std::min(top(f), top(g));
        const auto [f0, f1] = cof(f, v);
        const auto [g0, g1] = cof(g, v);
        const Ref r0 = band(f0, g0);
        r = mk(v, r0, band(f1, g1));
        store(kAnd, f, g, 0, r);
        return r;
    }

    Ref bor(Ref f, Ref g) {
        if (f == kTrue || g == kTrue) return kTrue;
        if (f == kFalse) return g;
        if (g == kFalse || f == g) return f;
        if (f > g) std::swap(f, g);
        Ref r;
        if (lookup(kOr, f, g, 0, r)) return r;
        const std::uint32_t v = std::min(top(f), top(g));
        const auto [f0, f1] = cof(f, v);
        const auto [g0, g1] = cof(g, v);
        const Ref r0 = bor(f0, g0);
        r = mk(v, r0, bor(f1, g1));
        store(kOr, f, g, 0, r);
        return r;
    }

    Ref bxor(Ref f, Ref g) {
        if (f == g) return kFalse;
        if (f == kFalse) return g;
        if (g == kFalse) return f;
        if (f == kTrue) return neg(g);
        if (g == kTrue) return neg(f);
        if (f > g) std::swap(f, g);
        Ref r;
        if (lookup(kXor, f, g, 0, r)) return r;
        const std::uint32_t v = std::min(top(f), top(g));
        const auto [f0, f1] = cof(f, v);
        const auto [g0, g1] = cof(g, v);
        const Ref r0 = bxor(f0, g0);
        r = mk(v, r0, bxor(f1, g1));
        store(kXor, f, g, 0, r);
        return r;
    }

    Ref biff(Ref f, Ref g) { return neg(bxor(f, g)); }
    Ref bdiff(Ref f, Ref g) { return band(f, neg(g)); }       // f ∧ ¬g
    [[nodiscard]] bool subset(Ref f, Ref g) { return bdiff(f, g) == kFalse; }

    Ref ite(Ref f, Ref g, Ref h) {
        if (f == kTrue) return g;
        if (f == kFalse) return h;
        if (g == h) return g;
        if (g == kTrue && h == kFalse) return f;
        if (g == kFalse && h == kTrue) return neg(f);
        Ref r;
        if (lookup(kIte, f, g, h, r)) return r;
        const std::uint32_t v = std::min(top(f), std::min(top(g), top(h)));
        const auto [f0, f1] = cof(f, v);
        const auto [g0, g1] = cof(g, v);
        const auto [h0, h1] = cof(h, v);
        const Ref r0 = ite(f0, g0, h0);
        r = mk(v, r0, ite(f1, g1, h1));
        store(kIte, f, g, h, r);
        return r;
    }

    // f with variable v fixed to val.
    Ref cofactor(Ref f, std::uint32_t v, bool val) {
        if (top(f) > v) return f;
        if (top(f) == v) return val ? hi(f) : lo(f);
        Ref r;
        const std::uint32_t op = val ? kCof1 : kCof0;
        if (lookup(op, f, v, 0, r)) return r;
        const Node n = nodes_[f];
        const Ref r0 = cofactor(n.lo, v, val);
        r = mk(n.var, r0, cofactor(n.hi, v, val));
        store(op, f, v, 0, r);
        return r;
    }

    // f restricted by a conjunction of literals (a cube).
    Ref restrict_cube(Ref f, Ref cube) {
        while (cube > kTrue && f > kTrue) {
            const std::uint32_t c = top(cube);
            if (c < top(f)) { cube = lo(cube) == kFalse ? hi(cube) : lo(cube); continue; }
            break;
        }
        if (cube <= kTrue || f <= kTrue) return f;
        Ref r;
        if (lookup(kRestrict, f, cube, 0, r)) return r;
        const std::uint32_t v = top(f);
        const Node n = nodes_[f];
        if (top(cube) == v) {
            const bool pos = lo(cube) == kFalse;
            const Ref next = pos ? hi(cube) : lo(cube);
            r = restrict_cube(pos ? n.hi : n.lo, next);
        } else {
            const Ref r0 = restrict_cube(n.lo, cube);
            r = mk(v, r0, restrict_cube(n.hi, cube));
        }
        store(kRestrict, f, cube, 0, r);
        return r;
    }

    // Positive cube of the given variables (any order, duplicates allowed).
    Ref varset(std::vector<std::uint32_t> vs) {
        std::sort(vs.begin(), vs.end(), std::greater<>());
        Ref c = kTrue;
        std::uint32_t last = kTerm;
        for (std::uint32_t v : vs) if (v != last) { c = mk(v, kFalse, c); last = v; }
        return c;
    }
    // Conjunction of literals (var, value).
    Ref cube(std::vector<std::pair<std::uint32_t, bool>> lits) {
        std::sort(lits.begin(), lits.end(), [](auto& a, auto& b) { return a.first > b.first; });
        Ref c = kTrue;
        for (auto [v, b] : lits) c = b ? mk(v, kFalse, c) : mk(v, c, kFalse);
        return c;
    }

    Ref exists(Ref f, Ref vars) {
        if (f <= kTrue) return f;
        while (vars > kTrue && top(vars) < top(f)) vars = hi(vars);
        if (vars == kTrue) return f;
        Ref r;
        if (lookup(kExists, f, vars, 0, r)) return r;
        const Node n = nodes_[f];
        if (top(vars) == n.var) {
            const Ref r0 = exists(n.lo, hi(vars));
            r = r0 == kTrue ? kTrue : bor(r0, exists(n.hi, hi(vars)));
        } else {
            const Ref r0 = exists(n.lo, vars);
            r = mk(n.var, r0, exists(n.hi, vars));
        }
        store(kExists, f, vars, 0, r);
        return r;
    }

    // ∃vars. f ∧ g
    Ref and_exists(Ref f, Ref g, Ref vars) {
        if (f == kFalse || g == kFalse) return kFalse;
        if (f == kTrue && g == kTrue) return kTrue;
        if (f == kTrue || f == g) return exists(g, vars);
        if (g == kTrue) return exists(f, vars);
        if (f > g) std::swap(f, g);
        const std::uint32_t v = std::min(top(f), top(g));
        while (vars > kTrue && top(vars) < v) vars = hi(vars);
        if (vars == kTrue) return band(f, g);
        Ref r;
        if (lookup(kAndEx, f, g, vars, r)) return r;
        const auto [f0, f1] = cof(f, v);
        const auto [g0, g1] = cof(g, v);
        if (top(vars) == v) {
            const Ref r0 = and_exists(f0, g0, hi(vars));
            r = r0 == kTrue ? kTrue : bor(r0, and_exists(f1, g1, hi(vars)));
        } else {
            const Ref r0 = and_exists(f0, g0, vars);
            r = mk(v, r0, and_exists(f1, g1, vars));
        }
        store(kAndEx, f, g, vars, r);
        return r;
    }

    // Rename variables by perm (perm[v] = new variable of v; size num_vars()).
    // monotone = the caller guarantees perm is increasing on the support of f,
    // which permits a structural relabelling; otherwise ITE rebuilds.
    Ref permute(Ref f, const std::vector<std::uint32_t>& perm, bool monotone) {
        const std::uint32_t op = kPermBase + (++perm_epoch_ << 1) + (monotone ? 1u : 0u);
        perm_ = &perm;
        return permute_rec(f, op, monotone);
    }

    // Number of assignments to `vars` (any order, must contain supp(f)).
    // Gaps are scaled per level, so a small count is exact however many
    // variables the set has.
    double count(Ref f, std::vector<std::uint32_t> vars) {
        std::sort(vars.begin(), vars.end());
        // below[v] = number of set variables at levels >= v
        std::vector<std::uint32_t> below(nvars_ + 1, 0);
        std::size_t j = vars.size();
        for (std::uint32_t v = nvars_ + 1; v-- > 0;) {
            while (j > 0 && vars[j - 1] >= v) --j;
            below[v] = static_cast<std::uint32_t>(vars.size() - j);
        }
        auto lvl = [&](Ref r) { return top(r) == kTerm ? below[nvars_] : below[top(r)]; };
        std::unordered_map<Ref, double> memo;
        std::function<double(Ref)> rec = [&](Ref r) -> double {
            if (r == kFalse) return 0.0;
            if (r == kTrue) return 1.0;
            if (auto it = memo.find(r); it != memo.end()) return it->second;
            const std::uint32_t here = below[top(r)] - 1;       // set vars strictly below r's var
            const double c = std::ldexp(rec(lo(r)), int(here - lvl(lo(r)))) +
                             std::ldexp(rec(hi(r)), int(here - lvl(hi(r))));
            memo.emplace(r, c);
            return c;
        };
        return std::ldexp(rec(f), int(vars.size() - lvl(f)));
    }

    // Satisfying assignments over `vars` (sorted by level, ⊇ support).
    void enumerate(Ref f, const std::vector<std::uint32_t>& vars,
                   std::vector<std::vector<std::uint8_t>>& out) {
        std::vector<std::uint8_t> cur(vars.size());
        enum_rec(f, vars, 0, cur, out);
    }

    std::size_t node_count(const std::vector<Ref>& roots) const {
        std::unordered_set<Ref> seen;
        std::vector<Ref> st(roots.begin(), roots.end());
        while (!st.empty()) {
            const Ref r = st.back(); st.pop_back();
            if (!seen.insert(r).second || r <= kTrue) continue;
            st.push_back(nodes_[r].lo); st.push_back(nodes_[r].hi);
        }
        return seen.size();
    }

private:
    static constexpr Ref kNone = 0xFFFFFFFFu;
    enum : std::uint32_t { kNeg = 1, kAnd, kOr, kXor, kIte, kCof0, kCof1, kRestrict,
                           kExists, kAndEx, kPermBase = 64 };
    struct Entry { std::uint32_t op, a, b, c; Ref r; };

    static std::uint64_t hash3(std::uint64_t a, std::uint64_t b, std::uint64_t c) noexcept {
        std::uint64_t h = a * 0x9E3779B97F4A7C15ULL ^ (b + 0x632BE59BD9B4E019ULL) * 0xC2B2AE3D27D4EB4FULL;
        h ^= (c + 0x165667B19E3779F9ULL) * 0x27D4EB2F165667C5ULL;
        return h ^ (h >> 29);
    }
    bool lookup(std::uint32_t op, std::uint32_t a, std::uint32_t b, std::uint32_t c, Ref& r) const noexcept {
        const Entry& e = cache_[hash3(op ^ (std::uint64_t(a) << 32), b, c) & cmask_];
        if (e.op == op && e.a == a && e.b == b && e.c == c) { r = e.r; return true; }
        return false;
    }
    void store(std::uint32_t op, std::uint32_t a, std::uint32_t b, std::uint32_t c, Ref r) noexcept {
        cache_[hash3(op ^ (std::uint64_t(a) << 32), b, c) & cmask_] = Entry{op, a, b, c, r};
    }
    std::pair<Ref, Ref> cof(Ref f, std::uint32_t v) const noexcept {
        const Node& n = nodes_[f];
        return n.var == v ? std::pair<Ref, Ref>{n.lo, n.hi} : std::pair<Ref, Ref>{f, f};
    }
    void grow() {
        std::vector<Ref> t(table_.size() * 2, kNone);
        const std::size_t mask = t.size() - 1;
        for (Ref r = 2; r < nodes_.size(); ++r) {
            const Node& n = nodes_[r];
            std::size_t i = hash3(n.var, n.lo, n.hi) & mask;
            while (t[i] != kNone) i = (i + 1) & mask;
            t[i] = r;
        }
        table_.swap(t);
    }
    Ref permute_rec(Ref f, std::uint32_t op, bool monotone) {
        if (f <= kTrue) return f;
        Ref r;
        if (lookup(op, f, 0, 0, r)) return r;
        const Node n = nodes_[f];
        const Ref l = permute_rec(n.lo, op, monotone);
        const Ref h = permute_rec(n.hi, op, monotone);
        const std::uint32_t v = (*perm_)[n.var];
        r = monotone ? mk(v, l, h) : ite(var(v), h, l);
        store(op, f, 0, 0, r);
        return r;
    }
    double frac(Ref f, std::unordered_map<Ref, double>& memo) {
        if (f == kFalse) return 0.0;
        if (f == kTrue) return 1.0;
        auto it = memo.find(f);
        if (it != memo.end()) return it->second;
        const double r = 0.5 * (frac(lo(f), memo) + frac(hi(f), memo));
        memo.emplace(f, r);
        return r;
    }
    void enum_rec(Ref f, const std::vector<std::uint32_t>& vars, std::size_t i,
                  std::vector<std::uint8_t>& cur, std::vector<std::vector<std::uint8_t>>& out) {
        if (f == kFalse) return;
        if (i == vars.size()) {
            if (f != kTrue) throw std::logic_error("bdd: enumerate: support outside vars");
            out.push_back(cur); return;
        }
        const std::uint32_t v = vars[i];
        if (top(f) < v) throw std::logic_error("bdd: enumerate: support outside vars");
        const Ref l = top(f) == v ? lo(f) : f, h = top(f) == v ? hi(f) : f;
        cur[i] = 0; enum_rec(l, vars, i + 1, cur, out);
        cur[i] = 1; enum_rec(h, vars, i + 1, cur, out);
    }

    std::uint32_t nvars_;
    std::vector<Node> nodes_;
    std::vector<Ref> table_;
    std::vector<Entry> cache_;
    std::size_t cmask_{0};
    std::uint32_t perm_epoch_{0};
    const std::vector<std::uint32_t>* perm_{nullptr};
};

} // namespace bdd
