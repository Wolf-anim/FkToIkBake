#include "KeyReducer.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace fk2ik {

bool isConstant(const std::vector<double>& v, double eps) {
    for (size_t i = 1; i < v.size(); ++i)
        if (std::fabs(v[i] - v[0]) > eps) return false;
    return true;
}

std::vector<size_t> reduceKeys(const std::vector<double>& t, const std::vector<double>& v,
                               double tol) {
    const size_t n = v.size();
    std::vector<size_t> keep;
    if (n == 0) return keep;
    if (n <= 2) {
        for (size_t i = 0; i < n; ++i) keep.push_back(i);
        return keep;
    }
    if (isConstant(v)) return {0, n - 1};
    if (tol <= 0.0) {
        keep.resize(n);
        for (size_t i = 0; i < n; ++i) keep[i] = i;
        return keep;
    }

    // Iterative RDP (no recursion: safe for 100k+ frame sequences).
    std::vector<char> mark(n, 0);
    mark[0] = mark[n - 1] = 1;
    std::vector<std::pair<size_t, size_t>> stack;
    stack.emplace_back(0, n - 1);
    while (!stack.empty()) {
        auto [a, b] = stack.back();
        stack.pop_back();
        if (b <= a + 1) continue;
        const double ta = t[a], tb = t[b], va = v[a], vb = v[b];
        const double span = tb - ta;
        double worst = -1.0;
        size_t idx = a;
        for (size_t i = a + 1; i < b; ++i) {
            const double u = span > 0 ? (t[i] - ta) / span : 0.0;
            const double err = std::fabs(v[i] - (va + (vb - va) * u));
            if (err > worst) { worst = err; idx = i; }
        }
        if (worst > tol) {
            mark[idx] = 1;
            stack.emplace_back(a, idx);
            stack.emplace_back(idx, b);
        }
    }
    for (size_t i = 0; i < n; ++i)
        if (mark[i]) keep.push_back(i);
    return keep;
}

}  // namespace fk2ik
