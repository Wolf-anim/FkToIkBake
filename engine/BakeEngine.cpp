#include "BakeEngine.h"

#include <chrono>
#include <cmath>

namespace fk2ik {

std::vector<double> buildSampleTimes(double start, double end, double step) {
    std::vector<double> t;
    if (step <= 0.0 || end < start) {
        t.push_back(start);
        return t;
    }
    const long long n = static_cast<long long>(std::floor((end - start) / step + 1e-9));
    t.reserve(static_cast<size_t>(n) + 2);
    for (long long i = 0; i <= n; ++i) t.push_back(start + step * static_cast<double>(i));
    if (end - t.back() > 1e-9) t.push_back(end);
    return t;
}

void prepare(LimbJob& job) {
    const Mat4 m0 = composeLocal(job.poleDesc, {0, 0, 0}, job.poleRotate);
    job.polePivotAtZero = m0.transformPoint(job.poleDesc.rotatePivot);
}

Vec3 poleTranslate(const LimbJob& job, const Mat4& poleParentInverse, const Vec3& worldTarget) {
    return poleParentInverse.transformPoint(worldTarget) - job.polePivotAtZero;
}

bool BakeEngine::run(std::vector<LimbJob>& jobs, SampleSource& source,
                     const std::vector<double>& times, std::vector<LimbBakeResult>& results,
                     BakeStats& stats, std::string& error) {
    using clock = std::chrono::steady_clock;
    const size_t nl = jobs.size(), nt = times.size();
    results.assign(nl, LimbBakeResult{});
    for (auto& r : results) {
        for (auto& c : r.ctrl) c.resize(nt);
        for (auto& c : r.pole) c.resize(nt);
        r.poleTarget.resize(nt);
    }
    std::vector<LimbSample> samples(nl);
    std::vector<Euler> prevRot(nl);
    std::vector<bool> hasPrev(nl);
    for (size_t l = 0; l < nl; ++l) {
        prevRot[l] = jobs[l].ctrlStartRotate;
        hasPrev[l] = jobs[l].hasStartRotate;
        if (!jobs[l].calibration.valid) {
            error = "limb '" + jobs[l].name + "' has an invalid calibration (zero-length bones?)";
            return false;
        }
    }

    double tSample = 0, tSolve = 0;
    for (size_t i = 0; i < nt; ++i) {
        const auto a = clock::now();
        if (!source.sample(times[i], samples)) {
            error = "sampling aborted at time " + std::to_string(times[i]);
            return false;
        }
        const auto b = clock::now();
        for (size_t l = 0; l < nl; ++l) {
            const LimbJob& job = jobs[l];
            const LimbSample& s = samples[l];
            LimbBakeResult& res = results[l];
            const MatchResult m = match(s.fk, job.calibration, job.settings);

            if (m.reachRatio > res.maxReachRatio) res.maxReachRatio = m.reachRatio;
            if (m.reachRatio > 1.0005) ++res.overExtendedSamples;
            if (m.fallbackWeight > 0.0 && job.settings.mode == MatchMode::Position)
                ++res.straightFallbackSamples;

            const ControlChannels cc =
                solveChannels(job.ctrlDesc, m.ikCtrlWorld, s.ctrlParentInverse, prevRot[l], hasPrev[l]);
            prevRot[l] = cc.rotate;
            hasPrev[l] = true;
            res.ctrl[0][i] = cc.translate.x;
            res.ctrl[1][i] = cc.translate.y;
            res.ctrl[2][i] = cc.translate.z;
            res.ctrl[3][i] = cc.rotate.x;
            res.ctrl[4][i] = cc.rotate.y;
            res.ctrl[5][i] = cc.rotate.z;

            if (job.hasPole) {
                // poleVectorConstraint follows the pole's world rotate pivot.
                // Pole rotation is not keyed, so with the pole's current
                // rotation the translate is a pure offset:
                //   worldPivot = (rp * M0 + T) * parentWorld
                const Vec3 t = poleTranslate(job, s.poleParentInverse, m.poleWorld);
                res.poleTarget[i] = m.poleWorld;
                res.pole[0][i] = t.x;
                res.pole[1][i] = t.y;
                res.pole[2][i] = t.z;
            }
        }
        const auto c = clock::now();
        tSample += std::chrono::duration<double>(b - a).count();
        tSolve += std::chrono::duration<double>(c - b).count();
    }
    stats.sampleSeconds = tSample;
    stats.solveSeconds = tSolve;
    stats.samples = nt;
    return true;
}

}  // namespace fk2ik
