// Functional tests for the motion-family trackers: Sort, ByteTrack, OC-Sort.
//
// Each scenario feeds synthetic detections with known ground truth and checks
// per-algorithm invariants:
//   1. single-object linear motion -> exactly 1 track, same ID across frames
//   2. two crossing objects        -> both survive the crossing with 2 IDs
//   3. dropout (occlusion)         -> ID preserved after the blackout window
//   4. identity churn              -> track IDs never repeat after removal
//
// Exit code 0 = all scenarios passed for all three algorithms.
#include "Motrack.h"

#include <cmath>
#include <cstdio>
#include <set>
#include <vector>

using namespace motrack;

static int g_failures = 0;

#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("  FAIL: %s (at %s:%d)\n", msg, __FILE__, __LINE__);\
            ++g_failures;                                                   \
        }                                                                   \
    } while (0)

namespace {

struct ScenarioResult
{
    size_t max_concurrent = 0;
    std::set<unsigned long long> ids_seen;
};

ScenarioResult runScenario(TrackerType type, const TrackerConfig& cfg,
                           const std::vector<std::vector<Object>>& frames)
{
    Tracker tracker(type, cfg);
    ScenarioResult res;
    for (const auto& objects : frames)
    {
        const auto tracks = tracker.update(objects);
        if (tracks.size() > res.max_concurrent)
        {
            res.max_concurrent = tracks.size();
        }
        for (const auto& t : tracks)
        {
            res.ids_seen.insert(t.track_id);
        }
    }
    return res;
}

// Build a linear-motion object: starts at (x0,y0), moves (dx,dy) per frame.
Object linearObject(float x0, float y0, float dx, float dy, float w, float h,
                    float prob, unsigned int label, int frame)
{
    return Object(prob, label,
                  Rect(x0 + dx * frame, y0 + dy * frame, w, h));
}

std::vector<std::vector<Object>> singleObject(int frames)
{
    std::vector<std::vector<Object>> frames_in;
    for (int f = 0; f < frames; ++f)
    {
        frames_in.push_back({linearObject(100, 100, 10, 5, 50, 50, 0.9f, 0, f)});
    }
    return frames_in;
}

// Two objects moving toward each other and swapping positions (crossing).
std::vector<std::vector<Object>> crossingObjects(int frames)
{
    std::vector<std::vector<Object>> frames_in;
    for (int f = 0; f < frames; ++f)
    {
        frames_in.push_back({
            linearObject(100, 100, 12, 0, 40, 40, 0.9f, 0, f),   // moves right
            linearObject(500, 100, -12, 0, 40, 40, 0.9f, 1, f),  // moves left
        });
    }
    return frames_in;
}

// One object visible for `visible` frames, missing for `dropout` frames
// (simulating occlusion / detector blackout), then reappearing on the same
// trajectory. Ground truth: the tracker SHOULD keep the same ID through the
// blackout (ByteTrack/OC-Sort do via the lost pool; Sort's max_age covers it
// if dropout < max_age).
std::vector<std::vector<Object>> dropoutObject(int visible, int dropout, int tail)
{
    std::vector<std::vector<Object>> frames_in;
    int f = 0;
    for (int i = 0; i < visible; ++i, ++f)
    {
        frames_in.push_back({linearObject(100, 100, 10, 0, 50, 50, 0.9f, 0, f)});
    }
    for (int i = 0; i < dropout; ++i, ++f)
    {
        frames_in.push_back({});
    }
    for (int i = 0; i < tail; ++i, ++f)
    {
        // continues the same trajectory (prediction should be close)
        frames_in.push_back({linearObject(100, 100, 10, 0, 50, 50, 0.9f, 0, f)});
    }
    return frames_in;
}

// Tracks that vanish permanently; a tracker must never reuse their IDs.
std::vector<std::vector<Object>> churnObjects(int cycles)
{
    std::vector<std::vector<Object>> frames_in;
    int f = 0;
    for (int c = 0; c < cycles; ++c)
    {
        // object appears for 3 frames, then disappears for max_age+5 frames
        for (int i = 0; i < 3; ++i, ++f)
        {
            frames_in.push_back({linearObject(100 + c * 300, 100, 5, 0, 50, 50, 0.9f, 0, f)});
        }
        for (int i = 0; i < 35; ++i, ++f)
        {
            frames_in.push_back({});
        }
    }
    return frames_in;
}

void report(const char* name, TrackerType type, const ScenarioResult& r)
{
    const char* algo = (type == TrackerType::Sort ? "sort"
                      : type == TrackerType::ByteTrack ? "bytetrack"
                      : "ocsort");
    std::printf("  [%-9s] %-18s max_concurrent=%zu ids=%zu\n",
                algo, name, r.max_concurrent, r.ids_seen.size());
}

TrackerConfig defaultCfg()
{
    TrackerConfig cfg;
    cfg.max_age = 30;
    cfg.track_thresh = 0.5f;
    cfg.high_thresh = 0.6f;
    cfg.match_thresh = 0.8f;
    return cfg;
}

void testSingleObject(TrackerType type)
{
    // 40 frames, one object: expect exactly 1 track ID, seen every frame.
    auto res = runScenario(type, defaultCfg(), singleObject(40));
    report("single", type, res);
    CHECK(res.ids_seen.size() == 1, "single object must yield exactly 1 track id");
    CHECK(res.max_concurrent == 1, "single object must yield 1 concurrent track");
}

void testCrossing(TrackerType type)
{
    // 40 frames, two objects swapping sides: expect 2 ids, never 3.
    auto res = runScenario(type, defaultCfg(), crossingObjects(40));
    report("crossing", type, res);
    CHECK(res.ids_seen.size() == 2, "crossing objects must keep 2 track ids");
    CHECK(res.max_concurrent <= 2, "crossing must not spawn extra tracks");
}

void testDropout(TrackerType type)
{
    // 15 visible + 10 blackout + 15 tail, max_age=30: same ID must survive.
    auto res = runScenario(type, defaultCfg(), dropoutObject(15, 10, 15));
    report("dropout", type, res);
    CHECK(res.ids_seen.size() == 1,
          "dropout < max_age: object must keep the same track id");
}

void testChurn(TrackerType type)
{
    // Repeated appear/disappear beyond max_age: ids must be unique per cycle
    // (no id reuse) — 4 cycles -> at least 4 distinct ids, none shared.
    auto res = runScenario(type, defaultCfg(), churnObjects(4));
    report("churn", type, res);
    CHECK(res.ids_seen.size() >= 4,
          "churn: each reappearance cycle must produce a fresh track id");
}

} // namespace

int main(int argc, char* argv[])
{
    (void)argc; (void)argv;

    const TrackerType algos[] = {TrackerType::Sort, TrackerType::ByteTrack, TrackerType::OCSort};

    std::printf("=== motrack algorithm functional tests ===\n");
    for (TrackerType type : algos)
    {
        std::printf("[%s]\n",
                    type == TrackerType::Sort ? "sort"
                    : type == TrackerType::ByteTrack ? "bytetrack" : "ocsort");
        testSingleObject(type);
        testCrossing(type);
        testDropout(type);
        testChurn(type);
    }

    if (g_failures == 0)
    {
        std::printf("ALGO TESTS OK\n");
        return 0;
    }
    std::printf("ALGO TESTS FAILED: %d failure(s)\n", g_failures);
    return 1;
}
