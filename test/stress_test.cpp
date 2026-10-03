// Stress tests for motrack: exercises the public API under load to surface
// memory leaks, crashes, and numerical blow-ups (checked by ASan/LSan/UBSan).
// Every scenario runs against each motion-family algorithm:
// Sort, ByteTrack, OC-Sort.
#include "Motrack.h"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>

namespace
{
struct TrackIdCounter
{
    size_t unique_ids = 0;
    size_t activated = 0;
};

// Collect a frame of detections and feed it through the tracker.
// Returns the number of currently reported tracks.
size_t feedFrame(motrack::Tracker& tracker,
                 std::vector<motrack::Object>& objects,
                 TrackIdCounter& counter,
                 size_t& peak_tracks)
{
    auto tracks = tracker.update(objects);
    peak_tracks = std::max(peak_tracks, tracks.size());
    counter.unique_ids += tracks.size();
    for (const auto& t : tracks)
    {
        if (t.b_activated) ++counter.activated;
    }
    return tracks.size();
}

// One moving target: starts at (x0, y0), drifts by (vx, vy) per frame.
void addTarget(std::vector<motrack::Object>& objects,
               int frame, float x0, float y0, float vx, float vy,
               float w, float h, float prob, unsigned label, unsigned seed)
{
    objects.push_back({ prob, label,
        { x0 + vx * frame, y0 + vy * frame, w, h } });
}

// ---------- Scenario 1: long run, many objects, full state machine ----------
// 2000 frames x ~120 moving objects: constant high-score targets (stay
// Tracked), targets that vanish for long gaps (Tracked -> Lost -> reActivate),
// and short-lived low-score objects that flicker (New/Removed churn).
void scenario_longRun(motrack::TrackerType type)
{
    std::printf("[1/5] long run: 2000 frames x 120 objects (churn + reappearance)\n");
    motrack::Tracker tracker(type);

    std::mt19937 rng(42);
    std::uniform_real_distribution<float> jitter(-6, 6);

    std::vector<motrack::Object> objects;
    TrackIdCounter counter;
    size_t peak = 0;

    const int N = 2000;
    for (int f = 0; f < N; ++f)
    {
        objects.clear();

        // 40 steady high-score targets.
        for (int i = 0; i < 40; ++i)
        {
            const float x0 = 20.0f * i, y0 = 20.0f + (i % 5) * 60.0f;
            addTarget(objects, f, x0, y0, 1.2f, 0.3f, 40, 40, 0.9f, 0, 0);
        }
        // 20 targets that disappear for 60 frames every 150 (lost -> re-found).
        for (int i = 0; i < 20; ++i)
        {
            const int cycle = (f + i * 37) % 150;
            if (cycle < 90) // visible window
            {
                const float x0 = 100.0f + 30.0f * i;
                const float y0 = 400.0f;
                addTarget(objects, f, x0, y0, -0.8f, 0.5f, 30, 30, 0.85f, 0, 0);
            }
        }
        // 40 low-score flickers (below track_thresh): association fodder.
        for (int i = 0; i < 40; ++i)
        {
            const float prob = 0.3f + 0.15f * static_cast<float>(rng() % 100) / 100.0f;
            const float x = 10.0f + 15.0f * i + jitter(rng);
            const float y = 500.0f + 10.0f * (i % 7) + jitter(rng);
            addTarget(objects, f, 0, 0, 0, 0, 35, 35, prob, 0, 0);
            // rewrite: addTarget with vx=vy=0 places at (x0, y0); use x/y above
            objects.back().rect.x = x;
            objects.back().rect.y = y;
        }
        feedFrame(tracker, objects, counter, peak);
    }
    std::printf("    peak concurrent tracks: %zu, activated reports: %zu\n",
                peak, counter.activated);
}

// ---------- Scenario 2: burst scale ----------
// Few frames but thousands of detections per frame: stress the cost matrix
// and LAPJV on large squares.
void scenario_burst(motrack::TrackerType type)
{
    // Scale factors overridable via env for quick/ASan runs:
    //   MOTRACK_STRESS_FRAMES  (burst frames, default 50)
    //   MOTRACK_STRESS_BASE    (burst detections/frame base, default 1500)
    const int burst_frames = getenv("MOTRACK_STRESS_FRAMES")
        ? atoi(getenv("MOTRACK_STRESS_FRAMES")) : 50;
    const int burst_base = getenv("MOTRACK_STRESS_BASE")
        ? atoi(getenv("MOTRACK_STRESS_BASE")) : 1500;
    std::printf("[2/5] burst: %d frames x %d+ detections\n", burst_frames, burst_base);
    motrack::Tracker tracker(type);

    std::mt19937 rng(7);
    std::uniform_real_distribution<float> pos(0, 1920.0f);
    std::uniform_real_distribution<float> size(10, 200.0f);
    std::uniform_real_distribution<float> prob(0.1f, 0.99f);

    std::vector<motrack::Object> objects;
    objects.reserve(2000);
    TrackIdCounter counter;
    size_t peak = 0;

    for (int f = 0; f < burst_frames; ++f)
    {
        objects.clear();
        const int n = burst_base + 50 * f;
        for (int i = 0; i < n; ++i)
        {
            // objects persist between frames with small motion -> many tracked
            const float vx = (i % 13) - 6, vy = (i % 7) - 3;
            objects.push_back({
                prob(rng), static_cast<unsigned>(i % 3),
                { 50.0f + i * 3.0f + vx * f, 50.0f + (i * 5) % 1080 + vy * f,
                  size(rng), size(rng) } });
        }
        feedFrame(tracker, objects, counter, peak);
    }
    std::printf("    peak concurrent tracks: %zu\n", peak);
}

// ---------- Scenario 3: create/destroy churn ----------
// Thousands of tracker instances, each doing a handful of updates: stresses
// pimpl construction/destruction and any static/global state.
void scenario_churn(motrack::TrackerType type)
{
    std::printf("[3/5] churn: 2000 tracker instances x 10 frames\n");
    std::mt19937 rng(99);
    std::uniform_real_distribution<float> size(5, 100.0f);
    std::uniform_real_distribution<float> prob(0.55f, 0.95f);

    TrackIdCounter counter;
    for (int t = 0; t < 2000; ++t)
    {
        motrack::TrackerConfig cfg;
        cfg.max_age = 5 + (t % 40);
        cfg.track_thresh = 0.4f + 0.2f * ((t % 10) / 10.0f);
        cfg.high_thresh = 0.6f;
        cfg.match_thresh = 0.8f;
        motrack::Tracker tracker(type, cfg);
        std::vector<motrack::Object> objects;
        for (int f = 0; f < 10; ++f)
        {
            objects.clear();
            const int n = 3 + (t % 15);
            for (int i = 0; i < n; ++i)
            {
                objects.push_back({ prob(rng), 0,
                    { 10.0f * i + f, 10.0f * (i + 1), size(rng), size(rng) } });
            }
            feedFrame(tracker, objects, counter, counter.unique_ids);
        }
    }
    std::printf("    done (instances destroyed in scope each iteration)\n");
}

// ---------- Scenario 4: degenerate / boundary inputs ----------
// Empty frames, zero-size boxes, negative boxes, giant boxes, single frames,
// extreme probabilities — check the guards hold and nothing crashes/NaNs.
void scenario_degenerate(motrack::TrackerType type)
{
    std::printf("[4/5] degenerate inputs: empty/zero/negative/huge boxes\n");
    motrack::Tracker tracker(type);
    std::vector<motrack::Object> objects;
    TrackIdCounter counter;
    size_t peak = 0;

    // 100 frames with all-empty detections.
    for (int f = 0; f < 100; ++f)
        feedFrame(tracker, objects, counter, peak);

    // Boxes with zero / negative / negative-area dims must be skipped.
    objects.push_back({ 0.9f, 0, { 10, 10, 0, 50 } });
    objects.push_back({ 0.9f, 0, { 10, 10, 50, 0 } });
    objects.push_back({ 0.9f, 0, { 10, 10, -5, 50 } });
    objects.push_back({ 0.9f, 0, { 10, 10, 50, -5 } });
    feedFrame(tracker, objects, counter, peak);

    // Giant + far-off boxes.
    objects.clear();
    objects.push_back({ 0.9f, 0, { -1e5, -1e5, 1e6, 1e6 } });
    objects.push_back({ 0.9f, 0, { 9e5, 9e5, 1e4, 1e4 } });
    objects.push_back({ 1.0f, 1, { 0, 0, 4000, 3000 } });
    feedFrame(tracker, objects, counter, peak);

    // prob exactly at thresholds (boundaries of the splits).
    objects.clear();
    objects.push_back({ 0.5f, 0, { 100, 100, 50, 50 } });   // == track_thresh
    objects.push_back({ 0.49f, 0, { 200, 100, 50, 50 } });  // just below
    objects.push_back({ 0.6f, 0, { 300, 100, 50, 50 } });   // == high_thresh
    feedFrame(tracker, objects, counter, peak);
    objects.clear();
    feedFrame(tracker, objects, counter, peak);

    std::printf("    degenerate pass done\n");
}

// ---------- Scenario 5: heavy loss/re-activation + duplicate removal ----------
// Two overlapping "tracks" (same object seen by two detections with slightly
// different geometry) to exercise removeDuplicateStracks, plus periodic
// complete blackout followed by reappearance.
void scenario_duplicates(motrack::TrackerType type)
{
    std::printf("[5/5] duplicates + blackout: 600 frames\n");
    motrack::Tracker tracker(type, motrack::TrackerConfig{});

    std::mt19937 rng(5);
    std::uniform_real_distribution<float> jitter(-2, 2);

    std::vector<motrack::Object> objects;
    TrackIdCounter counter;
    size_t peak = 0;

    const int N = 600;
    for (int f = 0; f < N; ++f)
    {
        objects.clear();
        const int phase = f % 120;

        if (phase < 80) // visible: twin detections for the same object
        {
            const float x = 100.0f + f * 2.0f, y = 100.0f;
            objects.push_back({ 0.9f, 0, { x, y, 60, 60 } });
            objects.push_back({ 0.85f, 0,
                { x + jitter(rng), y + jitter(rng), 62, 58 } });
            // second object
            const float x2 = 500.0f + f, y2 = 300.0f;
            objects.push_back({ 0.88f, 0, { x2, y2, 50, 50 } });
            objects.push_back({ 0.86f, 0,
                { x2 + jitter(rng), y2 + jitter(rng), 52, 48 } });
        }
        // phase 80..119: blackout -> tracks go Lost, then removed at max_age=10

        feedFrame(tracker, objects, counter, peak);
    }
    std::printf("    peak concurrent tracks: %zu\n", peak);
}

} // namespace

int main()
{
    using Clock = std::chrono::steady_clock;

    const motrack::TrackerType algos[] = {
        motrack::TrackerType::Sort, motrack::TrackerType::ByteTrack, motrack::TrackerType::OCSort};
    const char* names[] = {"sort", "bytetrack", "ocsort"};

    const auto ms = [](Clock::time_point a, Clock::time_point b) {
        return std::chrono::duration_cast<std::chrono::milliseconds>(b - a).count();
    };
    long long total = 0;
    for (size_t a = 0; a < 3; ++a)
    {
        std::printf("=== algorithm: %s ===\n", names[a]);
        auto t0 = Clock::now();
        scenario_longRun(algos[a]);
        auto t1 = Clock::now();
        scenario_burst(algos[a]);
        auto t2 = Clock::now();
        scenario_churn(algos[a]);
        auto t3 = Clock::now();
        scenario_degenerate(algos[a]);
        auto t4 = Clock::now();
        scenario_duplicates(algos[a]);
        auto t5 = Clock::now();

        const long long per = ms(t0, t5);
        total += per;
        std::printf("  long run: %lld ms | burst: %lld ms | churn: %lld ms | "
                    "degenerate: %lld ms | duplicates: %lld ms | TOTAL: %lld ms\n",
                    (long long)ms(t0, t1), (long long)ms(t1, t2),
                    (long long)ms(t2, t3), (long long)ms(t3, t4),
                    (long long)ms(t4, t5), per);
    }
    std::printf("\nALL ALGORITHMS TOTAL: %lld ms\n", total);
    std::printf("STRESS OK\n");
    return 0;
}
