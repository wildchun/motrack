#pragma once
#include <memory>
#include <utility>
#include <vector>

namespace motrack
{
struct Rect
{
    float x{}, y{}, width{}, height{};

    Rect() = default;
    Rect(float x_, float y_, float width_, float height_)
    : x(x_), y(y_), width(width_), height(height_) {};
};

struct Object
{
    float prob{};
    unsigned int label{};
    Rect rect{};
    // Optional appearance embedding (e.g. Re-ID feature). Empty for
    // motion-only algorithms (ByteTrack / Sort / OC-Sort); required for
    // DeepSort / JDE appearance association. All vectors must share the same
    // dimensionality within one run.
    std::vector<float> feature{};

    Object() = default;
    Object(float prob_, unsigned int label_, Rect rect_)
    : prob(prob_), label(label_), rect(rect_) {};
    Object(float prob_, unsigned int label_, Rect rect_, std::vector<float> feature_)
    : prob(prob_), label(label_), rect(rect_), feature(std::move(feature_)) {};
};

struct Track
{
    bool b_activated{};
    unsigned long long track_id{};
    unsigned long long frame_id{};
    Object object{};

    Track() = default;
    Track(bool b_activated_, unsigned long long track_id_, unsigned long long frame_id_, Object obj_)
    : b_activated(b_activated_), track_id(track_id_), frame_id(frame_id_), object(obj_) {};
};

// Supported tracking algorithms.
// Motion-only association: ByteTrack (two/three-stage IoU), Sort (single-stage
// IoU), OC-Sort (IoU + observation-centric re-update / momentum).
// Appearance-based association: DeepSort (cascade matching + Re-ID features),
// JDE (detection head already emits an embedding per box, joint association).
enum class TrackerType
{
    ByteTrack = 0,
    Sort      = 1,
    OCSort    = 2,
    DeepSort  = 3,
    JDE       = 4,
};

// One configuration struct shared by all algorithms. Each algorithm reads the
// fields it needs; unused fields are ignored.
struct TrackerConfig
{
    unsigned int max_age = 30;        // frames a track survives unmatched
    float track_thresh = 0.5f;        // high/low detection split (ByteTrack) or det filter (Sort)
    float high_thresh  = 0.6f;        // min prob to seed a new track (ByteTrack)
    float match_thresh = 0.8f;        // IoU-cost gate for assignment (1 - IoU >= thresh rejected)

    // ---- appearance / motion-fusion parameters (DeepSort / JDE / OC-Sort) ----
    float appearance_thresh = 0.3f;   // cosine-distance gate for appearance matching
    float lambda_weight     = 0.98f;  // weight mixing motion and appearance costs: cost = lambda*motion + (1-lambda)*appearance
    unsigned int feature_dim = 512;   // expected Re-ID embedding dimensionality (0 = infer at runtime)
    unsigned int feature_budget = 100;// max stored embeddings per track (FIFO)
};

class BaseTracker;

// Unified facade. Same simple API as the original ByteTracker: construct with
// an algorithm type, feed detections per frame, get tracks back.
class Tracker
{
public:
    Tracker(TrackerType type, const TrackerConfig& config = TrackerConfig());
    ~Tracker();

    std::vector<Track> update(const std::vector<Object>& objects);

    TrackerType type() const { return type_; }

private:
    TrackerType type_;
    std::shared_ptr<BaseTracker> impl_;
};

// Factory helper, equivalent to constructing Tracker directly.
std::shared_ptr<Tracker> createTracker(TrackerType type, const TrackerConfig& config = TrackerConfig());
}
