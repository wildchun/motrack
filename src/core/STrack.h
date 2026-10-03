#pragma once

#include "KalmanFilter.h"
#include "Motrack.h"
#include <cstddef>
#include <memory>
#include <vector>

namespace motrack
{

enum class STrackState {
    New = 0,
    Tracked = 1,
    Lost = 2,
    Removed = 3,
};

class STrack
{
public:
    STrack(const Object& object);
    ~STrack();

    const Object& getObject() const;
    const STrackState& getSTrackState() const;

    const bool& isActivated() const;
    const size_t& getTrackId() const;
    const size_t& getFrameId() const;
    const size_t& getStartFrameId() const;
    const size_t& getTrackletLength() const;

    void activate(const size_t& frame_id, const size_t& track_id);
    void reActivate(const STrack &new_track, const size_t &frame_id, const int &new_track_id = -1);

    void predict();
    void update(const STrack &new_track, const size_t &frame_id);

    void markAsLost();
    void markAsRemoved();

    // Appearance gallery (used by DeepSort / JDE style trackers). Motion-only
    // trackers simply never call addFeature.
    void addFeature(const std::vector<float>& feature, size_t budget);
    const std::vector<std::vector<float>>& features() const;

    // Observed motion direction (center delta of the last matched update),
    // used by OC-Sort's observation-centric momentum term.
    void recordMomentum(const Rect& observed);
    float momentumX() const { return momentum_x_; }
    float momentumY() const { return momentum_y_; }

private:
    KalmanFilter kalman_filter_;
    KalmanFilter::StateMean mean_;
    KalmanFilter::StateCov covariance_;

    Object object_;
    STrackState state_;

    std::vector<std::vector<float>> feature_gallery_;

    float momentum_x_ = 0.0f;
    float momentum_y_ = 0.0f;

    bool is_activated_;
    size_t track_id_;
    size_t frame_id_;
    size_t start_frame_id_;
    size_t tracklet_len_;

    void updateRect();
};

using STrackPtr = std::shared_ptr<STrack>;

}