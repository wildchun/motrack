#pragma once
#include "core/BaseTracker.h"

namespace motrack
{

// SORT (Simple Online and Realtime Tracking): single-stage association.
// Kalman prediction vs detections by IoU cost + Hungarian assignment.
// Only detections with prob >= track_thresh participate; unmatched tracks are
// kept for max_age frames then removed. No lost-track re-association pass.
class SortTracker : public BaseTracker
{
public:
    using BaseTracker::BaseTracker;

    std::vector<STrackPtr> update(const std::vector<Object>& objects) override;

private:
    std::vector<STrackPtr> tracked_stracks_;
};

}
