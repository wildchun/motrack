#pragma once
#include "core/BaseTracker.h"

namespace motrack
{

// DeepSORT - simplified C++11 port (cosine-metric variant).
//
// Pipeline per frame:
//   1. KF predict on tracked ∪ lost (cascade pool).
//   2. CASCADE round 1: tracks whose time_since_update <= cascade_age are
//      matched against detections with a fused cost
//      cost = lambda * mahalanobis-motion + (1 - lambda) * cosine-appearance,
//      gated by appearance_thresh. Matching by descending cascade age lets
//      recently-seen tracks claim ambiguous detections first (the classic
//      DeepSORT cascade, implemented as a threshold ladder).
//   3. CASCADE round 2: remaining tracks fall back to pure IoU matching
//      (keeps tracks alive when the Re-ID feature is missing/unstable).
//   4. Unmatched high-score detections seed new tracks; unmatched tracks are
//      kept in a lost pool until max_age.
//
// Requires detections to carry `Object::feature`; entries without features
// gracefully degrade to IoU-only association.
class DeepSortTracker : public BaseTracker
{
public:
    using BaseTracker::BaseTracker;

    std::vector<STrackPtr> update(const std::vector<Object>& objects) override;

private:
    std::vector<STrackPtr> tracked_stracks_;
    std::vector<STrackPtr> lost_stracks_;
};

}
