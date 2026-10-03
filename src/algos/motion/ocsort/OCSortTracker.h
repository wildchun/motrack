#pragma once
#include "core/BaseTracker.h"

namespace motrack
{

// OC-SORT (Observation-Centric SORT, CVPR 2022) - simplified C++11 port.
//
// Deviations from vanilla SORT, all observation-centric:
//   OCM  (Observation-Centric Momentum): association cost blends IoU distance
//        with a velocity-direction consistency term computed from the track's
//        last two observed boxes, penalizing direction changes during
//        association (helps under occlusion / non-linear motion).
//   OCR  (Observation-Centric Re-Update): when a lost track is re-found after
//        >= 3 missed frames, the KF is "virtually" re-fed the re-detection so
//        the velocity estimate recovers from the observation rather than
//        accumulating drift from virtual unobserved updates.
//   ORT  (Observation-based Recovery): lost tracks stay in the association
//        pool until max_age expires, using predicted boxes (ByteTrack-style
//        lost pool) instead of being dropped immediately.
class OCSortTracker : public BaseTracker
{
public:
    using BaseTracker::BaseTracker;

    std::vector<STrackPtr> update(const std::vector<Object>& objects) override;

private:
    std::vector<STrackPtr> tracked_stracks_;
    std::vector<STrackPtr> lost_stracks_;
};

}
