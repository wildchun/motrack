#pragma once
#include "core/BaseTracker.h"

namespace motrack
{

// JDE (Joint Detection and Embedding) style tracking - simplified C++11 port.
//
// In the original JDE the detector itself emits an embedding per box
// (detection + Re-ID share one network), but the *tracking* stage is what
// this class implements: assume every incoming Object already carries its
// `feature` (as produced by a JDE-style head) and associate in one shot.
//
// Pipeline per frame:
//   1. KF predict on tracked ∪ lost.
//   2. Single fused association: cost = lambda * IoU-motion +
//      (1 - lambda) * cosine-appearance, gated by appearance_thresh.
//      (JDE associates in one pass, unlike DeepSORT's cascade - the joint
//      embedding is trained to be discriminative enough that a single
//      round suffices.)
//   3. New tracks are seeded only from detections above high_thresh
//      (JDE's confidence-aware track initialization, ByteTrack-style).
//   4. Lost tracks retire after max_age.
//
// Difference vs DeepSortTracker: no cascade ladder, no IoU-only fallback
// round, confidence-gated initialization. Lower latency, relies on stronger
// embeddings.
class JDETracker : public BaseTracker
{
public:
    using BaseTracker::BaseTracker;

    std::vector<STrackPtr> update(const std::vector<Object>& objects) override;

private:
    std::vector<STrackPtr> tracked_stracks_;
    std::vector<STrackPtr> lost_stracks_;
};

}
