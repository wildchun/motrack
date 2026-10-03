#include "JDETracker.h"
#include "core/Association.h"
#include "core/FeatureMetric.h"

#include <utility>

namespace motrack
{

std::vector<STrackPtr> JDETracker::update(const std::vector<Object>& objects)
{
    frame_id_++;

    std::vector<STrackPtr> det_stracks;
    det_stracks.reserve(objects.size());
    for (const auto &object : objects)
    {
        if (object.rect.width <= 0 || object.rect.height <= 0)
            continue;
        if (object.prob < config_.track_thresh)
            continue;
        det_stracks.push_back(std::make_shared<STrack>(object));
    }

    std::vector<STrackPtr> strack_pool = jointStracks(tracked_stracks_, lost_stracks_);
    for (auto &strack : strack_pool)
    {
        strack->predict();
    }

    // Single-shot fused association (JDE's one-pass design).
    const auto iou_cost = calcIouDistance(strack_pool, det_stracks);
    const auto appearance_cost = FeatureMetric::cosineDistance(strack_pool, det_stracks);
    const auto cost = FeatureMetric::fuseCost(iou_cost, appearance_cost,
                                              config_.lambda_weight, config_.appearance_thresh);

    std::vector<std::vector<int>> matches_idx;
    std::vector<int> unmatch_track_idx, unmatch_detection_idx;
    linearAssignment(cost, strack_pool.size(), det_stracks.size(), config_.match_thresh,
                     matches_idx, unmatch_track_idx, unmatch_detection_idx);

    std::vector<STrackPtr> current_tracked_stracks;
    std::vector<STrackPtr> refind_stracks;
    std::vector<STrackPtr> current_lost_stracks;

    for (const auto &match_idx : matches_idx)
    {
        const auto track = strack_pool[match_idx[0]];
        const auto det = det_stracks[match_idx[1]];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->update(*det, frame_id_);
        }
        else
        {
            track->reActivate(*det, frame_id_);
            refind_stracks.push_back(track);
        }
        track->addFeature(det->getObject().feature, config_.feature_budget);
        current_tracked_stracks.push_back(track);
    }

    // Unmatched tracks -> lost pool.
    for (const auto &unmatch_idx : unmatch_track_idx)
    {
        const auto track = strack_pool[unmatch_idx];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->markAsLost();
            current_lost_stracks.push_back(track);
        }
    }

    // Unmatched detections: confidence-gated initialization. Only detections
    // above high_thresh seed new tracks; weaker boxes are discarded (they are
    // more likely false positives since JDE has no low-score rescue pass).
    for (const auto &unmatch_idx : unmatch_detection_idx)
    {
        const auto track = det_stracks[unmatch_idx];
        if (track->getObject().prob < config_.high_thresh)
        {
            continue;
        }
        track->activate(frame_id_, nextTrackId());
        track->addFeature(track->getObject().feature, config_.feature_budget);
        current_tracked_stracks.push_back(track);
    }

    // Retire lost tracks older than max_age.
    std::vector<STrackPtr> removed_stracks;
    for (const auto &lost_strack : lost_stracks_)
    {
        if (frame_id_ - lost_strack->getFrameId() > config_.max_age)
        {
            lost_strack->markAsRemoved();
            removed_stracks.push_back(lost_strack);
        }
    }

    tracked_stracks_ = jointStracks(current_tracked_stracks, refind_stracks);
    auto still_lost = subStracks(lost_stracks_, tracked_stracks_);
    still_lost = jointStracks(still_lost, current_lost_stracks);
    lost_stracks_ = subStracks(still_lost, removed_stracks);

    return tracked_stracks_;
}

}
