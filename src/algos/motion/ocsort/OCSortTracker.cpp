#include "OCSortTracker.h"
#include "core/Association.h"

#include <cmath>
#include <utility>

namespace motrack
{

namespace
{

// Observation-Centric Momentum: penalize candidate pairs where the
// track->detection direction disagrees with the track's recent motion
// direction (angle between the two vectors, in [0, 1] normalized cost).
float momentumCost(const STrackPtr& track, const STrackPtr& det)
{
    const Object& cur = track->getObject();
    const Object& cand = det->getObject();

    // Track velocity hint: if we recorded the previous observation center,
    // derive direction from the KF velocity components via the predicted box
    // delta. We approximate direction with the KF mean velocities (vx, vy),
    // which STrack exposes indirectly; here we use the delta between the
    // current predicted box and the last updated box instead.
    const float vxc = cand.rect.x + cand.rect.width / 2 - (cur.rect.x + cur.rect.width / 2);
    const float vyc = cand.rect.y + cand.rect.height / 2 - (cur.rect.y + cur.rect.height / 2);
    const float norm_c = std::sqrt(vxc * vxc + vyc * vyc);

    // Previous direction is stored implicitly: use the track's frame-to-frame
    // motion captured in the object rect versus its Kalman prediction. With no
    // history available, no momentum penalty applies.
    static const float kNoHistory = 0.0f;
    const float prev_vx = track->momentumX();
    const float prev_vy = track->momentumY();
    const float norm_p = std::sqrt(prev_vx * prev_vx + prev_vy * prev_vy);

    if (norm_c <= 1e-6f || norm_p <= 1e-6f)
    {
        return kNoHistory;
    }

    const float cos_sim = (vxc * prev_vx + vyc * prev_vy) / (norm_c * norm_p);
    // Map cos similarity [-1, 1] to a cost [0, 1]: 0 = same direction.
    return (1.0f - cos_sim) * 0.5f;
}

} // namespace

std::vector<STrackPtr> OCSortTracker::update(const std::vector<Object>& objects)
{
    frame_id_++;

    // Build detection candidates (OC-SORT is motion-only: features ignored).
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

    // ORT: keep lost tracks in the association pool until max_age.
    std::vector<STrackPtr> strack_pool = jointStracks(tracked_stracks_, lost_stracks_);
    for (auto &strack : strack_pool)
    {
        strack->predict();
    }

    // Cost matrix: IoU distance blended with the OCM momentum term.
    const auto iou_cost = calcIouDistance(strack_pool, det_stracks);
    std::vector<std::vector<float>> cost = iou_cost;
    for (size_t i = 0; i < strack_pool.size(); ++i)
    {
        for (size_t j = 0; j < det_stracks.size(); ++j)
        {
            cost[i][j] = std::min(1.0f, iou_cost[i][j] + 0.2f * momentumCost(strack_pool[i], det_stracks[j]));
        }
    }

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

        // OCR: a track re-found after a long blackout re-updates its motion
        // model against the fresh observation instead of trusting the
        // drifted virtual trajectory.
        const bool was_lost = track->getSTrackState() == STrackState::Lost;
        if (was_lost)
        {
            track->reActivate(*det, frame_id_);
            refind_stracks.push_back(track);
        }
        else
        {
            track->update(*det, frame_id_);
        }
        // Record the observed motion direction for the next OCM round.
        track->recordMomentum(det->getObject().rect);
        current_tracked_stracks.push_back(track);
    }

    // Unmatched tracks: mark lost (they stay in the pool for ORT).
    for (const auto &unmatch_idx : unmatch_track_idx)
    {
        const auto track = strack_pool[unmatch_idx];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->markAsLost();
            current_lost_stracks.push_back(track);
        }
    }

    // Unmatched detections seed new tracks.
    for (const auto &unmatch_idx : unmatch_detection_idx)
    {
        const auto track = det_stracks[unmatch_idx];
        track->activate(frame_id_, nextTrackId());
        track->recordMomentum(track->getObject().rect);
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
