#include "SortTracker.h"
#include "core/Association.h"

#include <utility>

namespace motrack
{

std::vector<STrackPtr> SortTracker::update(const std::vector<Object>& objects)
{
    frame_id_++;

    // Keep only confident detections, wrap as candidate stracks.
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

    // Predict current pose by KF.
    for (auto &strack : tracked_stracks_)
    {
        strack->predict();
    }

    // Associate predictions with detections by IoU + Hungarian.
    std::vector<std::vector<int>> matches_idx;
    std::vector<int> unmatch_track_idx, unmatch_detection_idx;

    const auto dists = calcIouDistance(tracked_stracks_, det_stracks);
    linearAssignment(dists, tracked_stracks_.size(), det_stracks.size(), config_.match_thresh,
                     matches_idx, unmatch_track_idx, unmatch_detection_idx);

    std::vector<STrackPtr> current_tracked_stracks;
    current_tracked_stracks.reserve(tracked_stracks_.size());

    for (const auto &match_idx : matches_idx)
    {
        const auto track = tracked_stracks_[match_idx[0]];
        const auto det = det_stracks[match_idx[1]];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->update(*det, frame_id_);
        }
        else
        {
            track->reActivate(*det, frame_id_);
        }
        current_tracked_stracks.push_back(track);
    }

    // Seed new tracks from unmatched detections.
    for (const auto &unmatch_idx : unmatch_detection_idx)
    {
        const auto track = det_stracks[unmatch_idx];
        track->activate(frame_id_, nextTrackId());
        current_tracked_stracks.push_back(track);
    }

    // Age out tracks that were not matched this frame; drop them after max_age.
    std::vector<STrackPtr> still_tracked;
    still_tracked.reserve(current_tracked_stracks.size());
    for (const auto &unmatch_idx : unmatch_track_idx)
    {
        const auto track = tracked_stracks_[unmatch_idx];
        if (frame_id_ - track->getFrameId() > config_.max_age)
        {
            track->markAsRemoved();
        }
        else
        {
            still_tracked.push_back(track);
        }
    }

    tracked_stracks_ = std::move(still_tracked);
    for (const auto &track : current_tracked_stracks)
    {
        tracked_stracks_.push_back(track);
    }

    return tracked_stracks_;
}

}
