#include "ByteTrackTracker.h"
#include "core/Association.h"

#include <algorithm>
#include <utility>

namespace motrack {

std::vector<STrackPtr> ByteTrackTracker::update(const std::vector<Object>& objects)
{
    ////////////////// Step 1: Get detections //////////////////
    frame_id_++;

    // Create new STracks using the result of object detection
    std::vector<STrackPtr> det_stracks;
    std::vector<STrackPtr> det_low_stracks;
    det_stracks.reserve(objects.size());
    det_low_stracks.reserve(objects.size());

    for (const auto &object : objects)
    {
        if(object.rect.width <= 0 || object.rect.height <= 0)
            continue;
        const auto strack = std::make_shared<STrack>(object);
        if (object.prob >= config_.track_thresh)
        {
            det_stracks.push_back(strack);
        }
        else
        {
            det_low_stracks.push_back(strack);
        }
    }

    // Split existing tracks into activated vs unconfirmed.
    std::vector<STrackPtr> active_stracks;
    std::vector<STrackPtr> non_active_stracks;
    active_stracks.reserve(tracked_stracks_.size());
    non_active_stracks.reserve(tracked_stracks_.size());

    for (const auto& tracked_strack : tracked_stracks_)
    {
        if (!tracked_strack->isActivated())
        {
            non_active_stracks.push_back(tracked_strack);
        }
        else
        {
            active_stracks.push_back(tracked_strack);
        }
    }

    // Build the association pool. `active_stracks` is a subset of tracked_stracks_
    // (already deduplicated by removeDuplicateStracks at end of previous frame) and
    // lost_stracks_ is kept disjoint from tracked_stracks_ in step 5, so a plain
    // concat here is safe and skips the hash pass jointStracks would do.
    std::vector<STrackPtr> strack_pool;
    strack_pool.reserve(active_stracks.size() + lost_stracks_.size());
    strack_pool.insert(strack_pool.end(), active_stracks.begin(), active_stracks.end());
    strack_pool.insert(strack_pool.end(), lost_stracks_.begin(),  lost_stracks_.end());

    // Predict current pose by KF
    for (auto &strack : strack_pool)
    {
        strack->predict();
    }

    ////////////////// Step 2: First association, with IoU //////////////////
    std::vector<STrackPtr> current_tracked_stracks;
    std::vector<STrackPtr> remain_tracked_stracks;
    std::vector<STrackPtr> remain_det_stracks;
    std::vector<STrackPtr> refind_stracks;
    current_tracked_stracks.reserve(strack_pool.size() + det_stracks.size());
    remain_tracked_stracks.reserve(strack_pool.size());
    remain_det_stracks.reserve(det_stracks.size());
    refind_stracks.reserve(strack_pool.size());

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_detection_idx, unmatch_track_idx;

        const auto dists = calcIouDistance(strack_pool, det_stracks);
        linearAssignment(dists, strack_pool.size(), det_stracks.size(), config_.match_thresh,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = strack_pool[match_idx[0]];
            const auto det = det_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            remain_det_stracks.push_back(det_stracks[unmatch_idx]);
        }

        // NOTE: only unmatched *Tracked* tracks flow into the second (low-score) pass.
        // Unmatched *Lost* tracks are intentionally not pushed anywhere here -- they
        // are carried over via lost_stracks_ in step 5's subStracks/jointStracks chain.
        for (const auto &unmatch_idx : unmatch_track_idx)
        {
            if (strack_pool[unmatch_idx]->getSTrackState() == STrackState::Tracked)
            {
                remain_tracked_stracks.push_back(strack_pool[unmatch_idx]);
            }
        }
    }

    ////////////////// Step 3: Second association, using low score dets //////////////////
    std::vector<STrackPtr> current_lost_stracks;
    current_lost_stracks.reserve(remain_tracked_stracks.size());

    {
        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_track_idx, unmatch_detection_idx;

        const auto dists = calcIouDistance(remain_tracked_stracks, det_low_stracks);
        linearAssignment(dists, remain_tracked_stracks.size(), det_low_stracks.size(), 0.5,
                         matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const auto track = remain_tracked_stracks[match_idx[0]];
            const auto det = det_low_stracks[match_idx[1]];
            if (track->getSTrackState() == STrackState::Tracked)
            {
                track->update(*det, frame_id_);
                current_tracked_stracks.push_back(track);
            }
            else
            {
                track->reActivate(*det, frame_id_);
                refind_stracks.push_back(track);
            }
        }

        for (const auto &unmatch_track : unmatch_track_idx)
        {
            const auto track = remain_tracked_stracks[unmatch_track];
            if (track->getSTrackState() != STrackState::Lost)
            {
                track->markAsLost();
                current_lost_stracks.push_back(track);
            }
        }
    }

    ////////////////// Step 4: Init new stracks //////////////////
    std::vector<STrackPtr> current_removed_stracks;
    current_removed_stracks.reserve(non_active_stracks.size() + lost_stracks_.size());

    {
        std::vector<int> unmatch_detection_idx;
        std::vector<int> unmatch_unconfirmed_idx;
        std::vector<std::vector<int>> matches_idx;

        // Deal with unconfirmed tracks, usually tracks with only one beginning frame
        const auto dists = calcIouDistance(non_active_stracks, remain_det_stracks);
        linearAssignment(dists, non_active_stracks.size(), remain_det_stracks.size(), 0.7,
                         matches_idx, unmatch_unconfirmed_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            non_active_stracks[match_idx[0]]->update(*remain_det_stracks[match_idx[1]], frame_id_);
            current_tracked_stracks.push_back(non_active_stracks[match_idx[0]]);
        }

        for (const auto &unmatch_idx : unmatch_unconfirmed_idx)
        {
            const auto track = non_active_stracks[unmatch_idx];
            track->markAsRemoved();
            current_removed_stracks.push_back(track);
        }

        // Add new stracks
        for (const auto &unmatch_idx : unmatch_detection_idx)
        {
            const auto track = remain_det_stracks[unmatch_idx];
            if (track->getObject().prob < config_.high_thresh)
            {
                continue;
            }
            track->activate(frame_id_, nextTrackId());
            current_tracked_stracks.push_back(track);
        }
    }

    ////////////////// Step 5: Update state //////////////////
    for (const auto &lost_strack : lost_stracks_)
    {
        if (frame_id_ - lost_strack->getFrameId() > config_.max_age)
        {
            lost_strack->markAsRemoved();
            current_removed_stracks.push_back(lost_strack);
        }
    }

    tracked_stracks_ = jointStracks(current_tracked_stracks, refind_stracks);

    // Update lost pool: drop tracks that got re-found this frame, add fresh losses,
    // then drop anything marked removed.
    auto still_lost = subStracks(lost_stracks_, tracked_stracks_);
    still_lost      = jointStracks(still_lost, current_lost_stracks);
    lost_stracks_   = subStracks(still_lost, current_removed_stracks);

    std::vector<STrackPtr> tracked_stracks_out, lost_stracks_out;
    removeDuplicateStracks(tracked_stracks_, lost_stracks_, tracked_stracks_out, lost_stracks_out);
    tracked_stracks_ = std::move(tracked_stracks_out);
    lost_stracks_    = std::move(lost_stracks_out);

    std::vector<STrackPtr> output_stracks;
    output_stracks.reserve(tracked_stracks_.size());
    for (const auto &track : tracked_stracks_)
    {
        output_stracks.push_back(track);
    }

    return output_stracks;
}

}
