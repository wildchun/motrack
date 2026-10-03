#include "DeepSortTracker.h"
#include "core/Association.h"
#include "core/FeatureMetric.h"

#include <algorithm>
#include <utility>

namespace motrack
{

std::vector<STrackPtr> DeepSortTracker::update(const std::vector<Object>& objects)
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

    const auto iou_cost = calcIouDistance(strack_pool, det_stracks);
    const auto appearance_cost = FeatureMetric::cosineDistance(strack_pool, det_stracks);

    std::vector<STrackPtr> current_tracked_stracks;
    std::vector<STrackPtr> refind_stracks;
    std::vector<STrackPtr> current_lost_stracks;
    std::vector<bool> det_taken(det_stracks.size(), false);
    std::vector<bool> track_done(strack_pool.size(), false);

    // ---- Cascade round 1: appearance-aware matching, newest tracks first ----
    // DeepSORT's cascade prioritizes tracks with small time_since_update so
    // recently-seen tracks win ambiguous detections. We realize the ladder by
    // assigning rounds: age bucket r gets tracks missing exactly r frames.
    const int max_cascade = std::min<int>(config_.max_age, 30);
    for (int age = 0; age <= max_cascade; ++age)
    {
        std::vector<size_t> round_tracks;
        std::vector<size_t> round_dets;
        for (size_t i = 0; i < strack_pool.size(); ++i)
        {
            if (track_done[i])
                continue;
            const int missed = static_cast<int>(frame_id_ - strack_pool[i]->getFrameId());
            if (missed == age)
            {
                round_tracks.push_back(i);
            }
        }
        for (size_t j = 0; j < det_stracks.size(); ++j)
        {
            if (!det_taken[j])
            {
                round_dets.push_back(j);
            }
        }
        if (round_tracks.empty() || round_dets.empty())
        {
            continue;
        }

        // Fused cost restricted to this cascade round.
        std::vector<std::vector<float>> round_cost(round_tracks.size(),
                                                   std::vector<float>(round_dets.size(), 1.0f));
        for (size_t a = 0; a < round_tracks.size(); ++a)
        {
            for (size_t b = 0; b < round_dets.size(); ++b)
            {
                round_cost[a][b] = FeatureMetric::fuseCost(
                    {iou_cost[round_tracks[a]]}, {appearance_cost[round_tracks[a]]},
                    config_.lambda_weight, config_.appearance_thresh)[0][round_dets[b]];
            }
        }

        std::vector<std::vector<int>> matches_idx;
        std::vector<int> unmatch_track_idx, unmatch_detection_idx;
        linearAssignment(round_cost, round_tracks.size(), round_dets.size(),
                         config_.match_thresh, matches_idx, unmatch_track_idx, unmatch_detection_idx);

        for (const auto &match_idx : matches_idx)
        {
            const size_t ti = round_tracks[match_idx[0]];
            const size_t dj = round_dets[match_idx[1]];
            track_done[ti] = true;
            det_taken[dj] = true;

            const auto track = strack_pool[ti];
            const auto det = det_stracks[dj];
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
    }

    // ---- Cascade round 2: IoU-only fallback (feature missing or gated out) ----
    {
        std::vector<size_t> round_tracks, round_dets;
        for (size_t i = 0; i < strack_pool.size(); ++i)
        {
            if (!track_done[i]) round_tracks.push_back(i);
        }
        for (size_t j = 0; j < det_stracks.size(); ++j)
        {
            if (!det_taken[j]) round_dets.push_back(j);
        }
        if (!round_tracks.empty() && !round_dets.empty())
        {
            std::vector<std::vector<float>> round_cost(round_tracks.size(),
                                                       std::vector<float>(round_dets.size(), 1.0f));
            for (size_t a = 0; a < round_tracks.size(); ++a)
            {
                round_cost[a] = iou_cost[round_tracks[a]];
                // Compact to the still-available detection columns.
                std::vector<float> compact;
                compact.reserve(round_dets.size());
                for (size_t b = 0; b < round_dets.size(); ++b)
                {
                    compact.push_back(round_cost[a][round_dets[b]]);
                }
                round_cost[a] = std::move(compact);
            }

            std::vector<std::vector<int>> matches_idx;
            std::vector<int> unmatch_track_idx, unmatch_detection_idx;
            linearAssignment(round_cost, round_tracks.size(), round_dets.size(),
                             config_.match_thresh, matches_idx, unmatch_track_idx, unmatch_detection_idx);

            for (const auto &match_idx : matches_idx)
            {
                const size_t ti = round_tracks[match_idx[0]];
                const size_t dj = round_dets[match_idx[1]];
                const auto track = strack_pool[ti];
                const auto det = det_stracks[dj];
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
        }
    }

    // Unmatched tracks -> lost pool.
    for (size_t i = 0; i < strack_pool.size(); ++i)
    {
        if (track_done[i])
            continue;
        const auto track = strack_pool[i];
        if (track->getSTrackState() == STrackState::Tracked)
        {
            track->markAsLost();
            current_lost_stracks.push_back(track);
        }
    }

    // Unmatched detections -> new tracks (feature gallery seeded here).
    for (size_t j = 0; j < det_stracks.size(); ++j)
    {
        if (det_taken[j])
            continue;
        const auto track = det_stracks[j];
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
