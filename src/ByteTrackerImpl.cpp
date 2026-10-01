#include "ByteTrackerImpl.h"
#include "lapjv.h"
#include <algorithm>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include <unordered_set>
#include <utility>
#include <vector>

namespace bytetrack {

////////////////////////////////////////////////////////////////////////////////
// tracks operator
///////////////////////////////////////////////////////////////////////////////

// Union by track_id, preserving `a`'s order followed by new tracks from `b`.
std::vector<STrackPtr> jointStracks(const std::vector<STrackPtr> &a_tlist,
                                    const std::vector<STrackPtr> &b_tlist)
{
    std::unordered_set<size_t> seen;
    seen.reserve(a_tlist.size() + b_tlist.size());

    std::vector<STrackPtr> res;
    res.reserve(a_tlist.size() + b_tlist.size());

    for (const auto &t : a_tlist)
    {
        seen.insert(t->getTrackId());
        res.push_back(t);
    }
    for (const auto &t : b_tlist)
    {
        if (seen.insert(t->getTrackId()).second)
        {
            res.push_back(t);
        }
    }
    return res;
}

// Set difference by track_id, preserving `a`'s order.
std::vector<STrackPtr> subStracks(const std::vector<STrackPtr> &a_tlist,
                                  const std::vector<STrackPtr> &b_tlist)
{
    std::unordered_set<size_t> remove_ids;
    remove_ids.reserve(b_tlist.size());
    for (const auto &t : b_tlist)
    {
        remove_ids.insert(t->getTrackId());
    }

    std::vector<STrackPtr> res;
    res.reserve(a_tlist.size());
    for (const auto &t : a_tlist)
    {
        if (remove_ids.count(t->getTrackId()) == 0)
        {
            res.push_back(t);
        }
    }
    return res;
}

// Compute IoU distance (1 - IoU) directly from STrack pairs.
// Returns an empty matrix when either side is empty.
std::vector<std::vector<float>> calcIouDistance(const std::vector<STrackPtr> &a_tracks,
                                                const std::vector<STrackPtr> &b_tracks)
{
    if (a_tracks.empty() || b_tracks.empty())
    {
        return {};
    }

    std::vector<std::vector<float>> cost(a_tracks.size(),
                                         std::vector<float>(b_tracks.size(), 1.0f));

    for (size_t i = 0; i < a_tracks.size(); ++i)
    {
        const auto &oa = a_tracks[i]->getObject();
        const auto &ra = oa.rect;
        const float area_a = ra.width * ra.height;

        for (size_t j = 0; j < b_tracks.size(); ++j)
        {
            const auto &ob = b_tracks[j]->getObject();
            if (oa.label != ob.label)
            {
                continue;   // cost stays at 1.0 (max distance)
            }
            const auto &rb = ob.rect;

            const float left   = std::max(ra.x, rb.x);
            const float right  = std::min(ra.x + ra.width,  rb.x + rb.width);
            const float top    = std::max(ra.y, rb.y);
            const float bottom = std::min(ra.y + ra.height, rb.y + rb.height);
            if (right <= left || bottom <= top)
            {
                continue;
            }

            const float inter = (right - left) * (bottom - top);
            const float iou   = inter / (area_a + rb.width * rb.height - inter);
            cost[i][j] = 1.0f - iou;
        }
    }
    return cost;
}

// If a tracked strack (a) and a lost strack (b) overlap heavily (IoU > 0.85),
// drop the one with the shorter tracklet history.
void removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                            const std::vector<STrackPtr> &b_stracks,
                            std::vector<STrackPtr> &a_res,
                            std::vector<STrackPtr> &b_res)
{
    const auto dists = calcIouDistance(a_stracks, b_stracks);

    std::vector<bool> a_dup(a_stracks.size(), false);
    std::vector<bool> b_dup(b_stracks.size(), false);

    for (size_t i = 0; i < dists.size(); ++i)
    {
        for (size_t j = 0; j < dists[i].size(); ++j)
        {
            if (dists[i][j] >= 0.15f)
            {
                continue;
            }
            const size_t tp = a_stracks[i]->getFrameId() - a_stracks[i]->getStartFrameId();
            const size_t tq = b_stracks[j]->getFrameId() - b_stracks[j]->getStartFrameId();
            (tp > tq ? b_dup[j] : a_dup[i]) = true;
        }
    }

    a_res.reserve(a_stracks.size());
    for (size_t i = 0; i < a_stracks.size(); ++i)
    {
        if (!a_dup[i]) a_res.push_back(a_stracks[i]);
    }

    b_res.reserve(b_stracks.size());
    for (size_t j = 0; j < b_stracks.size(); ++j)
    {
        if (!b_dup[j]) b_res.push_back(b_stracks[j]);
    }
}
/////////////////////////////////////////////////////////////////////////////////
//// linear assignment
/////////////////////////////////////////////////////////////////////////////////

double execLapjv(const std::vector<std::vector<float>> &cost,
                                          std::vector<int> &rowsol,
                                          std::vector<int> &colsol,
                                          bool extend_cost,
                                          float cost_limit,
                                          bool return_cost = true)
{
    std::vector<std::vector<float> > cost_c;
    cost_c.assign(cost.begin(), cost.end());

    std::vector<std::vector<float> > cost_c_extended;

    int n_rows = cost.size();
    int n_cols = cost[0].size();
    rowsol.resize(n_rows);
    colsol.resize(n_cols);

    int n = 0;
    if (n_rows == n_cols)
    {
        n = n_rows;
    }
    else
    {
        if (!extend_cost)
        {
            throw std::runtime_error("The `extend_cost` variable should set True");
        }
    }

    if (extend_cost || cost_limit < std::numeric_limits<float>::max())
    {
        n = n_rows + n_cols;
        cost_c_extended.resize(n);
        for (size_t i = 0; i < cost_c_extended.size(); i++)
            cost_c_extended[i].resize(n);

        if (cost_limit < std::numeric_limits<float>::max())
        {
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_limit / 2.0;
                }
            }
        }
        else
        {
            float cost_max = -1;
            for (size_t i = 0; i < cost_c.size(); i++)
            {
                for (size_t j = 0; j < cost_c[i].size(); j++)
                {
                    if (cost_c[i][j] > cost_max)
                        cost_max = cost_c[i][j];
                }
            }
            for (size_t i = 0; i < cost_c_extended.size(); i++)
            {
                for (size_t j = 0; j < cost_c_extended[i].size(); j++)
                {
                    cost_c_extended[i][j] = cost_max + 1;
                }
            }
        }

        for (size_t i = n_rows; i < cost_c_extended.size(); i++)
        {
            for (size_t j = n_cols; j < cost_c_extended[i].size(); j++)
            {
                cost_c_extended[i][j] = 0;
            }
        }
        for (int i = 0; i < n_rows; i++)
        {
            for (int j = 0; j < n_cols; j++)
            {
                cost_c_extended[i][j] = cost_c[i][j];
            }
        }

        cost_c.clear();
        cost_c.assign(cost_c_extended.begin(), cost_c_extended.end());
    }

    double **cost_ptr;
    cost_ptr = new double *[sizeof(double *) * n];
    for (int i = 0; i < n; i++)
        cost_ptr[i] = new double[sizeof(double) * n];

    for (int i = 0; i < n; i++)
    {
        for (int j = 0; j < n; j++)
        {
            cost_ptr[i][j] = cost_c[i][j];
        }
    }

    int* x_c = new int[sizeof(int) * n];
    int *y_c = new int[sizeof(int) * n];

    int ret = lapjv_internal(n, cost_ptr, x_c, y_c);

    // Extract the solution (and optional cost) before releasing the buffers,
    // so the error path below can throw without leaking them.
    double opt = 0.0;
    if (ret == 0)
    {
        if (n != n_rows)
        {
            for (int i = 0; i < n; i++)
            {
                if (x_c[i] >= n_cols)
                    x_c[i] = -1;
                if (y_c[i] >= n_rows)
                    y_c[i] = -1;
            }
            for (int i = 0; i < n_rows; i++)
            {
                rowsol[i] = x_c[i];
            }
            for (int i = 0; i < n_cols; i++)
            {
                colsol[i] = y_c[i];
            }

            if (return_cost)
            {
                for (size_t i = 0; i < rowsol.size(); i++)
                {
                    if (rowsol[i] != -1)
                    {
                        opt += cost_ptr[i][rowsol[i]];
                    }
                }
            }
        }
        else if (return_cost)
        {
            for (size_t i = 0; i < rowsol.size(); i++)
            {
                opt += cost_ptr[i][rowsol[i]];
            }
        }
    }

    for (int i = 0; i < n; i++)
    {
        delete[]cost_ptr[i];
    }
    delete[]cost_ptr;
    delete[]x_c;
    delete[]y_c;

    if (ret != 0)
    {
        throw std::runtime_error("The result of lapjv_internal() is invalid.");
    }

    return opt;
}

void linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                                               const int &cost_matrix_size,
                                               const int &cost_matrix_size_size,
                                               const float &thresh,
                                               std::vector<std::vector<int>> &matches,
                                               std::vector<int> &a_unmatched,
                                               std::vector<int> &b_unmatched)
{
    if (cost_matrix.size() == 0)
    {
        a_unmatched.reserve(cost_matrix_size);
        for (int i = 0; i < cost_matrix_size; i++)
        {
            a_unmatched.push_back(i);
        }
        b_unmatched.reserve(cost_matrix_size_size);
        for (int i = 0; i < cost_matrix_size_size; i++)
        {
            b_unmatched.push_back(i);
        }
        return;
    }

    std::vector<int> rowsol; std::vector<int> colsol;
    execLapjv(cost_matrix, rowsol, colsol, true, thresh);

    matches.reserve(std::min(rowsol.size(), colsol.size()));
    a_unmatched.reserve(rowsol.size());
    b_unmatched.reserve(colsol.size());

    for (size_t i = 0; i < rowsol.size(); i++)
    {
        if (rowsol[i] >= 0)
        {
            matches.push_back({static_cast<int>(i), rowsol[i]});
        }
        else
        {
            a_unmatched.push_back(i);
        }
    }

    for (size_t i = 0; i < colsol.size(); i++)
    {
        if (colsol[i] < 0)
        {
            b_unmatched.push_back(i);
        }
    }
}

ByteTrackerImpl::ByteTrackerImpl(const int& max_age,
                                     const float& track_thresh,
                                     const float& high_thresh,
                                     const float& match_thresh) :
    track_thresh_(track_thresh),
    high_thresh_(high_thresh),
    match_thresh_(match_thresh),
    max_time_lost_(max_age),
    frame_id_(0),
    track_id_count_(0)
{
}

ByteTrackerImpl::~ByteTrackerImpl()
{
}

std::vector<STrackPtr> ByteTrackerImpl::update(const std::vector<Object>& objects)
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
        if (object.prob >= track_thresh_)
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
        linearAssignment(dists, strack_pool.size(), det_stracks.size(), match_thresh_,
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
            if (track->getObject().prob < high_thresh_)
            {
                continue;
            }
            track_id_count_++;
            track->activate(frame_id_, track_id_count_);
            current_tracked_stracks.push_back(track);
        }
    }

    ////////////////// Step 5: Update state //////////////////
    for (const auto &lost_strack : lost_stracks_)
    {
        if (frame_id_ - lost_strack->getFrameId() > max_time_lost_)
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
        // if (track->isActivated())               // Only output activated tracks
        {
            output_stracks.push_back(track);
        }
    }

    return output_stracks;
}

ByteTracker::ByteTracker(const unsigned int& max_age, const float& track_thresh,
                const float& high_thresh, const float& match_thresh)
{
    tracker_impl_ = std::make_shared<ByteTrackerImpl>(max_age, track_thresh, high_thresh, match_thresh);
}


std::vector<Track> ByteTracker::update(const std::vector<Object>& objects)
{
    std::vector<STrackPtr> stracks = tracker_impl_->update(objects);
    std::vector<Track> tracks;
    tracks.reserve(stracks.size());
    for (const auto& strack : stracks)
    {
        tracks.push_back(Track{strack->isActivated(),
                               strack->getTrackId(),
                               strack->getFrameId(),
                               strack->getObject()});
    }
    return tracks;
}
}
