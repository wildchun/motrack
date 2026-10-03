#pragma once
#include "STrack.h"
#include <vector>

namespace motrack
{

// Appearance metrics shared by DeepSort / JDE style trackers.
struct FeatureMetric
{
    // Cosine distance matrix (1 - cos_sim) between each track's gallery
    // (best = min over stored embeddings) and each detection feature.
    // Entries are 1.0 when either side has no feature.
    static std::vector<std::vector<float>> cosineDistance(
        const std::vector<STrackPtr>& tracks,
        const std::vector<STrackPtr>& dets);

    // Fuse motion (IoU) and appearance costs:
    // cost = lambda * iou_cost + (1 - lambda) * appearance_cost.
    // Entries with appearance_cost > appearance_thresh are pushed to 1.0
    // (gated out) so Hungarian cannot pick them.
    static std::vector<std::vector<float>> fuseCost(
        const std::vector<std::vector<float>>& iou_cost,
        const std::vector<std::vector<float>>& appearance_cost,
        float lambda,
        float appearance_thresh);
};

}
