#include "FeatureMetric.h"

#include <cmath>

namespace motrack
{

namespace
{

float cosine(const std::vector<float>& a, const std::vector<float>& b)
{
    if (a.empty() || a.size() != b.size())
    {
        return 0.0f;
    }
    float dot = 0.0f, na = 0.0f, nb = 0.0f;
    for (size_t i = 0; i < a.size(); ++i)
    {
        dot += a[i] * b[i];
        na  += a[i] * a[i];
        nb  += b[i] * b[i];
    }
    if (na <= 0.0f || nb <= 0.0f)
    {
        return 0.0f;
    }
    return dot / (std::sqrt(na) * std::sqrt(nb));
}

} // namespace

std::vector<std::vector<float>> FeatureMetric::cosineDistance(
    const std::vector<STrackPtr>& tracks,
    const std::vector<STrackPtr>& dets)
{
    std::vector<std::vector<float>> cost(tracks.size(),
                                         std::vector<float>(dets.size(), 1.0f));
    for (size_t i = 0; i < tracks.size(); ++i)
    {
        const auto& gallery = tracks[i]->features();
        if (gallery.empty())
        {
            continue;
        }
        for (size_t j = 0; j < dets.size(); ++j)
        {
            const auto& feat = dets[j]->getObject().feature;
            if (feat.empty())
            {
                continue;
            }
            float best = 1.0f;  // max distance; lower is better
            for (const auto& g : gallery)
            {
                const float d = 1.0f - cosine(g, feat);
                if (d < best)
                {
                    best = d;
                }
            }
            cost[i][j] = best;
        }
    }
    return cost;
}

std::vector<std::vector<float>> FeatureMetric::fuseCost(
    const std::vector<std::vector<float>>& iou_cost,
    const std::vector<std::vector<float>>& appearance_cost,
    float lambda,
    float appearance_thresh)
{
    std::vector<std::vector<float>> cost = iou_cost;
    for (size_t i = 0; i < cost.size() && i < appearance_cost.size(); ++i)
    {
        for (size_t j = 0; j < cost[i].size() && j < appearance_cost[i].size(); ++j)
        {
            float c = lambda * iou_cost[i][j] + (1.0f - lambda) * appearance_cost[i][j];
            // Gate: reject pairs whose appearance is too dissimilar.
            if (appearance_cost[i][j] > appearance_thresh)
            {
                c = 1.0f;
            }
            cost[i][j] = c;
        }
    }
    return cost;
}

}
