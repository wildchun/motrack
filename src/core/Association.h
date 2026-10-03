#pragma once
#include "STrack.h"
#include <vector>

namespace motrack
{
using STrackPtr = std::shared_ptr<STrack>;

// Set/matrix helpers shared by all tracking algorithms.

// Union by track_id, preserving `a`'s order followed by new tracks from `b`.
std::vector<STrackPtr> jointStracks(const std::vector<STrackPtr> &a_tlist,
                                    const std::vector<STrackPtr> &b_tlist);

// Set difference by track_id, preserving `a`'s order.
std::vector<STrackPtr> subStracks(const std::vector<STrackPtr> &a_tlist,
                                  const std::vector<STrackPtr> &b_tlist);

// Compute IoU distance (1 - IoU) directly from STrack pairs.
// Returns an empty matrix when either side is empty.
std::vector<std::vector<float>> calcIouDistance(const std::vector<STrackPtr> &a_tracks,
                                                const std::vector<STrackPtr> &b_tracks);

// If a tracked strack (a) and a lost strack (b) overlap heavily (IoU > 0.85),
// drop the one with the shorter tracklet history.
void removeDuplicateStracks(const std::vector<STrackPtr> &a_stracks,
                            const std::vector<STrackPtr> &b_stracks,
                            std::vector<STrackPtr> &a_res,
                            std::vector<STrackPtr> &b_res);

// Hungarian (lapjv) assignment over a cost matrix.
// Matches whose cost >= thresh are rejected and reported as unmatched.
void linearAssignment(const std::vector<std::vector<float>> &cost_matrix,
                      const int &cost_matrix_size,
                      const int &cost_matrix_size_size,
                      const float &thresh,
                      std::vector<std::vector<int>> &matches,
                      std::vector<int> &a_unmatched,
                      std::vector<int> &b_unmatched);
}
