#pragma once
#include "STrack.h"
#include "Motrack.h"
#include <vector>

namespace motrack
{

using STrackPtr = std::shared_ptr<STrack>;

// Abstract per-algorithm tracker. Implementations live in
// src/algos/<family>/<algo>/ and only need to override update().
class BaseTracker
{
public:
    explicit BaseTracker(const TrackerConfig& config);
    virtual ~BaseTracker();

    // Feed one frame of detections, get the currently tracked stracks back.
    virtual std::vector<STrackPtr> update(const std::vector<Object>& objects) = 0;

    // Shared id counter, so all algorithms hand out global-unique track ids.
    unsigned long long nextTrackId();

protected:
    const TrackerConfig config_;

    unsigned long long frame_id_ = 0;

private:
    unsigned long long track_id_count_ = 0;
};

}
