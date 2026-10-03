#pragma once
#include "core/BaseTracker.h"

namespace motrack
{

// Canonical ByteTrack: high-score pass -> low-score pass -> unconfirmed pass,
// with max_age pruning and duplicate removal. Logic migrated from the original
// ByteTrackerImpl.
class ByteTrackTracker : public BaseTracker
{
public:
    using BaseTracker::BaseTracker;

    std::vector<STrackPtr> update(const std::vector<Object>& objects) override;

private:
    std::vector<STrackPtr> tracked_stracks_;
    std::vector<STrackPtr> lost_stracks_;
};

}
