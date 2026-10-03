#include "BaseTracker.h"

namespace motrack
{

BaseTracker::BaseTracker(const TrackerConfig& config) : config_(config)
{
}

BaseTracker::~BaseTracker()
{
}

unsigned long long BaseTracker::nextTrackId()
{
    return ++track_id_count_;
}

}
