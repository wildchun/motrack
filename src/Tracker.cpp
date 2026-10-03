#include "Motrack.h"
#include "core/BaseTracker.h"
#include "algos/motion/bytetrack/ByteTrackTracker.h"
#include "algos/motion/sort/SortTracker.h"
#include "algos/motion/ocsort/OCSortTracker.h"
#include "algos/appearance/deepsort/DeepSortTracker.h"
#include "algos/appearance/jde/JDETracker.h"

namespace motrack
{

namespace
{
std::shared_ptr<BaseTracker> makeImpl(TrackerType type, const TrackerConfig& config)
{
    switch (type)
    {
    case TrackerType::ByteTrack:
        return std::make_shared<ByteTrackTracker>(config);
    case TrackerType::Sort:
        return std::make_shared<SortTracker>(config);
    case TrackerType::OCSort:
        return std::make_shared<OCSortTracker>(config);
    case TrackerType::DeepSort:
        return std::make_shared<DeepSortTracker>(config);
    case TrackerType::JDE:
        return std::make_shared<JDETracker>(config);
    }
    return nullptr;
}
} // namespace

Tracker::Tracker(TrackerType type, const TrackerConfig& config)
: type_(type), impl_(makeImpl(type, config))
{
    if (!impl_)
    {
        impl_ = std::make_shared<ByteTrackTracker>(config);
        type_ = TrackerType::ByteTrack;
    }
}

Tracker::~Tracker() = default;

std::vector<Track> Tracker::update(const std::vector<Object>& objects)
{
    std::vector<STrackPtr> stracks = impl_->update(objects);
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

std::shared_ptr<Tracker> createTracker(TrackerType type, const TrackerConfig& config)
{
    return std::make_shared<Tracker>(type, config);
}

}
