#include "Motrack.h"
#include <iostream>
int main(int argc, char* argv[])
{
    motrack::Tracker tracker(motrack::TrackerType::ByteTrack);
    for (int i = 0; i < 20; i++) {
        std::vector<motrack::Object> objects;

        objects.push_back({0.9, 0, {100+i*10, 100, 50, 50}});
        objects.push_back({0.85, 1, {150, 150+i*10, 40, 40}});

        std::vector<motrack::Track> tracks = tracker.update(objects);
        for (const auto& track : tracks) {
            std::cout << "Track ID: " << track.track_id << " Score: " << track.object.prob << " Label: " << track.object.label << std::endl;
            std::cout << "Bounding Box: (" << track.object.rect.x << ", " << track.object.rect.y << ", "
                    << track.object.rect.width << ", " << track.object.rect.height << ")" << std::endl;
            std::cout << "--------------------------" << std::endl;
        }
    }
    return 0;
}
