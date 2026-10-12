#include "utilities/sd_lock.hpp"

namespace
{
    /// One mutex for the whole card. Function-local so it is constructed on the
    /// first lock, which is the Logger constructor -- before any task but the
    /// initialize task exists, so the lazy construction itself cannot race.
    pros::Mutex &cardMutex()
    {
        static pros::Mutex mutex;
        return mutex;
    }
} // namespace

SdLock::SdLock() : held_(cardMutex().take(TIMEOUT_MS))
{
}

SdLock::~SdLock()
{
    if (held_)
    {
        cardMutex().give();
    }
}
