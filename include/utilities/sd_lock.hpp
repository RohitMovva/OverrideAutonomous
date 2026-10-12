#ifndef SD_LOCK_HPP
#define SD_LOCK_HPP

#include "pros/rtos.hpp"
#include <cstdint>

/**
 * @brief Serialises every access to the microSD card.
 *
 * The card is reached from more than one task: the logger task fflushes a line
 * after every message, while the initialize task (route selector, trajectory
 * loader) lists directories and reads route files. Two tasks inside the FAT
 * layer at the same time is what boots 0141/0143/0144 (09-19-2026) look like --
 * initialize() stopped dead right after the card listing, the brain never got a
 * page up, and boot 0142 came up but silently lost the log line that was being
 * written in that same window.
 *
 * Every leaf-level card operation takes this lock for as long as it is inside
 * the FAT layer and gives it straight back. Leaf-level is the rule that keeps it
 * deadlock-free: a function that takes the lock must not call another function
 * that takes it (pros::Mutex is not recursive). Composite work -- listCard()
 * walking three folders, loadSelected() reading a route -- locks each underlying
 * read, not the whole sequence.
 *
 * Logging from inside a locked region is safe: Logger::log only queues, and the
 * logger task blocks on this lock like anyone else. Logger::flush() is NOT --
 * it waits for that task, which cannot write until the lock is free.
 *
 * Usage:
 * @code
 * {
 *     SdLock lock;
 *     FILE *f = fopen("/usd/routes/skills.txt", "r");
 *     ...
 * } // released here
 * @endcode
 */
class SdLock
{
public:
    /// Longest a caller waits for the card. A timeout means something else is
    /// stuck in the FAT layer; the caller goes ahead unguarded rather than
    /// hanging the task, which is the failure this class exists to avoid.
    static constexpr std::uint32_t TIMEOUT_MS = 2000;

    SdLock();
    ~SdLock();

    SdLock(const SdLock &) = delete;
    SdLock &operator=(const SdLock &) = delete;

    /// False when the lock timed out and the operation is running unguarded
    bool held() const { return held_; }

private:
    bool held_;
};

#endif // SD_LOCK_HPP
