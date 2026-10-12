#include "utilities/logger.hpp"
#include "utilities/sd_lock.hpp"
#include <cerrno>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>
#include <cstring>
#include "api.h"

// Initialize static member
Logger *Logger::instance = nullptr;

namespace
{
    // Boot logs live here when the folder exists on the card (the brain cannot
    // create folders, so tools/sdcard in the planner repo / the user makes it).
    // Two spellings: pros::usd::list_files wants paths relative to the card
    // root with no /usd prefix, fopen wants the full /usd path.
    constexpr const char *LOG_DIR_LIST = "/logs";
    constexpr const char *LOG_DIR_OPEN = "/usd/logs";
    constexpr const char *BOOT_PREFIX = "boot_";

    // Highest boot_NNNN.txt number in a card directory, or 0 when there is
    // none. Sets *exists to false only when the directory itself is missing.
    // A truncated listing (buffer full) is not fatal: the probe loop in
    // generateLogFilename() never reuses a name that is actually on the card.
    int highestBootNumber(const char *dirForList, bool *exists)
    {
        static char buffer[8192]; // ~400 names; static keeps it off the task stack
        buffer[0] = '\0';
        errno = 0;
        int32_t rc;
        {
            SdLock lock;
            rc = pros::usd::list_files(dirForList, buffer, sizeof(buffer));
        }
        *exists = !(rc == PROS_ERR && errno == ENOENT);
        buffer[sizeof(buffer) - 1] = '\0';

        int highest = 0;
        const size_t prefixLen = strlen(BOOT_PREFIX);
        for (char *name = strtok(buffer, "\n"); name != nullptr; name = strtok(nullptr, "\n"))
        {
            if (strncmp(name, BOOT_PREFIX, prefixLen) != 0)
                continue;
            char *end = nullptr;
            long n = strtol(name + prefixLen, &end, 10);
            if (end != name + prefixLen && strcmp(end, ".txt") == 0 && n > highest)
                highest = (int)n;
        }
        return highest;
    }

    bool fileExists(const std::string &path)
    {
        SdLock lock;
        FILE *f = fopen(path.c_str(), "r");
        if (f == nullptr)
            return false;
        fclose(f);
        return true;
    }
} // namespace

Logger::LogMessage::LogMessage(const std::string &level, const std::string &msg, uint32_t time)
    : logLevel(level), message(msg), timestamp(time) {}

Logger::Logger() : loggerTask(nullptr), isRunning(false)
{
    isSDCardMounted = false;
    generateLogFilename();
    initializeSDCard();
    startLoggingTask();
}

void Logger::generateLogFilename()
{
    // The V5 has no battery-backed clock: localtime() reads 1970 on every boot,
    // so the old timestamp name made every boot append to the same file and the
    // log had to be split by hand. Number the boots instead. One file per boot,
    // /usd/logs/boot_0007.txt, continuing from the highest number already on
    // the card; tests/import_logs.py pulls them off by number.
    if (!pros::usd::is_installed())
    {
        logFilename = std::string(LOG_DIR_OPEN) + "/boot_0000.txt"; // never opened
        return;
    }

    bool haveLogsDir = false;
    int next = highestBootNumber(LOG_DIR_LIST, &haveLogsDir) + 1;
    std::string dir = LOG_DIR_OPEN;
    if (!haveLogsDir)
    {
        // No logs/ folder on this card: fall back to the root rather than
        // losing the log. (Create <card>/logs to keep the root tidy.)
        bool rootExists = true;
        next = highestBootNumber("/", &rootExists) + 1;
        dir = "/usd";
    }

    // Belt and braces: the listing can be truncated on a very full folder, so
    // never take a name that already exists.
    char name[64];
    for (int probe = 0; probe < 1000; ++probe, ++next)
    {
        snprintf(name, sizeof(name), "%s/%s%04d.txt", dir.c_str(), BOOT_PREFIX, next);
        if (!fileExists(name))
        {
            logFilename = name;
            return;
        }
    }
    logFilename = dir + "/boot_overflow.txt";
}

void Logger::initializeSDCard()
{
    if (pros::usd::is_installed())
    {
        isSDCardMounted = true;
        {
            SdLock lock;
            logFile = fopen(logFilename.c_str(), "a");
        }
        if (logFile != nullptr)
        {
            writeToFile("INFO", "Logger initialized successfully");
            writeToFile("INFO", "Log file: " + logFilename);
        }
        else
        {
            printf("Failed to open log file: %s\n", logFilename.c_str());
        }
    }
    else
    {
        printf("SD card not installed\n");
    }
}

void Logger::startLoggingTask()
{
    isRunning = true;
    loggerTask = new pros::Task([this]
                                { processLogQueue(); });
}

void Logger::writeMessage(const LogMessage &msg)
{
    if (logFile == nullptr)
        return;
    uint32_t millis = msg.timestamp;
    uint32_t seconds = millis / 1000;
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;
    // Held across the write and its flush: this task is the one that shares the
    // card with the route selector's listings and route reads (see SdLock).
    SdLock lock;
    fprintf(logFile, "[%02d:%02d:%02d.%03d] [%s] %s\n",
            (int)hours % 24,
            (int)minutes % 60,
            (int)seconds % 60,
            (int)millis % 1000,
            msg.logLevel.c_str(),
            msg.message.c_str());
}

void Logger::flushFile()
{
    if (logFile == nullptr)
        return;
    SdLock lock;
    fflush(logFile);
}

void Logger::processLogQueue()
{
    while (isRunning)
    {
        // Drain everything queued so far this wake, not just one message. The old
        // one-per-iteration behavior let the queue build a long tail that was
        // lost if the program crashed/rebooted before the task caught up (which
        // is exactly what makes a boot-crash log look empty). Draining fully
        // keeps the on-disk file as close to real time as the SD writes allow.
        queueMutex.take();
        if (messageQueue.empty())
        {
            queueMutex.give();
            pros::delay(10); // Sleep to prevent CPU hogging
            continue;
        }
        // One fflush per batch rather than per line: a flush per line is one SD write
        // per line (~1400/s in auton), which the card could not keep up with, so a
        // backlog built all run and the last ~10 s were lost at power-off (boots
        // 0664-0670 all end 48.6 s in). Still flush at least every FLUSH_INTERVAL_MS
        // during a long drain so a crash loses little.
        uint32_t lastFlush = pros::millis();
        while (!messageQueue.empty())
        {
            LogMessage msg = messageQueue.front();
            messageQueue.pop();
            queueMutex.give();   // release around the (slow) SD write
            writeMessage(msg);
            if (pros::millis() - lastFlush >= FLUSH_INTERVAL_MS)
            {
                flushFile();
                lastFlush = pros::millis();
            }
            queueMutex.take();
        }
        queueMutex.give();
        flushFile();
    }
}

void Logger::flush()
{
    if (!isSDCardMounted)
        return;
    // Wait for the background task to drain the queue. Bounded so a stuck/absent
    // logger task can never hang the caller (e.g. initialize()).
    for (int i = 0; i < 500; ++i)
    {
        queueMutex.take();
        bool empty = messageQueue.empty();
        queueMutex.give();
        if (empty)
        {
            // Queue drained. The task may still be mid-write on the message it
            // just popped, so give one short beat for that fflush to land.
            pros::delay(2);
            return;
        }
        pros::delay(1);
    }
}

void Logger::writeToFile(const std::string &level, const std::string &message)
{
    queueMutex.take();
    messageQueue.emplace(level, message, pros::millis());
    queueMutex.give();
}

Logger *Logger::getInstance()
{
    if (instance == nullptr)
    {
        instance = new Logger();
    }
    return instance;
}

void Logger::log(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("LOG", std::string(buffer));
}

void Logger::logError(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("ERROR", std::string(buffer));
}

void Logger::logInfo(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("INFO", std::string(buffer));
}

void Logger::logDebug(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("DEBUG", std::string(buffer));
}

void Logger::logWarning(const char *format, ...)
{
    if (!isSDCardMounted)
        return;
    char buffer[512];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    writeToFile("WARNING", std::string(buffer));
}

const std::string &Logger::getLogFilename() const
{
    return logFilename;
}

void Logger::close()
{
    if (isRunning)
    {
        isRunning = false;
        if (loggerTask != nullptr)
        {
            loggerTask->remove();
            delete loggerTask;
            loggerTask = nullptr;
        }
    }
    if (logFile != nullptr)
    {
        fclose(logFile);
        logFile = nullptr;
    }
}

Logger::~Logger()
{
    close();
}