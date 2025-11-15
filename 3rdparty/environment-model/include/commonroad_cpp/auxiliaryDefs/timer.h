#pragma once
#include <chrono>
/**
 * Taken from https://github.com/AutoPas/AutoPas/blob/ecbf799e8c1f828eb51e3507b76275afe98a8cb2/src/autopas/utils/Timer.h
 * RV-Monitor contains a similar class.
 */
class Timer final {
  public:
    Timer();

    ~Timer();

    /**
     * Start the timer.
     * @return Start time as high resolution time point.
     */
    static std::chrono::high_resolution_clock::time_point start();

    /**
     * Stops the timer and returns the time elapsed in nanoseconds since the last call to start.
     * It also adds the duration to the total time.
     *
     * @param startTime time as high resolution time point.
     * @return elapsed time in nanoseconds
     */
    long stop(std::chrono::high_resolution_clock::time_point startTime);

    /**
     * Adds the given amount of nanoseconds to the total time.
     * @param nanoseconds
     */
    void addTime(long nanoseconds);

    /**
     * Get total accumulated time.
     * @return Total time in nanoseconds.
     */
    [[nodiscard]] long getTotalTime() const { return totalTime; }

    /**
     * Resets timer.
     */
    void reset();

  private:
    /**
     * Accumulated total time.
     */
    long totalTime = 0;
};
