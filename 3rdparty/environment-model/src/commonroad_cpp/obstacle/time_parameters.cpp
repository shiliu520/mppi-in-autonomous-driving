
#include "commonroad_cpp/obstacle/time_parameters.h"
#include <cassert>
#include <cstddef>

TimeParameters::TimeParameters(size_t relevantHistorySize, double reactionTime, double timeStepSize)
    : relevantHistorySize{relevantHistorySize}, reactionTime{reactionTime}, timeStepSize(timeStepSize) {
    assert(reactionTime >= 0.0);
    assert(timeStepSize > 0.0);
}

void TimeParameters::setTimeStepSize(double timeStepSize) { this->timeStepSize = timeStepSize; }

size_t TimeParameters::getRelevantHistorySize() const noexcept { return relevantHistorySize; }

double TimeParameters::getReactionTime() const noexcept { return reactionTime; }

double TimeParameters::getTimeStepSize() const noexcept { return timeStepSize; }

TimeParameters TimeParameters::dynamicDefaults() { return TimeParameters{50, 0.3, 0.1}; }

TimeParameters TimeParameters::staticDefaults() { return TimeParameters{0, 0.0, 0.1}; }
