/**
 * @file Measurement.hpp
 * @brief Measurement data management and calculations.
 *
 * Handles:
 * - Storage of measurement data
 * - Time calculations
 * - Result formatting
 */

#ifndef MEASUREMENT_HPP
#define MEASUREMENT_HPP

#include <Arduino.h>
#include "main.hpp"

/**
 * @class MeasurementManager
 * @brief Manages competition run measurements.
 */
class MeasurementManager {
public:
    /**
     * @brief Get reference to current measurement data.
     * @return Reference to Measurement struct
     */
    static Measurement& getCurrentMeasurement();

    /**
     * @brief Reset all measurement data for next run.
     */
    static void reset();

    /**
     * @brief Calculate final time from raw timestamps.
     * Converts microsecond difference to MM:SS:mmm format.
     * @param startTime Start timestamp in microseconds
     * @param finishTime Finish timestamp in microseconds
     * @return Formatted time string (MM:SS:mmm)
     */
    static String calculateFinalTime(uint64_t startTime, uint64_t finishTime);

    /**
     * @brief Calculate milliseconds from two timestamps.
     * @param startTime Start timestamp in microseconds
     * @param finishTime Finish timestamp in microseconds
     * @return Duration in milliseconds
     */
    static uint64_t calculateDuration(uint64_t startTime, uint64_t finishTime);

    /**
     * @brief Format raw milliseconds to MM:SS:mmm string.
     * @param milliseconds Duration in milliseconds
     * @return Formatted time string
     */
    static String formatTimeString(uint64_t milliseconds);

    /**
     * @brief Get minutes component from milliseconds.
     * @param milliseconds Duration in milliseconds
     * @return Minutes (0-59)
     */
    static uint32_t getMinutes(uint64_t milliseconds);

    /**
     * @brief Get seconds component from milliseconds.
     * @param milliseconds Duration in milliseconds
     * @return Seconds (0-59)
     */
    static uint32_t getSeconds(uint64_t milliseconds);

    /**
     * @brief Get milliseconds component.
     * @param milliseconds Duration in milliseconds
     * @return Milliseconds (0-999)
     */
    static uint32_t getMilliseconds(uint64_t milliseconds);

private:
    // Private constructor (static methods only)
    MeasurementManager() = delete;

    static Measurement measurement;
};

#endif  // MEASUREMENT_HPP
