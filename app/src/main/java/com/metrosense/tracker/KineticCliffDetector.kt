package com.metrosense.tracker

import kotlin.math.sqrt

/**
 * Dual-Window Kinetic Cliff Detector for metro station arrival detection.
 *
 * ## Algorithm Overview
 *
 * The detector works on **1Hz downsampled** data (called once per second via [feed]).
 * It maintains a fixed-size circular ring buffer of `accel_std` values and makes
 * decisions with a **10-second look-ahead delay**.
 *
 * ### Timeline (decision at time T = dp)
 * ```
 * Second:  0 ... 9 | 10 | 11 ... 20
 *          ^^^^^^^^   ^   ^^^^^^^^
 *           Past      dp   Future
 *          Drive          Quiet
 *          (10s)          (10s)
 *
 * We are at current time = 20.  decision-point dp = 10.
 * We have 10 seconds of look-ahead (11..20) to confirm the stop.
 * ```
 *
 * The decision for a station stop at time `dp` is made **10 seconds later**
 * at time `dp + QUIET_WINDOW`, once the future-quiet window is fully populated.
 * This means station arrivals are reported with a 10-second latency, which is
 * imperceptible in a metro context (trains dwell for 20-60s at stations).
 */
class KineticCliffDetector(
    private val onStationDetected: (stationNumber: Int) -> Unit
) {
    companion object {
        // ── Window sizes (seconds at 1Hz) ──
        private const val DRIVE_WINDOW = 10
        private const val QUIET_WINDOW = 10
        private const val ENV_WINDOW = 60

        // ── Thresholds ──
        private const val HAND_SHAKE_THRESHOLD_DEG = 25.0f
        private const val BASELINE_QUIET = 0.05f
        private const val DRIVE_AVG_MIN = 0.08f
        private const val ADAPTIVE_MULTIPLIER = 0.50f
        private const val ADAPTIVE_CLAMP_MIN = 0.05f
        private const val ADAPTIVE_CLAMP_MAX = 0.18f

        // ── Cooldown ──
        private const val MIN_STATION_INTERVAL_SEC = 70L

        // ── Ring-buffer capacity (must hold ENV_WINDOW + DRIVE_WINDOW + QUIET_WINDOW + margin) ──
        private const val CAPACITY = 120
    }

    // ──────────────── Ring Buffer (pre-allocated, fixed-size, zero GC in hot path) ────────────────

    private val buffer = FloatArray(CAPACITY)
    private var head = 0           // logical index 0 is the oldest sample
    private var count = 0          // total samples stored (capped at CAPACITY)

    // Pre-allocated temp array for median computation (avoids allocation per evaluation)
    private val medianBuffer = FloatArray(QUIET_WINDOW)

    // ──────────────── State ────────────────

    private var lastStationDecisionSecond = -MIN_STATION_INTERVAL_SEC

    /** Total stations detected since last [reset]. */
    var stationCount = 0
        private set

    // ──────────────── Public API ────────────────

    /**
     * Feed one second of aggregated sensor statistics.
     *
     * This is expected to be called **once per second** by the caller
     * (the [MetroTrackingService] handles raw sensor accumulation
     * and 1 Hz downsampling).
     *
     * @param accelStd   standard deviation of linear-acceleration magnitude
     *                   over the past 1-second window.
     * @param gyroStd    standard deviation of gyroscope angular-velocity
     *                   magnitude (in deg/s) over the same window.
     */
    fun feed(accelStd: Float, gyroStd: Float) {
        // ── 1. Hand-shake rejection ──
        // If the gyro shows significant rotation (>25°/s std), the phone is
        // being handled / the user is shifting posture.  Clamp accel_std to
        // the baseline quiet value to prevent false-positive motion triggers.
        val clampedAccelStd = if (gyroStd > HAND_SHAKE_THRESHOLD_DEG) {
            BASELINE_QUIET
        } else {
            accelStd
        }

        // ── 2. Store into ring buffer ──
        val writeIndex = (head + count) % CAPACITY
        buffer[writeIndex] = clampedAccelStd

        if (count < CAPACITY) {
            count++
        } else {
            // Buffer is full – advance head to drop the oldest sample
            head = (head + 1) % CAPACITY
        }

        // ── 3. Evaluate dual-window decision ──
        evaluate()
    }

    /** Reset all internal state (e.g. when starting a new trip). */
    fun reset() {
        head = 0
        count = 0
        lastStationDecisionSecond = -MIN_STATION_INTERVAL_SEC
        stationCount = 0
    }

    // ──────────────── Ring-buffer helpers ────────────────

    /** Retrieve the sample at logical index [i] (0 = oldest). */
    private fun get(i: Int): Float {
        return buffer[(head + i) % CAPACITY]
    }

    // ──────────────── Adaptive Threshold ────────────────

    /**
     * 60-second rolling average of environmental energy, scaled by 0.50
     * and clamped to [ADAPTIVE_CLAMP_MIN, ADAPTIVE_CLAMP_MAX].
     *
     * Uses the **newest** [ENV_WINDOW] samples in the buffer.
     */
    private fun computeAdaptiveThreshold(): Float {
        val windowSize = minOf(count, ENV_WINDOW)
        if (windowSize == 0) return ADAPTIVE_CLAMP_MIN

        var sum = 0.0
        // Iterate from newest (count-1) backward for `windowSize` samples
        for (i in 0 until windowSize) {
            sum += get(count - 1 - i)
        }
        val mean = sum / windowSize
        return (mean * ADAPTIVE_MULTIPLIER).toFloat()
            .coerceIn(ADAPTIVE_CLAMP_MIN, ADAPTIVE_CLAMP_MAX)
    }

    // ──────────────── Dual-Window Decision ────────────────

    private fun evaluate() {
        // Need at least DRIVE + QUIET + 1 samples for a meaningful evaluation
        if (count < DRIVE_WINDOW + QUIET_WINDOW + 1) return

        // ── Decision point ──
        // The "decision point" (dp) is QUIET_WINDOW seconds *before* the
        // current time (count-1).  We now have enough look-ahead data to
        // decide whether the train **actually stopped** at time dp.
        //
        //   [Past Drive 10s] [dp] [Future Quiet 10s]
        //   ^^^^^^^^^^^^^^^^   ^   ^^^^^^^^^^^^^^^^
        //   get(dp-10..dp-1)   |   get(dp+1..dp+10)
        //                       dp
        //
        // The call to evaluate() happens *after* storing the value for
        // the current second, so:
        //   current time = count - 1
        //   dp = count - 1 - QUIET_WINDOW
        val dp = count - 1 - QUIET_WINDOW

        // ── 1. Past Drive Window ──
        // Average accel_std over the 10 seconds BEFORE the decision point.
        // Must exceed DRIVE_AVG_MIN, confirming the train was moving.
        var pastSum = 0.0
        for (i in 0 until DRIVE_WINDOW) {
            pastSum += get(dp - DRIVE_WINDOW + i)
        }
        if ((pastSum / DRIVE_WINDOW) <= DRIVE_AVG_MIN) return

        // ── 2. Future Quiet Window ──
        // Median accel_std over the 10 seconds AFTER the decision point.
        // Must fall BELOW the adaptive threshold, confirming the train
        // has entered a motionless dead-zone (station stop).
        for (i in 0 until QUIET_WINDOW) {
            medianBuffer[i] = get(dp + 1 + i)
        }
        medianBuffer.sort()
        val futureMedian = medianBuffer[QUIET_WINDOW / 2]

        val threshold = computeAdaptiveThreshold()
        if (futureMedian >= threshold) return

        // ── 3. Cooldown Guard ──
        // Prevent re-triggering on the same station stop.
        if (dp.toLong() - lastStationDecisionSecond < MIN_STATION_INTERVAL_SEC) return

        // ── Trigger ──
        lastStationDecisionSecond = dp.toLong()
        stationCount++
        onStationDetected(stationCount)
    }
}
