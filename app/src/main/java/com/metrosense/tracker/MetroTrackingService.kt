package com.metrosense.tracker

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import android.os.Handler
import android.os.HandlerThread
import android.os.IBinder
import android.os.PowerManager
import android.os.VibrationEffect
import android.os.Vibrator
import android.os.VibratorManager
import android.util.Log
import androidx.core.app.NotificationCompat
import kotlin.math.sqrt

/**
 * Android Foreground Service that tracks metro trips using inertial sensor fusion.
 *
 * ## Lifecycle
 * ```
 * onCreate()  →  init sensors, handler thread, wakelock, detector
 * onStartCommand()  →  register sensor listeners, startForeground()
 * onDestroy()  →  unregister listeners, quit thread, release wakelock
 * ```
 *
 * ## Sensor Processing Pipeline
 * ```
 * Raw sensors (50 Hz)  →  1 s batch accumulator  →  std computation
 *       ↓
 * KineticCliffDetector.feed(accelStd, gyroStd)
 *       ↓
 * Dual-window decision (10 s look-ahead)  →  onStationDetected callback
 *       ↓
 * Notification update + vibration
 * ```
 *
 * ## Manifest Requirements (Android 14+)
 * ```xml
 * <uses-permission android:name="android.permission.FOREGROUND_SERVICE" />
 * <uses-permission android:name="android.permission.FOREGROUND_SERVICE_SPECIAL_USE" />
 * <uses-permission android:name="android.permission.POST_NOTIFICATIONS" />
 * <uses-permission android:name="android.permission.VIBRATE" />
 * <uses-permission android:name="android.permission.WAKE_LOCK" />
 *
 * <service
 *     android:name=".MetroTrackingService"
 *     android:foregroundServiceType="specialUse"
 *     android:exported="false" />
 * ```
 */
class MetroTrackingService : Service() {

    // ──────────────── Constants ────────────────

    companion object {
        private const val TAG = "MetroTracking"
        private const val NOTIFICATION_ID = 1001
        private const val CHANNEL_ID = "metro_tracking_channel"
        private const val CHANNEL_NAME = "Metro Trip Detection"
        private const val WAKELOCK_TIMEOUT_MS = 8 * 60 * 60 * 1000L // 8 h safety timeout

        /** Maximum samples to buffer per 1-second window (safe for 100 Hz). */
        private const val MAX_BATCH_SIZE = 200

        /** Convenience entry point. */
        fun start(context: Context) {
            val intent = Intent(context, MetroTrackingService::class.java)
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                context.startForegroundService(intent)
            } else {
                @Suppress("DEPRECATION")
                context.startService(intent)
            }
        }

        /** Convenience shutdown. */
        fun stop(context: Context) {
            context.stopService(Intent(context, MetroTrackingService::class.java))
        }
    }

    // ──────────────── Services ────────────────

    private lateinit var notificationManager: NotificationManager
    private lateinit var sensorManager: SensorManager
    private var vibrator: Vibrator? = null

    // ──────────────── Sensors ────────────────

    private var accelerometer: Sensor? = null
    private var gyroscope: Sensor? = null

    // ──────────────── Processing ────────────────

    private var detector: KineticCliffDetector? = null

    // Dedicated background thread for sensor processing (avoids Main Thread work)
    private var sensorThread: HandlerThread? = null
    private var sensorHandler: Handler? = null

    // 1-second batch accumulators (pre-allocated, zero GC in hot path)
    private val accelBatch = FloatArray(MAX_BATCH_SIZE)
    private var accelBatchCount = 0
    private val gyroBatch = FloatArray(MAX_BATCH_SIZE)
    private var gyroBatchCount = 0
    private var batchStartNs = 0L

    // ──────────────── Power management ────────────────

    private var wakeLock: PowerManager.WakeLock? = null

    // ──────────────── Sensor event listener ────────────────

    /**
     * Single [SensorEventListener] handling both accelerometer and gyroscope.
     *
     * All sensor events arrive on a dedicated background thread (registered via
     * [SensorManager.registerListener] with a Handler from our [android.os.HandlerThread]),
     * keeping the Main Thread free.
     *
     * The listener performs **1 Hz downsampling**:
     * 1. Accumulate raw magnitudes into pre-allocated float arrays.
     * 2. When the elapsed time from [batchStartNs] crosses 1 second, finalise
     *    the batch by computing the population standard deviation for both
     *    accel and gyro samples.
     * 3. Feed the per-second stats to [KineticCliffDetector.feed].
     * 4. Reset accumulators and continue with the current event starting the
     *    next 1-second window.
     */
    private val sensorEventListener = object : SensorEventListener {

        override fun onSensorChanged(event: SensorEvent) {
            // Initialise batch start on first event
            if (batchStartNs == 0L) {
                batchStartNs = event.timestamp
            }

            // ── Boundary check BEFORE adding the current sample ──
            // If one full second has elapsed, finalise the current batch,
            // reset accumulators, and let the current event start the new window.
            if (event.timestamp - batchStartNs >= 1_000_000_000L) {
                val accelStd = computeStd(accelBatch, accelBatchCount)
                val gyroStd  = computeStd(gyroBatch, gyroBatchCount)
                detector?.feed(accelStd, gyroStd)

                batchStartNs = event.timestamp
                accelBatchCount = 0
                gyroBatchCount = 0
            }

            // ── Accumulate current sensor sample ──
            when (event.sensor.type) {
                Sensor.TYPE_LINEAR_ACCELERATION -> {
                    val mag = sqrt(
                        event.values[0] * event.values[0] +
                        event.values[1] * event.values[1] +
                        event.values[2] * event.values[2]
                    ) - 9.81f
                    accelBatch[accelBatchCount++] = mag
                }

                Sensor.TYPE_GYROSCOPE -> {
                    val mag = sqrt(
                        event.values[0] * event.values[0] +
                        event.values[1] * event.values[1] +
                        event.values[2] * event.values[2]
                    ) * 57.2958f   // rad/s → deg/s
                    gyroBatch[gyroBatchCount++] = mag
                }
            }
        }

        override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {
            // Not needed
        }
    }

    // ──────────────── Lifecycle ────────────────

    override fun onCreate() {
        super.onCreate()
        Log.i(TAG, "Creating MetroTrackingService")

        notificationManager = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        sensorManager = getSystemService(SENSOR_SERVICE) as SensorManager

        // Vibrator (API 31+ uses VibratorManager)
        vibrator = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            val vm = getSystemService(VIBRATOR_MANAGER_SERVICE) as VibratorManager
            vm.defaultVibrator
        } else {
            @Suppress("DEPRECATION")
            getSystemService(VIBRATOR_SERVICE) as Vibrator
        }

        // Sensors
        accelerometer = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
        gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE)
        if (accelerometer == null) Log.w(TAG, "Linear accelerometer not available")
        if (gyroscope == null) Log.w(TAG, "Gyroscope not available")

        // Dedicated handler thread – all sensor processing runs here
        sensorThread = HandlerThread("MetroSensorThread").apply { start() }
        sensorHandler = Handler(sensorThread!!.looper)

        // Wakelock – keeps CPU awake during sensor processing
        val pm = getSystemService(POWER_SERVICE) as PowerManager
        wakeLock = pm.newWakeLock(
            PowerManager.PARTIAL_WAKE_LOCK,
            "$packageName:metro_sensor_lock"
        )
        wakeLock?.acquire(WAKELOCK_TIMEOUT_MS)

        // Notification channel & foreground notification
        createNotificationChannel()
        startForeground(NOTIFICATION_ID, buildNotification(0))

        // Detector – station detection fires back on the sensor-thread
        // via the Handler (single-threaded), so notification updates
        // are naturally serialised with sensor processing.
        detector = KineticCliffDetector { stationNumber ->
            Log.i(TAG, "Station $stationNumber detected!")
            updateNotification(stationNumber)
            triggerHapticFeedback()
        }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        Log.i(TAG, "onStartCommand")

        // If critical sensors are missing, refuse to start
        if (accelerometer == null || gyroscope == null) {
            Log.e(TAG, "Cannot track – critical sensors missing")
            stopSelf()
            return START_NOT_STICKY
        }

        // Register listeners on the dedicated HandlerThread.
        // This keeps sensor processing off the Main Thread and serialises
        // all incoming events on a single background Looper.
        val handler = sensorHandler
        if (handler == null) {
            Log.e(TAG, "sensorHandler is null – cannot register")
            stopSelf()
            return START_NOT_STICKY
        }

        sensorManager.registerListener(
            sensorEventListener, accelerometer,
            SensorManager.SENSOR_DELAY_GAME, handler
        )
        sensorManager.registerListener(
            sensorEventListener, gyroscope,
            SensorManager.SENSOR_DELAY_GAME, handler
        )

        return START_STICKY
    }

    override fun onDestroy() {
        Log.i(TAG, "Destroying MetroTrackingService")

        // ── Critical: unregister listeners to prevent sensor leaks ──
        sensorManager.unregisterListener(sensorEventListener)

        // Quit the background thread
        sensorThread?.quitSafely()
        sensorThread = null
        sensorHandler = null

        // Release wakelock
        wakeLock?.let {
            if (it.isHeld) it.release()
        }
        wakeLock = null

        detector = null

        super.onDestroy()
    }

    override fun onBind(intent: Intent?): IBinder? = null

    // ──────────────── 1 Hz downsampling helper ────────────────

    /**
     * Population standard deviation of the values in [array][0..n-1].
     *
     * Returns `0f` when `n <= 1` (insufficient data for a meaningful std).
     * Uses `Double` for the accumulator to maintain precision over many samples.
     */
    private fun computeStd(values: FloatArray, n: Int): Float {
        if (n <= 1) return 0f
        var mean = 0.0
        for (i in 0 until n) mean += values[i]
        mean /= n
        var variance = 0.0
        for (i in 0 until n) {
            val diff = values[i] - mean
            variance += diff * diff
        }
        return sqrt(variance / n).toFloat()
    }

    // ──────────────── Notifications ────────────────

    private fun createNotificationChannel() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            val channel = NotificationChannel(
                CHANNEL_ID,
                CHANNEL_NAME,
                NotificationManager.IMPORTANCE_HIGH
            ).apply {
                description = "Live updates for metro station arrival detection"
                setShowBadge(false)
            }
            notificationManager.createNotificationChannel(channel)
        }
    }

    private fun buildNotification(stationCount: Int): Notification {
        val contentText = if (stationCount > 0) {
            "Station $stationCount detected! Arrived safely."
        } else {
            "Monitoring metro trip... Stations detected: 0"
        }

        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setContentTitle("Metro Trip Tracking")
            .setContentText(contentText)
            .setSmallIcon(android.R.drawable.ic_dialog_info)
            .setOngoing(true)
            .setPriority(NotificationCompat.PRIORITY_HIGH)
            .setCategory(NotificationCompat.CATEGORY_SERVICE)
            .build()
    }

    private fun updateNotification(stationCount: Int) {
        val notification = buildNotification(stationCount)
        notificationManager.notify(NOTIFICATION_ID, notification)
    }

    // ──────────────── Haptic feedback ────────────────

    private fun triggerHapticFeedback() {
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                vibrator?.vibrate(
                    VibrationEffect.createOneShot(400, VibrationEffect.DEFAULT_AMPLITUDE)
                )
            } else {
                @Suppress("DEPRECATION")
                vibrator?.vibrate(400)
            }
        } catch (_: Exception) {
            // Vibration not available or permission missing – silently ignore
        }
    }
}
