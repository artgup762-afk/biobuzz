package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.util.RobotLog;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.firstinspires.ftc.vision.VisionPortal;

/** Keeps SDK camera semaphore waits off the OpMode thread, including during STOP. */
final class VisionPortalLifecycle implements AutoCloseable {
  private final Runnable resumeStream;
  private final Runnable stopStream;
  private final Runnable closeCamera;
  private final ExecutorService worker;
  private volatile boolean closed;
  private boolean streamingRequested;

  VisionPortalLifecycle(VisionPortal portal, boolean streaming) {
    this(
        portal::resumeStreaming,
        portal::stopStreaming,
        portal::close,
        streaming,
        Executors.newSingleThreadExecutor(
            task -> {
              Thread thread = new Thread(task, "HiveVision-Camera");
              thread.setDaemon(true);
              return thread;
            }));
  }

  VisionPortalLifecycle(
      Runnable resumeStream,
      Runnable stopStream,
      Runnable closeCamera,
      boolean streaming,
      ExecutorService worker) {
    this.resumeStream = resumeStream;
    this.stopStream = stopStream;
    this.closeCamera = closeCamera;
    this.streamingRequested = streaming;
    this.worker = worker;
  }

  synchronized void setStreaming(boolean enabled) {
    if (closed || streamingRequested == enabled) return;
    streamingRequested = enabled;
    worker.execute(
        () -> {
          if (closed) return;
          try {
            if (enabled) resumeStream.run();
            else stopStream.run();
          } catch (RuntimeException e) {
            RobotLog.ee("HiveVision", e, "Camera streaming request failed");
            synchronized (this) {
              // Allow the next matching request to retry if this is still the requested state.
              if (!closed && streamingRequested == enabled) streamingRequested = !enabled;
            }
          }
        });
  }

  @Override
  public synchronized void close() {
    if (closed) return;
    closed = true;
    worker.execute(
        () -> {
          try {
            // SDK 12 close() waits for asynchronous opening/streaming to release a semaphore.
            // A wait here must never consume the OpMode's STOP watchdog budget.
            closeCamera.run();
          } catch (RuntimeException e) {
            RobotLog.ee("HiveVision", e, "Camera close failed");
          }
        });
    worker.shutdown(); // Drain the close task without waiting or interrupting SDK cleanup.
  }
}
