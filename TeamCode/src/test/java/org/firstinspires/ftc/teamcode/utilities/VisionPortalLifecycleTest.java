package org.firstinspires.ftc.teamcode.utilities;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;

class VisionPortalLifecycleTest {
  private final Runnable resume = mock(Runnable.class);
  private final Runnable stop = mock(Runnable.class);
  private final Runnable close = mock(Runnable.class);
  private final ExecutorService worker = mock(ExecutorService.class);
  private final List<Runnable> tasks = new ArrayList<>();

  private VisionPortalLifecycle lifecycle(boolean streaming) {
    doAnswer(
            call -> {
              tasks.add(call.getArgument(0));
              return null;
            })
        .when(worker)
        .execute(any(Runnable.class));
    return new VisionPortalLifecycle(resume, stop, close, streaming, worker);
  }

  @Test
  void closeReturnsWithoutCallingSdkAndRunsExactlyOnceOnWorker() {
    var camera = lifecycle(true);
    camera.close();
    camera.close();
    camera.setStreaming(false);
    camera.setStreaming(true);
    verifyNoInteractions(resume, stop, close);
    assertEquals(1, tasks.size());
    verify(worker).shutdown();
    verify(worker, never()).shutdownNow();
    tasks.get(0).run();
    verify(close).run();
    verifyNoMoreInteractions(resume, stop, close);
  }

  @Test
  void stopDiscardsQueuedStreamChangesAndStillReleasesCamera() {
    var camera = lifecycle(true);
    camera.setStreaming(false);
    camera.setStreaming(true);
    camera.close();
    tasks.forEach(Runnable::run);
    verify(close).run();
    verifyNoMoreInteractions(resume, stop, close);
  }

  @Test
  void streamingChangesAreDeferredOrderedAndRepeatedRequestsCoalesced() {
    var camera = lifecycle(false);
    camera.setStreaming(false);
    camera.setStreaming(true);
    camera.setStreaming(true);
    camera.setStreaming(false);
    verifyNoInteractions(resume, stop, close);
    assertEquals(2, tasks.size());
    tasks.forEach(Runnable::run);
    var order = inOrder(resume, stop, close);
    order.verify(resume).run();
    order.verify(stop).run();
    order.verifyNoMoreInteractions();
  }

  @Test
  void failedStreamRequestCanBeRetried() {
    var camera = lifecycle(false);
    doThrow(new IllegalStateException("opening")).doNothing().when(resume).run();
    camera.setStreaming(true);
    tasks.get(0).run();
    camera.setStreaming(true);
    tasks.get(1).run();
    verify(resume, times(2)).run();
  }

  @Test
  void sdkCloseFailureDoesNotEscapeStopOrLeaveWorkerAcceptingWork() {
    var camera = lifecycle(true);
    doThrow(new IllegalStateException("camera error")).when(close).run();
    camera.close();
    assertDoesNotThrow(() -> tasks.get(0).run());
    verify(worker).shutdown();
    camera.close();
    assertEquals(1, tasks.size());
  }

  @Test
  @Timeout(5)
  void openingCameraMayBlockCleanupWithoutBlockingCaller() throws Exception {
    CountDownLatch entered = new CountDownLatch(1);
    CountDownLatch opened = new CountDownLatch(1);
    ExecutorService executor = Executors.newSingleThreadExecutor();
    doAnswer(
            call -> {
              entered.countDown();
              opened.await();
              return null;
            })
        .when(close)
        .run();
    var camera = new VisionPortalLifecycle(resume, stop, close, true, executor);
    try {
      camera.close();
      assertTrue(entered.await(1, TimeUnit.SECONDS));
      assertFalse(executor.isTerminated());
      camera.close();
      opened.countDown();
      assertTrue(executor.awaitTermination(1, TimeUnit.SECONDS));
      verify(close).run();
    } finally {
      opened.countDown();
      executor.shutdownNow();
    }
  }
}
