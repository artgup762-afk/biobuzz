package org.firstinspires.ftc.teamcode.agent;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.List;
import java.util.Set;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.junit.jupiter.api.Test;

class OptionalTurnPinpointTest {
  @Test
  void disabledSensorIsSkippedWithoutInitializingEitherPinpoint() {
    config.turret.turn_pinpoint_enabled = false;
    HardwareMap hw = mock(HardwareMap.class);
    GoBildaPinpointDriver drive = mock(GoBildaPinpointDriver.class);
    GoBildaPinpointDriver turn = mock(GoBildaPinpointDriver.class);
    when(hw.unsafeIterable()).thenReturn(List.of(drive, turn));
    when(hw.getNamesOf(drive)).thenReturn(Set.of("pinpoint"));
    when(hw.getNamesOf(turn)).thenReturn(Set.of("turnpinpoint"));
    for (var source : HardwareRecorder.Source.values()) {
      var recorder =
          new HardwareRecorder(
              hw, new HardwareRecorder.Context(() -> null, () -> 0, () -> 0), 0, source);
      String layout = recorder.layout().toString();
      assertTrue(layout.contains("pinpoint.x"));
      assertFalse(layout.contains("turnpinpoint.x"));
      assertTrue(layout.contains("disabled by turret.turn_pinpoint_enabled"));
    }
    verify(hw, never()).getAll(GoBildaPinpointDriver.class);
    verifyNoInteractions(drive, turn);
  }

  @Test
  void enabledSensorRemainsAvailableToRecorder() {
    config.turret.turn_pinpoint_enabled = true;
    HardwareMap hw = mock(HardwareMap.class);
    GoBildaPinpointDriver turn = mock(GoBildaPinpointDriver.class);
    when(hw.unsafeIterable()).thenReturn(List.of(turn));
    when(hw.getNamesOf(turn)).thenReturn(Set.of("turnpinpoint"));
    var recorder =
        new HardwareRecorder(
            hw,
            new HardwareRecorder.Context(() -> null, () -> 0, () -> 0),
            0,
            HardwareRecorder.Source.PASSIVE);
    assertTrue(recorder.layout().toString().contains("turnpinpoint.x"));
    verifyNoInteractions(turn);
  }
}
