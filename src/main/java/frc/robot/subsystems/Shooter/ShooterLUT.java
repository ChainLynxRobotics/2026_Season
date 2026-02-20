package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;

public class ShooterLUT {
  public record ShooterSetpoint(LinearVelocity flywheelSurfaceSpeed, Angle rotation) {}

  public static ShooterSetpoint getSpeedAndRotation(Distance distance) {
    return new ShooterSetpoint(
        MetersPerSecond.of(kShooterSpeedMap.get(distance.in(Meters))),
        Degrees.of(kShooterAngleMap.get(distance.in(Meters))));
  }

  private static final InterpolatingDoubleTreeMap kShooterSpeedMap = generateSpeedMap();
  private static final InterpolatingDoubleTreeMap kShooterAngleMap = generateAngleMap();

  private static InterpolatingDoubleTreeMap generateSpeedMap() {
    var map = new InterpolatingDoubleTreeMap();

    map.put(1.8546, 60.0);
    map.put(2.221275, 65.0);
    map.put(2.647, 65.0);
    map.put(2.934, 67.5);
    map.put(3.333, 70.0);
    map.put(3.70912, 75.0);
    map.put(4.1389, 77.5);
    map.put(4.5181, 80.0);

    return map;
  }

  private static InterpolatingDoubleTreeMap generateAngleMap() {
    var map = new InterpolatingDoubleTreeMap();

    map.put(1.8546, 25.0);
    map.put(2.221275, 25.0);
    map.put(2.647, 27.5);
    map.put(2.934, 27.5);
    map.put(3.333, 27.5);
    map.put(3.70912, 27.5);
    map.put(4.1389, 30.0);
    map.put(4.5181, 30.0);

    return map;
  }
}
