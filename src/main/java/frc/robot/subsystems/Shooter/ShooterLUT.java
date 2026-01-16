package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;

public class ShooterLUT {
  public record ShooterSetpoint(LinearVelocity flywheelSurfaceSpeed, Angle rotation) {}

  public static ShooterSetpoint getSpeedAndRotation(Distance distance) {
    return new ShooterSetpoint(
        MetersPerSecond.of(kShooterSpeedMap.get(distance.in(Meters))),
        Rotations.of(kShooterAngleMap.get(distance.in(Meters))));
  }

  private static final InterpolatingDoubleTreeMap kShooterSpeedMap = generateSpeedMap();
  private static final InterpolatingDoubleTreeMap kShooterAngleMap = generateAngleMap();

  private static InterpolatingDoubleTreeMap generateSpeedMap() {
    var map = new InterpolatingDoubleTreeMap();

    map.put(0.55, 23.64333);
    map.put(0.66176, 23.65667);
    map.put(0.77352, 23.69667);
    map.put(0.88527, 23.92333);
    map.put(0.99703, 24.14);
    map.put(1.10879, 24.46667);
    map.put(1.22055, 24.75);
    map.put(1.3323, 24.76667);
    map.put(1.444406, 25.0);
    map.put(1.55582, 25.3);
    map.put(1.66758, 25.5);
    map.put(1.77933, 23.64333);
    map.put(1.89109, 25.88333);
    map.put(2.00285, 26.01333);
    map.put(2.11461, 26.11667);
    map.put(2.22636, 26.22667);
    map.put(2.33812, 26.36667);
    map.put(2.89691, 27.03333);
    map.put(3.00867, 27.16);
    map.put(3.56736, 27.77333);
    map.put(4.12624, 28.38333);
    map.put(4.238, 28.49);
    map.put(5.13206, 29.46);
    map.put(5.69085, 30.11);
    map.put(6.13788, 30.63333);

    return map;
  }

  private static InterpolatingDoubleTreeMap generateAngleMap() {
    var map = new InterpolatingDoubleTreeMap();

    map.put(0.55, 89.669);
    map.put(0.66176, 87.82);
    map.put(0.77352, 85.99);
    map.put(0.88527, 84.425);
    map.put(0.99703, 83.74);
    map.put(1.10879, 83.17);
    map.put(1.22055, 82.63);
    map.put(1.3323, 81.8);
    map.put(1.444406, 81.26);
    map.put(1.55582, 80.82);
    map.put(1.66758, 80.26);
    map.put(1.77933, 79.69);
    map.put(1.89109, 79.23);
    map.put(2.00285, 78.7);
    map.put(2.11461, 78.1);
    map.put(2.22636, 77.55);
    map.put(2.33812, 77.05);
    map.put(2.89691, 74.56);
    map.put(3.00867, 74.2);
    map.put(3.56736, 71.65);
    map.put(4.12624, 69.28);
    map.put(4.238, 68.83);
    map.put(5.13206, 65.19);
    map.put(5.69085, 63.04);
    map.put(6.13788, 61.34);

    return map;
  }
}
