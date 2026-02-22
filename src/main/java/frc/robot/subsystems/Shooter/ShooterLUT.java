package frc.robot.subsystems.Shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;

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
    if (RobotBase.isReal()) {
      System.out.println("Real");
      map.put(1.8546, 60.0);
      map.put(2.221275, 65.0);
      map.put(2.647, 65.0);
      map.put(2.934, 67.5);
      map.put(3.333, 70.0);
      map.put(3.70912, 75.0);
      map.put(4.1389, 77.5);
      map.put(4.5181, 80.0);
    } else {
      System.out.println("Sim");
      map.put(1.0, 26.25);
      map.put(1.1, 26.75);
      map.put(1.3, 27.0);
      map.put(1.5, 27.1);
      map.put(1.7, 27.2);
      map.put(2.25, 27.85);
      map.put(2.5, 28.15);
      map.put(3.5, 29.25);
      map.put(4.0, 30.0);
      map.put(4.5, 30.5);
      map.put(5.0, 30.85);
      map.put(5.5, 31.6);
      map.put(6.13788, 32.5);
      System.out.println(map.get(3.5));
    }
    return map;
  }

  private static InterpolatingDoubleTreeMap generateAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    if (RobotBase.isReal()) {
      map.put(1.8546, 25.0);
      map.put(2.221275, 25.0);
      map.put(2.647, 27.5);
      map.put(2.934, 27.5);
      map.put(3.333, 27.5);
      map.put(3.70912, 27.5);
      map.put(4.1389, 30.0);
      map.put(4.5181, 30.0);
    } else {
      map.put(0.99703, 6.26);
      map.put(1.10879, 6.83);
      map.put(1.22055, 7.37);
      map.put(1.3323, 8.2);
      map.put(1.444406, 8.74);
      map.put(1.55582, 9.18);
      map.put(1.66758, 9.74);
      map.put(1.77933, 10.31);
      map.put(1.89109, 10.77);
      map.put(2.11461, 11.9);
      map.put(2.22636, 12.45);
      map.put(2.33812, 12.95);
      map.put(3.00867, 15.8);
      map.put(3.56736, 18.35);
      map.put(4.12624, 20.72);
      map.put(4.238, 21.17);
      map.put(5.13206, 24.81);
      map.put(5.69085, 26.96);
      map.put(6.13788, 28.66);
    }
    return map;
  }
}
