package frc.robot.utils;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.*;

public class RobotMath {
  /**
   * @param measure measure to check.
   * @param bounds bounds to check against.
   * @return true if -bounds < measure < bounds, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> bounds) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(bounds)) {
      return false;
    }
    return measure.lt(bounds) && measure.gt(bounds.unaryMinus());
  }

  /**
   * @param measure measure to check.
   * @param lower lower bound to check against.
   * @param upper upper bound to check against.
   * @return true if lower < measure < upper, false otherwise.
   */
  public static <U extends Unit> boolean measureWithinBounds(
      Measure<U> measure, Measure<U> lower, Measure<U> upper) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    if (isMeasureNaN(lower)) {
      return false;
    }
    if (isMeasureNaN(upper)) {
      return false;
    }
    return measure.lt(upper) && measure.gt(lower);
  }

  /**
   * @param measure measure to get the absolute value of.
   * @param other measuere to get dist to.
   * @return abs of measuere1 - measuere2.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M dist(M measure1, M measure2) {
    // No need for NaN checks as minus with return NaN if either are NaN
    return abs((M) measure1.minus(measure2));
  }

  /**
   * @param measure measure to get the absolute value of.
   * @return the absolute value of measure with the zero point of the unit for that type.
   */
  @SuppressWarnings("unchecked")
  public static <U extends Unit, M extends Measure<U>> M abs(M measure) {
    if (isMeasureNaN(measure)) {
      // NaN's propagate
      return (M) measure.baseUnit().of(Double.NaN);
    }
    return measure.gte((M) measure.unit().zero())
        ? measure
        : (M) ((M) measure.unit().zero()).minus(measure);
  }

  public static <U extends Unit> boolean isWithinTolerance(
      Measure<U> measure1, Measure<U> measure2, Measure<U> tol) {
    if (isMeasureNaN(measure1)) {
      return false;
    }
    if (isMeasureNaN(measure2)) {
      return false;
    }
    if (isMeasureNaN(tol)) {
      return false;
    }
    return dist(measure1, measure2).lte(tol);
  }

  @SuppressWarnings("unchecked")
  public static <U extends Unit> boolean approxZero(Measure<U> measure) {
    return approxZero(measure, (Measure<U>) measure.unit().of(1e-6));
  }

  public static <U extends Unit> boolean approxZero(Measure<U> measure, Measure<U> tol) {
    if (isMeasureNaN(measure)) {
      return false;
    }
    return abs(measure).lte(tol);
  }

  public static <U extends Unit> boolean isMeasureNaN(Measure<U> measure) {
    return Double.isNaN(measure.baseUnitMagnitude());
  }

  public static boolean isPoseInSquare(Pose2d pose, Pose2d corner1, Pose2d corner2) {
    var leftMostCorner = corner1.getX() <= corner2.getX() ? corner1 : corner2;
    var rightMostCorner = corner1.getX() >= corner2.getX() ? corner1 : corner2;
    var bottomMostCorner = corner1.getY() <= corner2.getY() ? corner1 : corner2;
    var topMostCorner = corner1.getY() >= corner2.getY() ? corner1 : corner2;

    return pose.getX() >= leftMostCorner.getX()
        && pose.getX() <= rightMostCorner.getX()
        && pose.getY() >= bottomMostCorner.getY()
        && pose.getY() <= topMostCorner.getY();
  }

  public static final Distance getDistanceBetweenPoses(Pose2d pose1, Pose2d pose2) {
    var relitivePose = pose1.relativeTo(pose2);
    return Meters.of(
        Math.sqrt(Math.pow(relitivePose.getX(), 2) + Math.pow(relitivePose.getY(), 2)));
  }
}
