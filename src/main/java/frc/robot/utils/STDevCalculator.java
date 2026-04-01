package frc.robot.utils;

import java.util.List;

public class STDevCalculator {
  // https://www.mathsisfun.com/data/standard-deviation-formulas.html
  public static double calculateSTDevs(List<Double> data) {
    if (data.size() == 0) return 0;
    var mean = 0.0;
    for (var dataPoint : data) {
      mean += dataPoint;
    }
    mean = mean / data.size();

    var squaredMeanDifferenceMean = 0.0;
    for (var dataPoint : data) {
      squaredMeanDifferenceMean += Math.pow(dataPoint - mean, 2);
    }
    squaredMeanDifferenceMean = squaredMeanDifferenceMean / data.size();

    return Math.sqrt(squaredMeanDifferenceMean);
  }
}
