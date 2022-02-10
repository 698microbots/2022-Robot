// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;
import java.util.ArrayList;
import java.util.Vector;

public class PathFinder extends SubsystemBase {
  private double wheelTrack;
  private double v_max;
  private double a_max;
  private double j_max;
  /** Creates a new PathFinder. */
  public PathFinder(double robotWheelTrack, double maxVelocity, double maxAcceleration, double maxJerk) {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double[] calculatePath(double x_0, double x_1, double y_0, double y_1, double theta_0, double theta_1) {
    double delta_x = x_1 - x_0;
    double c = Math.tan(theta_0*Math.PI/180);
    double d = Math.tan(theta_0*Math.PI/180);
    double[] result = new double[4];

    result[0] = (c*delta_x + d*delta_x + 2*y_0 - 2*y_1) / Math.pow(delta_x, 3);
    result[1] = (3*y_1 - d*delta_x - 3*y_0 - 2*c*delta_x) / Math.pow(delta_x, 2);
    result[2] = c;
    result[3] = y_0;
    result[4] = x_0;
    result[5] = x_1;
    return result;
  }

  public ArrayList<double[]> calculateTrajectory(double[] coefficients) {
    double x_0 = coefficients[4];
    double x_1 = coefficients[5];
    double y_0 = coefficients[3];
    double a = coefficients[0];
    double b = coefficients[1];
    double c = coefficients[2];
    double d = coefficients[3];
    double inflection = (-1/3)*(b/a);
    double length = 0;
    double lastX = x_0;
    double lastY = y_0;

    for(double x=x_0; x<inflection; x += (x_1-x_0)/10000) {
      double y = a*Math.pow(x-x_0, 3) + b*Math.pow(x-x_0, 2) + c*(x-x_0) + d;
      double perpendicularSlope = 1/(3*a*Math.pow(x-x_0, 2) + 2*b*(x-x_0) + c);

    }
  }
}
