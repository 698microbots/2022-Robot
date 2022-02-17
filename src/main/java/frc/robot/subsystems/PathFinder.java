// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.TrajectoryElement;

// import java.lang.Math;
// import java.util.ArrayList;
// import java.util.Vector;

// public class PathFinder extends SubsystemBase {
//   private double wheelTrack;
//   private double v_max;
//   private double a_max;
//   private double j_max;
//   /** Creates a new PathFinder. */
//   public PathFinder(double robotWheelTrack, double maxVelocity, double maxAcceleration, double maxJerk) {
//     wheelTrack = robotWheelTrack;
//     v_max = maxVelocity;
//     a_max = maxAcceleration;
//     j_max = maxJerk;
//   }

//   private int nextGreaterThan(ArrayList<double[]> list, double target) {
//     int start = 0, end = list.size() - 1;
   
//         int ans = -1;
//         while (start <= end) {
//             int mid = (start + end) / 2;
   
//             // Move to right side if target is
//             // greater.
//             if (list.get(mid)[0] <= target) {
//                 start = mid + 1;
//             }
   
//             // Move left side.
//             else {
//                 ans = mid;
//                 end = mid - 1;
//             }
//         }
//         return ans;
//   }

//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }

//   public double[] calculatePath(double x_0, double x_1, double y_0, double y_1, double theta_0, double theta_1) {
//     double x_dot_0 = Constants.x_dot * (x_1-x_0);
//     double x_dot_1 = Constants.x_dot * (x_1-x_0);
//     double y_dot_0 = x_dot_0 * Math.tan(theta_0*Math.PI/180);
//     double y_dot_1 = x_dot_1 * Math.tan(theta_1*Math.PI/180);
//     double[] result = new double[8];

//     result[0] = 2*x_0 - 2*x_1 + x_dot_0 + x_dot_1;
//     result[1] = -3*x_0 + 3*x_1 - 2*x_dot_0 - x_dot_1;
//     result[2] = x_dot_0;
//     result[3] = x_0;
//     result[4] = 2*y_0 - 2*y_1 + y_dot_0 + y_dot_1;
//     result[5] = -3*y_0 + 3*y_1 - 2*y_dot_0 - y_dot_1;
//     result[6] = y_dot_0;
//     result[7] = y_0;
//     return result;
//   }

//   public ArrayList<TrajectoryElement> calculateTrajectory(double[] coefficients, double v_0, double v_1, double dt) {
//     double lastXPos = coefficients[3];
//     double lastYPos = coefficients[7];
//     double lastXNeg = coefficients[3];
//     double lastYNeg = coefficients[7];
//     double length = 0;
//     double lengthLeft = 0;
//     double lengthRight = 0;
//     double a = coefficients[0];
//     double b = coefficients[1];
//     double c = coefficients[2];
//     double d = coefficients[3];
//     double e = coefficients[4];
//     double f = coefficients[5];
//     double g = coefficients[6];
//     double h = coefficients[7];
//     double inflection1 = (-6*e*c + 6*a*g + Math.sqrt(Math.pow(6*e*c - 6*a*g, 2) - 4*(6*e*b - 6*a*f)*(2*c*f - 2*b*g))) / (2*(6*e*b - 6*a*f));
//     double inflection2 = (-6*e*c + 6*a*g - Math.sqrt(Math.pow(6*e*c - 6*a*g, 2) - 4*(6*e*b - 6*a*f)*(2*c*f - 2*b*g))) / (2*(6*e*b - 6*a*f));
//     double inflection;
//     if(inflection1 > 0 && inflection1 < 1) {
//       inflection = inflection1;
//     }
//     else if (inflection2 > 0 && inflection2 < 1) {
//       inflection = inflection2;
//     }
//     else {
//       inflection = -1;
//     }
//     ArrayList<double[]> lengths = new ArrayList<double[]>();

//     for(double t=0; t<1; t+=.0001) {
//       double xPos = coefficients[0]*Math.pow(t, 3) + coefficients[1]*Math.pow(t, 2) + coefficients[2]*t + coefficients[3];
//       double yPos = coefficients[4]*Math.pow(t, 3) + coefficients[5]*Math.pow(t, 2) + coefficients[6]*t + coefficients[7];
//       double xNeg = xPos;
//       double yNeg = yPos;
//       double dxdt = 3*coefficients[0]*Math.pow(t, 2) + 2*coefficients[1]*t + coefficients[2];
//       double dydt = 3*coefficients[4]*Math.pow(t, 2) + 2*coefficients[5]*t + coefficients[6];
//       double dx2dt2 = 6*coefficients[0]*t + 2*coefficients[1];
//       double dy2dt2 = 6*coefficients[4]*t + 2*coefficients[5];
//       double dydx = dydt / dxdt;
//       double dy2dx2 = (dy2dt2*dxdt - dydt*dx2dt2) / dx2dt2;
      
//       int sign1 = (int) Math.signum(dydx);
//       int sign2 = (int) Math.signum(dy2dx2);
//       if(sign2 == 0) {
//         sign2 = 1;
//       }

//       if(sign1 == 0) {
//         yPos -= wheelTrack/2;
//         yNeg += wheelTrack/2;
//       }
//       else {
//         xPos += sign1 * Math.pow(wheelTrack, 2) / (4 + 4*Math.pow(dydx, -2));
//         xNeg -= sign1 * Math.pow(wheelTrack, 2) / (4 + 4*Math.pow(dydx, -2));
//         yPos -= sign1 * Math.pow(wheelTrack, 2) / (4*dydx + 4*Math.pow(dydx, -1));
//         yNeg += sign1 * Math.pow(wheelTrack, 2) / (4*dydx + 4*Math.pow(dydx, -1));
//       }

//       if(sign2 == 1) {
//         length += Math.sqrt(Math.pow(xPos - lastXPos, 2) + Math.pow(yPos - lastYPos, 2));
//         if(t<inflection) {
//           lengthRight = length;
//         }
//       }
//       else if(sign2 == -1) {
//         length += Math.sqrt(Math.pow(xNeg - lastXNeg, 2) + Math.pow(yNeg - lastYNeg, 2));
//         if(t<inflection) {
//           lengthLeft = length;
//         }
//       }
//       lastXPos = xPos;
//       lastXNeg = xNeg;
//       lastYPos = yPos;
//       lastYNeg = yNeg;

//       double[] temp = new double[4];
//       temp[0] = length;
//       temp[1] = sign1;
//       temp[2] = sign2;
//       temp[3] = t;
//       lengths.add(temp);
//     }

//     double x_0 = 0;
//     double x_1 = length;
//     // Duration of the constant jerk portions of the acceleration segment.
//     double T_j1;
//     // Duration of the constant jerk portions of the deceleration segment.
//     double T_j2;
//     // Duration of the acceleration segment.
//     double T_a;
//     // Duration of the deceleration segment.
//     double T_d;
//     // Duration of the constant velocity segment.
//     double T_v;
//     // Duration of the trajectory.
//     double T;
//     // The maximum acceleration of the trajectory.
//     double a_lim_a;
//     // The manimum acceleration of the trajectory.
//     double a_lim_d;
//     // The maximum velocity of the trajectory.
//     double v_lim;
//     // Displacement.
//     double x;
//     // Time for calculating points.
//     double t;
//     // Vector of TrajectoryElement to be returned.
//     ArrayList<TrajectoryElement> trajectory;
//     // Whether the trajectory will be reversed.
//     int reversed = 1;

//     // Calculate x from x_0 and x_1.
//     x = x_1 - x_0;
//     if(x < 0) {
//         reversed = -1;
//         x *= -1;
//         x_0 *= -1;
//         x_1 *= -1;
//         v_0 *= -1;
//         v_1 *= -1;
//     }

//     // Calculate T_j1 and T_a depending on whether a_max is reached in the acceleration segment (3.19, 3.21, 3.22).
//     if((v_max-v_0)*j_max < Math.pow(a_max, 2)) {
//         T_j1 = Math.sqrt((v_max-v_0)/j_max);
//         T_a = 2*T_j1;
//     } else {
//         T_j1 = a_max/j_max;
//         T_a = T_j1 + (v_max-v_0)/a_max;
//     }

//     // Calculate T_j2 and T_d depending on whether a_max is reached in the deceleration segment (3.20, 3.23, 3.24).
//     if((v_max-v_1)*j_max < Math.pow(a_max, 2)) {
//         T_j2 = Math.sqrt((v_max-v_1)/j_max);
//         T_d = 2*T_j2;
//     } else {
//         T_j2 = a_max/j_max;
//         T_d = T_j2 + (v_max-v_1)/a_max;
//     }

//     // Calculate T_v, negative value indicates that there will be no constant velocity segment (3.25).
//     T_v = (x_1-x_0)/v_max - (T_a/2)*(1+(v_0/v_max)) - (T_d/2)*(1+(v_1/v_max));

//     // If there is no constant velocity segment, recalculate T_j1, T_a, and T_d (3.26, 3.27).
//     if(T_v < 0) {
//         T_v = 0;
//         double delta = Math.pow(a_max, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_max*(4*(x) - 2*a_max/j_max*(v_0+v_1));
//         T_j1 = T_j2 = a_max/j_max;
//         T_a = ((Math.pow(a_max, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_max);
//         T_d = ((Math.pow(a_max, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_max);

//         // Maximum acceleration is not reached (very rare). Described on p. 84.
//         for(double gamma = 1; T_a < 2*T_j1 || T_d < 2*T_j1; gamma -= 0.001) {
//             double a_temp = gamma * a_max;
//             delta = Math.pow(a_temp, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_temp*(4*(x) - 2*a_temp/j_max*(v_0+v_1));
//             T_j1 = T_j2 = a_temp/j_max;
//             T_a = ((Math.pow(a_temp, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_temp);
//             T_d = ((Math.pow(a_temp, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_temp);

//             // If T_a becomes negative, the acceleration segment is unnecessary (3.28).
//             if(T_a < 0) {
//                 T_a = 0;
//                 T_j1 = 0;
//                 T_d = 2*x/(v_1+v_0);
//                 T_j2 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) + Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
//             }

//             // If T_d becomes negative, the deceleration segment is unnecessary (3.29).
//             if(T_d < 0) {
//                 T_d = 0;
//                 T_j2 = 0;
//                 T_a = 2*x/(v_1+v_0);
//                 T_j1 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) - Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
//             }
//         }
//     }

//     // Now all durations have been determined, so maximum/minimum accelerations and maximum velocity can be found.
//     a_lim_a = j_max*T_j1;
//     a_lim_d = -1*j_max*T_j2;
//     v_lim = v_0 + (T_a-T_j1)*a_lim_a;

//     // Calculate total duration.
//     T = T_a + T_v + T_d;

//     double lastXLeft = 0;
//     double lastXRight = 0;
//     double lastVLeft = v_0;
//     double lastVRight = v_0;
//     double lastAleft = 0;
//     double lastARight = 0;

//     // Uing the durations found and kinematic equations, calculate x, v, a, j for each period dt in the trajectory (3.30).
//     // Segment a: acceleration with positive jerk.
//     while(t < T_j1) {
//         TrajectoryElement temp;
//         double j = j_max;
//         double a = j_max*t;
//         double v = v_0 + j_max*Math.pow(t, 2)/2;
//         double xTemp = x_0 + v_0*t + j_max*Math.pow(t, 3)/6;
//         int sign1 = (int) lengths.get(nextGreaterThan(lengths, xTemp))[1];
//         int sign2 = (int) lengths.get(nextGreaterThan(lengths, xTemp))[2];
//         double s = lengths.get(nextGreaterThan(lengths, xTemp))[3];

//         if(sign2 == 0) {
//           temp.rightJ = j;
//           temp.rightA = a;
//           temp.rightV = v;
//           temp.leftJ = j;
//           temp.leftA = a;
//           temp.leftV = v;
//         }
//         if(sign2 > 0) {
//           temp.rightJ = j;
//           temp.rightA = a;
//           temp.rightV = v;
//           temp.rightX = xTemp;
//         }
//         else if(sign2 < 0) {
//           temp.leftJ = j;
//           temp.leftA = a;
//           temp.leftV = v;
//           temp.leftX = xTemp;
//         }

//         temp.t = t;
//         temp.j = j_max;
//         temp.a = j_max*t;
//         temp.v = v_0 + j_max*Math.pow(t, 2)/2;
//         temp.x = x_0 + v_0*t + j_max*Math.pow(t, 3)/6;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment b: constant acceleration.
//     while(t < T_a-T_j1) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = 0;
//         temp.a = a_lim_a;
//         temp.v = v_0 + a_lim_a*(t - T_j1/2);
//         temp.x = x_0 + v_0*t + a_lim_a/6 * (3*Math.pow(t, 2) - 3*T_j1*t + Math.pow(T_j1, 2));

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment c: acceleration with negative jerk.
//     while(t < T_a) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = -1*j_max;
//         temp.a = j_max*(T_a-t);
//         temp.v = v_lim - j_max*Math.pow((T_a - t), 2)/2;
//         temp.x = x_0 + (v_lim + v_0)*T_a/2 - v_lim*(T_a-t) + j_max*Math.pow((T_a-t), 3)/6;

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment d: constant velocity.
//     while(t < T_a+T_v) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = 0;
//         temp.a = 0;
//         temp.v = v_lim;
//         temp.x = x_0 + (v_lim + v_0)*T_a/2 + v_lim*(t-T_a);

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment e: deceleration with negative jerk.
//     while(t < T-T_d+T_j2) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = -1*j_max;
//         temp.a = -1*j_max*(t-T+T_d);
//         temp.v = v_lim - j_max*Math.pow((t-T+T_d), 2)/2;
//         temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) - j_max*Math.pow((t-T+T_d), 3)/6;

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment f: constant deceleration.
//     while(t < T-T_j2) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = 0;
//         temp.a = a_lim_d;
//         temp.v = v_lim + a_lim_d*(t-T+T_d-T_j2/2);
//         temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) + a_lim_d/6 * (3*Math.pow(t-T+T_d, 2) - 3*T_j2*(t-T+T_d) + Math.pow(T_j2, 2));

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }
//     // Segment g: deceleration with positive jerk
//     while(t < T) {
//         TrajectoryElement temp;
//         temp.t = t;
//         temp.j = j_max;
//         temp.a = -1*j_max*(T-t);
//         temp.v = v_1 + j_max*Math.pow(T-t, 2)/2;
//         temp.x = x_1 - v_1*(T-t) - j_max*Math.pow(T-t, 3)/6;

//         temp.j *= reversed;
//         temp.a *= reversed;
//         temp.v *= reversed;
//         temp.x *= reversed;

//         trajectory.push_back(temp);
//         t += dt;
//     }

//     // Ensure that the final state is reached even if it goes slightly beyond T.
//     TrajectoryElement final;
//     final.t = t;
//     final.j = 0;
//     final.a = 0;
//     final.v = v_1;
//     final.x = x_1;

//     final.j *= reversed;
//     final.a *= reversed;
//     final.v *= reversed;
//     final.x *= reversed;
    
//     trajectory.push_back(final);


//     return trajectory;
//   }
// }
