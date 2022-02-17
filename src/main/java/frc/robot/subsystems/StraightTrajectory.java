// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.TrajectoryElement;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;
import java.util.*;

public class StraightTrajectory extends SubsystemBase {
    public double L;
    public double v_max;
    public double a_max;
    public double j_max;
    private int counter = 0;
    private ArrayList<TrajectoryElement> trajectory;

    /** Creates a new StraightTrajectory. */
    public StraightTrajectory(double wheelTrack, double v, double a, double j) {
        L = wheelTrack;
        v_max = v;
        a_max = a;
        j_max = j;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
    /**
        * Calculate an s-curve trajectory with stepwise jerk for a 1D path.
        * 
        * Creates a 7-segment trajectory.
        * Method from Trajectory Planning for Automatic Machines and Robots by Luigi Biagiotti and Claudio Melchiorri, pp. 79-107.
        * 
        * @param x_0 initial position (m).
        * @param x_1 final position (m).
        * @param v_0 initial velocity (m/s).
        * @param v_1 final velocity (m/s).
        * @param dt time increment (s).
        * @return vector<TrajectoryElement> set of TrajectoryElements forming the trajectory.
        */
    public ArrayList<TrajectoryElement> calculateStraight(double x_0, double x_1, double v_0, double v_1, double dt) {
        // Duration of the constant jerk portions of the acceleration segment.
        double T_j1;
        // Duration of the constant jerk portions of the deceleration segment.
        double T_j2;
        // Duration of the acceleration segment.
        double T_a;
        // Duration of the deceleration segment.
        double T_d;
        // Duration of the constant velocity segment.
        double T_v;
        // Duration of the trajectory.
        double T;
        // The maximum acceleration of the trajectory.
        double a_lim_a;
        // The manimum acceleration of the trajectory.
        double a_lim_d;
        // The maximum velocity of the trajectory.
        double v_lim;
        // Displacement.
        double x;
        // Time for calculating points.
        double t = 0;
        // Vector of TrajectoryElement to be returned.
        ArrayList<TrajectoryElement> trajectory = new ArrayList<TrajectoryElement>();
        // Whether the trajectory will be reversed.
        int reversed = 1;

        // Calculate x from x_0 and x_1.
        x = x_1 - x_0;
        if(x < 0) {
            reversed = -1;
            x *= -1;
            x_0 *= -1;
            x_1 *= -1;
            v_0 *= -1;
            v_1 *= -1;
        }

        // Calculate T_j1 and T_a depending on whether a_max is reached in the acceleration segment (3.19, 3.21, 3.22).
        if((v_max-v_0)*j_max < Math.pow(a_max, 2)) {
            T_j1 = Math.sqrt((v_max-v_0)/j_max);
            T_a = 2*T_j1;
        } else {
            T_j1 = a_max/j_max;
            T_a = T_j1 + (v_max-v_0)/a_max;
        }

        // Calculate T_j2 and T_d depending on whether a_max is reached in the deceleration segment (3.20, 3.23, 3.24).
        if((v_max-v_1)*j_max < Math.pow(a_max, 2)) {
            T_j2 = Math.sqrt((v_max-v_1)/j_max);
            T_d = 2*T_j2;
        } else {
            T_j2 = a_max/j_max;
            T_d = T_j2 + (v_max-v_1)/a_max;
        }

        // Calculate T_v, negative value indicates that there will be no constant velocity segment (3.25).
        T_v = (x_1-x_0)/v_max - (T_a/2)*(1+(v_0/v_max)) - (T_d/2)*(1+(v_1/v_max));

        // If there is no constant velocity segment, recalculate T_j1, T_a, and T_d (3.26, 3.27).
        if(T_v < 0) {
            T_v = 0;
            double delta = Math.pow(a_max, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_max*(4*(x) - 2*a_max/j_max*(v_0+v_1));
            T_j1 = T_j2 = a_max/j_max;
            T_a = ((Math.pow(a_max, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_max);
            T_d = ((Math.pow(a_max, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_max);

            // Maximum acceleration is not reached (very rare). Described on p. 84.
            for(double gamma = 1; T_a < 2*T_j1 || T_d < 2*T_j1; gamma -= 0.001) {
                double a_temp = gamma * a_max;
                delta = Math.pow(a_temp, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_temp*(4*(x) - 2*a_temp/j_max*(v_0+v_1));
                T_j1 = T_j2 = a_temp/j_max;
                T_a = ((Math.pow(a_temp, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_temp);
                T_d = ((Math.pow(a_temp, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_temp);

                // If T_a becomes negative, the acceleration segment is unnecessary (3.28).
                if(T_a < 0) {
                    T_a = 0;
                    T_j1 = 0;
                    T_d = 2*x/(v_1+v_0);
                    T_j2 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) + Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
                }

                // If T_d becomes negative, the deceleration segment is unnecessary (3.29).
                if(T_d < 0) {
                    T_d = 0;
                    T_j2 = 0;
                    T_a = 2*x/(v_1+v_0);
                    T_j1 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) - Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
                }
            }
        }

        // Now all durations have been determined, so maximum/minimum accelerations and maximum velocity can be found.
        a_lim_a = j_max*T_j1;
        a_lim_d = -1*j_max*T_j2;
        v_lim = v_0 + (T_a-T_j1)*a_lim_a;

        // Calculate total duration.
        T = T_a + T_v + T_d;

        // Uing the durations found and kinematic equations, calculate x, v, a, j for each period dt in the trajectory (3.30).
        // Segment a: acceleration with positive jerk.
        while(t < T_j1) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = j_max;
            temp.a = j_max*t;
            temp.v = v_0 + j_max*Math.pow(t, 2)/2;
            temp.x = x_0 + v_0*t + j_max*Math.pow(t, 3)/6;

            trajectory.add(temp);
            t += dt;
        }
        // Segment b: constant acceleration.
        while(t < T_a-T_j1) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = a_lim_a;
            temp.v = v_0 + a_lim_a*(t - T_j1/2);
            temp.x = x_0 + v_0*t + a_lim_a/6 * (3*Math.pow(t, 2) - 3*T_j1*t + Math.pow(T_j1, 2));

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment c: acceleration with negative jerk.
        while(t < T_a) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = -1*j_max;
            temp.a = j_max*(T_a-t);
            temp.v = v_lim - j_max*Math.pow((T_a - t), 2)/2;
            temp.x = x_0 + (v_lim + v_0)*T_a/2 - v_lim*(T_a-t) + j_max*Math.pow((T_a-t), 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment d: constant velocity.
        while(t < T_a+T_v) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = 0;
            temp.v = v_lim;
            temp.x = x_0 + (v_lim + v_0)*T_a/2 + v_lim*(t-T_a);

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment e: deceleration with negative jerk.
        while(t < T-T_d+T_j2) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = -1*j_max;
            temp.a = -1*j_max*(t-T+T_d);
            temp.v = v_lim - j_max*Math.pow((t-T+T_d), 2)/2;
            temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) - j_max*Math.pow((t-T+T_d), 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment f: constant deceleration.
        while(t < T-T_j2) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = a_lim_d;
            temp.v = v_lim + a_lim_d*(t-T+T_d-T_j2/2);
            temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) + a_lim_d/6 * (3*Math.pow(t-T+T_d, 2) - 3*T_j2*(t-T+T_d) + Math.pow(T_j2, 2));

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment g: deceleration with positive jerk
        while(t < T) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = j_max;
            temp.a = -1*j_max*(T-t);
            temp.v = v_1 + j_max*Math.pow(T-t, 2)/2;
            temp.x = x_1 - v_1*(T-t) - j_max*Math.pow(T-t, 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }

        // Ensure that the last state is reached even if it goes slightly beyond T.
        TrajectoryElement last = new TrajectoryElement();
        last.t = t;
        last.j = 0;
        last.a = 0;
        last.v = v_1;
        last.x = x_1;

        last.j *= reversed;
        last.a *= reversed;
        last.v *= reversed;
        last.x *= reversed;
        
        trajectory.add(last);


        return trajectory;
    }

    /**
      * Calculate an s-curve trajectory with stepwise jerk for a counterclockwise point turn.
      * 
      * Creates a 7-segment trajectory.
      * Should be used by following positive with right side, negative with left side.
      * Method from Trajectory Planning for Automatic Machines and Robots by Luigi Biagiotti and Claudio Melchiorri, pp. 79-107.
      * 
      * @param theta_0 initial angle (deg).
      * @param theta_1 final angle (deg).
      * @param omega_0 initial angular velocity (deg/s).
      * @param omega_1 final velocity velocity (deg/s).
      * @param dt time increment (s).
      * @return vector<TrajectoryElement> set of TrajectoryElements forming the trajectory.
      */
    public ArrayList<TrajectoryElement> calculateTurn(double theta_0, double theta_1, double omega_0, double omega_1, double dt) {
        // Calculated intitial position from intital angle.
        double x_0 = L/2 * (theta_0*Math.PI/180);
        // Calculated final position from final angle.
        double x_1 = L/2 * (theta_1*Math.PI/180);
        // Calculated intitial velocity from intital angular velocity.
        double v_0 = L/2 * (omega_0*Math.PI/180);
        // Calculated final velocity from final angular velocity.
        double v_1 = L/2 * (omega_1*Math.PI/180);
        // Duration of the constant jerk portions of the acceleration segment.
        double T_j1;
        // Duration of the constant jerk portions of the deceleration segment.
        double T_j2;
        // Duration of the acceleration segment.
        double T_a;
        // Duration of the deceleration segment.
        double T_d;
        // Duration of the constant velocity segment.
        double T_v;
        // Duration of the trajectory.
        double T;
        // The maximum acceleration of the trajectory.
        double a_lim_a;
        // The manimum acceleration of the trajectory.
        double a_lim_d;
        // The maximum velocity of the trajectory.
        double v_lim;
        // Displacement.
        double x;
        // Time for calculating points.
        double t = 0;
        // Vector of TrajectoryElement to be returned.
        ArrayList<TrajectoryElement> trajectory = new ArrayList<TrajectoryElement>();
        // Whether the trajectory will be reversed.
        int reversed = 1;

        // Calculate x from x_0 and x_1.
        x = x_1 - x_0;
        if(x < 0) {
            reversed = -1;
            x *= -1;
            x_0 *= -1;
            x_1 *= -1;
            v_0 *= -1;
            v_1 *= -1;
        }

        // Calculate T_j1 and T_a depending on whether a_max is reached in the acceleration segment (3.19, 3.21, 3.22).
        if((v_max-v_0)*j_max < Math.pow(a_max, 2)) {
            T_j1 = Math.sqrt((v_max-v_0)/j_max);
            T_a = 2*T_j1;
        } else {
            T_j1 = a_max/j_max;
            T_a = T_j1 + (v_max-v_0)/a_max;
        }

        // Calculate T_j2 and T_d depending on whether a_max is reached in the deceleration segment (3.20, 3.23, 3.24).
        if((v_max-v_1)*j_max < Math.pow(a_max, 2)) {
            T_j2 = Math.sqrt((v_max-v_1)/j_max);
            T_d = 2*T_j2;
        } else {
            T_j2 = a_max/j_max;
            T_d = T_j2 + (v_max-v_1)/a_max;
        }

        // Calculate T_v, negative value indicates that there will be no constant velocity segment (3.25).
        T_v = (x_1-x_0)/v_max - (T_a/2)*(1+(v_0/v_max)) - (T_d/2)*(1+(v_1/v_max));

        // If there is no constant velocity segment, recalculate T_j1, T_a, and T_d (3.26, 3.27).
        if(T_v < 0) {
            T_v = 0;
            double delta = Math.pow(a_max, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_max*(4*(x) - 2*a_max/j_max*(v_0+v_1));
            T_j1 = T_j2 = a_max/j_max;
            T_a = ((Math.pow(a_max, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_max);
            T_d = ((Math.pow(a_max, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_max);

            // Maximum acceleration is not reached (very rare). Described on p. 84.
            for(double gamma = 1; T_a < 2*T_j1 || T_d < 2*T_j1; gamma -= 0.001) {
                double a_temp = gamma * a_max;
                delta = Math.pow(a_temp, 4)/Math.pow(j_max, 2) + 2*(Math.pow(v_0, 2) + Math.pow(v_1, 2)) + a_temp*(4*(x) - 2*a_temp/j_max*(v_0+v_1));
                T_j1 = T_j2 = a_temp/j_max;
                T_a = ((Math.pow(a_temp, 2)/j_max) - 2*v_0 + Math.sqrt(delta)) / (2*a_temp);
                T_d = ((Math.pow(a_temp, 2)/j_max) - 2*v_1 + Math.sqrt(delta)) / (2*a_temp);

                // If T_a becomes negative, the acceleration segment is unnecessary (3.28).
                if(T_a < 0) {
                    T_a = 0;
                    T_j1 = 0;
                    T_d = 2*x/(v_1+v_0);
                    T_j2 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) + Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
                }

                // If T_d becomes negative, the deceleration segment is unnecessary (3.29).
                if(T_d < 0) {
                    T_d = 0;
                    T_j2 = 0;
                    T_a = 2*x/(v_1+v_0);
                    T_j1 = (j_max*x - Math.sqrt(j_max*(j_max*Math.pow(x, 2) - Math.pow((v_1 + v_0), 2)*(v_1-v_0)))) / (j_max*(v_1+v_0));
                }
            }
        }

        // Now all durations have been determined, so maximum/minimum accelerations and maximum velocity can be found.
        a_lim_a = j_max*T_j1;
        a_lim_d = -1*j_max*T_j2;
        v_lim = v_0 + (T_a-T_j1)*a_lim_a;

        // Calculate total duration.
        T = T_a + T_v + T_d;

        // Uing the durations found and kinematic equations, calculate x, v, a, j for each period dt in the trajectory (3.30).
        // Segment a: acceleration with positive jerk.
        while(t < T_j1) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = j_max;
            temp.a = j_max*t;
            temp.v = v_0 + j_max*Math.pow(t, 2)/2;
            temp.x = x_0 + v_0*t + j_max*Math.pow(t, 3)/6;

            trajectory.add(temp);
            t += dt;
        }
        // Segment b: constant acceleration.
        while(t < T_a-T_j1) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = a_lim_a;
            temp.v = v_0 + a_lim_a*(t - T_j1/2);
            temp.x = x_0 + v_0*t + a_lim_a/6 * (3*Math.pow(t, 2) - 3*T_j1*t + Math.pow(T_j1, 2));

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment c: acceleration with negative jerk.
        while(t < T_a) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = -1*j_max;
            temp.a = j_max*(T_a-t);
            temp.v = v_lim - j_max*Math.pow((T_a - t), 2)/2;
            temp.x = x_0 + (v_lim + v_0)*T_a/2 - v_lim*(T_a-t) + j_max*Math.pow((T_a-t), 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment d: constant velocity.
        while(t < T_a+T_v) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = 0;
            temp.v = v_lim;
            temp.x = x_0 + (v_lim + v_0)*T_a/2 + v_lim*(t-T_a);

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment e: deceleration with negative jerk.
        while(t < T-T_d+T_j2) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = -1*j_max;
            temp.a = -1*j_max*(t-T+T_d);
            temp.v = v_lim - j_max*Math.pow((t-T+T_d), 2)/2;
            temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) - j_max*Math.pow((t-T+T_d), 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment f: constant deceleration.
        while(t < T-T_j2) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = 0;
            temp.a = a_lim_d;
            temp.v = v_lim + a_lim_d*(t-T+T_d-T_j2/2);
            temp.x = x_1 - (v_lim + v_1)*T_d/2 + v_lim*(t-T+T_d) + a_lim_d/6 * (3*Math.pow(t-T+T_d, 2) - 3*T_j2*(t-T+T_d) + Math.pow(T_j2, 2));

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }
        // Segment g: deceleration with positive jerk
        while(t < T) {
            TrajectoryElement temp = new TrajectoryElement();
            temp.t = t;
            temp.j = j_max;
            temp.a = -1*j_max*(T-t);
            temp.v = v_1 + j_max*Math.pow(T-t, 2)/2;
            temp.x = x_1 - v_1*(T-t) - j_max*Math.pow(T-t, 3)/6;

            temp.j *= reversed;
            temp.a *= reversed;
            temp.v *= reversed;
            temp.x *= reversed;

            trajectory.add(temp);
            t += dt;
        }

        // Ensure that the last state is reached even if it goes slightly beyond T.
        TrajectoryElement last = new TrajectoryElement();
        last.t = t;
        last.j = 0;
        last.a = 0;
        last.v = v_1;
        last.x = x_1;

        last.j *= reversed;
        last.a *= reversed;
        last.v *= reversed;
        last.x *= reversed;
        
        trajectory.add(last);


        return trajectory;
    }

    /**
      * Set a straight line s-curve trajectory to follow.
      * 
      * Follows a 7-segment trajectory.
      * Ensure that the dt for the trajectory is the same as the frequency this will run at.
      *
      * @param traj the trajectory to be followed.
      */
    public void setTrajectory(ArrayList<TrajectoryElement> traj) {
        trajectory = traj;
        counter = 0;
    }

    /**
      * Follow a straight line s-curve trajectory generated by calculateStraight.
      * 
      * Follows a 7-segment trajectory.
      * Ensure that the dt for the trajectory is the same as the frequency this will run at.
      *
      * @return boolean whether the follower is finished.
      */
    public boolean followTrajectoryStraight() {
        if(counter >= trajectory.size()) {
            return true;
        }

        Robot.drive.rightSpeed(trajectory.get(counter).v);
        Robot.drive.leftSpeed(trajectory.get(counter).v);
        counter++;
        return false;
    }

    /**
      * Follow a turning s-curve trajectory generated by calculateTurn.
      * 
      * Follows a 7-segment trajectory.
      * Ensure that the dt for the trajectory is the same as the frequency this will run at.
      *
      * @return boolean whether the follower is finished.
      */
      public boolean followTrajectoryTurn() {
        if(counter >= trajectory.size()) {
            return true;
        }

        Robot.drive.rightSpeed(trajectory.get(counter).v);
        Robot.drive.leftSpeed(-trajectory.get(counter).v);
        counter++;
        return false;
    }
}
