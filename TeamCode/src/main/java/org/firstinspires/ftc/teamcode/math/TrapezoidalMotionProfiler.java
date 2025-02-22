package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;
import static org.firstinspires.ftc.teamcode.Constants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Constants.MAX_ACC;

class TrapeziodalMotionProfiler {
    ElapsedTime timer = new ElapsedTime();
    boolean init = false;

    double[] calculate(double dist)
    {
        if (!init)
            timer.reset();
        // Calculate the time it takes to accelerate to max velocity
      double accel_dt = MAX_VEL / MAX_ACC;

      // If we can't accelerate to max velocity in the given dist, we'll accelerate as much as possible
      double accel_dist = 0.5 * MAX_ACC * accel_dt * accel_dt;

      if (accel_dist > (dist / 2))
        accel_dt = Math.sqrt((dist / 2) / (0.5 * MAX_ACC));

      accel_dist = 0.5 * MAX_ACC * accel_dt * accel_dt;

      // recalculate max velocity based on the time we have to accelerate and decelerate
      double max_vel = MAX_ACC * accel_dt;

      // we decelerate at the same rate as we accelerate
      double decel_dt = accel_dt;

      // calculate the time that we're at max velocity
        double cruise_dist = dist - 2 * accel_dist;
        double cruise_dt = cruise_dist / max_vel;
        double decel_time = accel_dt + cruise_dt;

      // check if we're still in the motion profile
      double entire_dt = accel_dt + cruise_dt + decel_dt;
      if (timer.seconds() > entire_dt) {
        return new double[] { dist };
      }

      // if we're accelerating
      if (timer.seconds() < accel_dt)
        // use the kinematic equation for accel
          return new double[]{ 0.5 * MAX_ACC * timer.seconds() * timer.seconds(), max_vel, MAX_ACC };

      // if we're cruising
      else if (timer.seconds() < decel_time) {
        accel_dist = 0.5 * MAX_ACC * accel_dt * accel_dt;
        double cruise_current_dt = timer.seconds() - accel_dt;

        // use the kinematic equation for constant velocity
          return new double[] {accel_dist + MAX_VEL * cruise_current_dt , max_vel, MAX_ACC};
      }

      // if we're decelerating
      else {
        accel_dist = 0.5 * MAX_ACC * accel_dt * accel_dt;
        cruise_dist = max_vel * cruise_dt;
        decel_time = timer.seconds() - decel_time;

        // use the kinematic equations to calculate the instantaneous desired position
          return new double[] { accel_dist + cruise_dist + max_vel * decel_time - 0.5 * MAX_ACC * decel_time * decel_time, max_vel, MAX_ACC };
      }
    }
}
