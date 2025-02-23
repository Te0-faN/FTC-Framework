package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.Constants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.Constants.MAX_ACC;

class TrapeziodalMotionProfiler {
    ElapsedTime timer = new ElapsedTime();
    boolean init = false;

    double[] calculate(double dist)
    {
        if (!init) {
            timer.reset();
            init = true;
        }
        double t = timer.seconds();
        double deltat_amax = MAX_VEL / MAX_ACC;
        double dist_amax  = 0.5 * MAX_ACC * deltat_amax * deltat_amax;
        double v_max = MAX_VEL;

        if (dist_amax > 0.5*dist) {
            deltat_amax = Math.sqrt(dist / MAX_ACC);
            dist_amax = 0.5 * MAX_ACC * deltat_amax * deltat_amax;
            v_max = MAX_ACC * deltat_amax;
        }

        double dist_cruize = dist - 2 * dist_amax;
        double deltat_cruize = dist_cruize / v_max;

        double deltat_tot = 2 * deltat_amax + deltat_cruize;
        if (t >= deltat_tot)
            return new double[] { dist };

        if (t < deltat_amax) {
             return new double[] { 0.5 * MAX_ACC * t * t, MAX_ACC * t, MAX_ACC };
        } else if (t < deltat_amax + deltat_cruize) {
            return new double[] { dist_amax + v_max * (t - deltat_amax), v_max, 0 };
        } else {
            double deltat_decel = t - deltat_amax - deltat_cruize;
            return new double[] {
                    dist_amax + v_max * deltat_cruize + v_max * deltat_decel - 0.5 * MAX_ACC * deltat_decel * deltat_decel,
                    v_max - MAX_ACC * deltat_decel,
                   -MAX_ACC
            };
        }

    }
}
