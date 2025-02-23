package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

import static org.firstinspires.ftc.teamcode.utils.Constants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.utils.Constants.MAX_ACC;

class TrapesiodalMotionProfiler {
    ElapsedTime timer = new ElapsedTime();
    boolean init = false;

    double[] calculate(double dist)
    {
        if (!init) {
            timer.reset();
            init = true;
        }
        double t = timer.seconds();
        // timpul pentru accelerare la vmax
        double deltat_amax = MAX_VEL / MAX_ACC;
        // x(t) = x0 + v0(t-t0) + a(t-t0)^2/2
        // v(t) = v0 + a(t-t0)
        double dist_amax  = 0.5 * MAX_ACC * deltat_amax * deltat_amax;
        double v_max = MAX_VEL;


        // verificam daca avem timp sa ajungem ca viteza maxima
        if (dist_amax > 0.5*dist) {
            deltat_amax = Math.sqrt(dist / MAX_ACC);
            dist_amax = 0.5 * MAX_ACC * deltat_amax * deltat_amax;
            v_max = MAX_ACC * deltat_amax;
        }

        double dist_cruise = dist - 2 * dist_amax;
        double deltat_cruise = dist_cruise / v_max;

        // verificam daca inca suntem in motion profiler
        double deltat_tot = 2 * deltat_amax + deltat_cruise;
        if (t >= deltat_tot)
            return new double[] { dist };
        // accelerare
        if (t < deltat_amax) {
             return new double[] { 0.5 * MAX_ACC * t * t, MAX_ACC * t, MAX_ACC };
        // viteza maxima constanta
        } else if (t < deltat_amax + deltat_cruise) {
            return new double[] { dist_amax + v_max * (t - deltat_amax), v_max, 0 };
        // decelerare
        } else {
            double deltat_decel = t - deltat_amax - deltat_cruise;
            return new double[] {
                    dist_amax + v_max * deltat_cruise + v_max * deltat_decel - 0.5 * MAX_ACC * deltat_decel * deltat_decel,
                    v_max - MAX_ACC * deltat_decel,
                   -MAX_ACC
            };
        }

    }
}
