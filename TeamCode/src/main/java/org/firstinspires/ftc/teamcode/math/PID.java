package org.firstinspires.ftc.teamcode.math;

import com.qualcomm.robotcore.util.ElapsedTime;

class PID {
    private final double kp, ki, kd, kf;
    private final ElapsedTime timer = new ElapsedTime();

    private double prev_error;
    private double prev_reference = 0;
    private double integral;
    public double max_integral;
    private boolean init = false;

    PID(double kp, double ki, double kd)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = 0;

        this.max_integral = 1.0;
    }

    PID(double kp, double ki, double kd, double kf)
    {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.kf = kf;

        this.max_integral = 1.0;
    }

    double calculate(double reference, double state)
    {
        double error = reference - state;

        if (!init) {
            init = true;
            timer.reset();
        }
        double deltat = timer.seconds();

        if (reference != prev_reference)
            integral = 0;

        integral += (error * deltat);

        if (integral > max_integral)
            integral = max_integral;

        if (integral < -max_integral)
            integral = -max_integral;

        double derivative = (error - prev_error) / deltat;

        prev_reference = reference;
        prev_error = error;

        timer.reset();
        return kp * error + ki * integral + kd * derivative + kf;
    }
}
