package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Intake Pitch", group = "Test")
public class IntakePitch extends LinearOpMode {

    static class Pitch {
        public Servo l;
        public Servo r;
    }
    Pitch pitch;

    public void runOpMode()
    {
        pitch.l = hardwareMap.servo.get("pitch");
        pitch.r = hardwareMap.servo.get("rpitch");

		pitch.l.setPosition(0.5);
		pitch.r.setPosition(0.5);

        waitForStart();

        while (opModeIsActive()) {
            Gamepad g = new Gamepad();
            g.copy(gamepad1);

             if (g.dpad_up) {
                pitch.l.setPosition(pitch.l.getPosition() + 0.1);
                pitch.r.setPosition(pitch.r.getPosition() + 0.1);
            } else if (g.dpad_down) {
                pitch.l.setPosition(pitch.l.getPosition() - 0.1);
                pitch.r.setPosition(pitch.r.getPosition() - 0.1);
            }
        }

    }
}