package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


 @TeleOp(name = "Intake Pitch", group = "Test")
public class IntakePitch extends LinearOpMode {

    public void runOpMode()
    {
        final double UP = 0.65;
        final double DOWN = 0.775;

        Servo l, r;
        l = hardwareMap.servo.get("pitch");
        r = hardwareMap.servo.get("rpitch");
        l.setDirection(Servo.Direction.FORWARD);
        r.setDirection(Servo.Direction.REVERSE);

        l.setPosition(DOWN);
        r.setPosition(DOWN);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                l.setPosition(l.getPosition() - 0.1);
                r.setPosition(r.getPosition() - 0.1);
            } else if (gamepad1.dpad_down) {
                l.setPosition(l.getPosition() + 0.1);
                r.setPosition(r.getPosition() + 0.1);
            } else if (gamepad1.dpad_right) {
                l.setPosition(l.getPosition() - 0.05);
                r.setPosition(r.getPosition() - 0.05);
            } else if (gamepad1.dpad_left) {
                l.setPosition(l.getPosition() + 0.05);
                r.setPosition(r.getPosition() + 0.05);
            }

            telemetry.addData("Left: ", l.getPosition());
            telemetry.addData("Right: ", r.getPosition());
            telemetry.update();

            sleep(250);
        }

    }
}