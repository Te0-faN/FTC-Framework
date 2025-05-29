package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

 @TeleOp(name = "Pivoter Servos", group = "Test")
public class PivoterServos extends LinearOpMode {

     final double CLAW_OPEN = 0.015;
     final double CLAW_CLOSED = 0.35;
     Servo claw;
     Servo pitch;
     /* 0.0 -> poz outtake */
     /* 0.4 -> poz intake  */
     boolean claw_open;
     ElapsedTime timer = new ElapsedTime();

     public void telemetryLog()
    {
        telemetry.addLine("patrat - cleste");
        telemetry.addData("pozitie cleste: ", claw.getPosition());
        telemetry.addLine("pitch  - dpad");
        telemetry.addData("pozitie pitch: ", pitch.getPosition());
        telemetry.update();
    }

    public void runOpMode()
    {
        claw = hardwareMap.servo.get("mip");
        pitch = hardwareMap.servo.get("milici");

        claw.setPosition(CLAW_OPEN);
        claw_open = true;

        pitch.setPosition(0.0);

        telemetryLog();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                pitch.setPosition(pitch.getPosition() + 0.1);
            } else if (gamepad1.dpad_down) {
                pitch.setPosition(pitch.getPosition() - 0.1);
            }
            if (timer.seconds() > 0.5 && gamepad1.square) {
                claw.setPosition(claw_open ? CLAW_CLOSED : CLAW_OPEN);
                claw_open = !claw_open;
                timer.reset();
            }

            telemetryLog();
            sleep(1000);
        }

    }
}