package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Regionala 2025")
public class TeleOpMain extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    DcMotor intake_slider;
    DcMotor outtake_slider;
    DcMotor pivoter;

    Servo pitch;
    Servo Rpitch;
    Servo milici;
    Servo mip;
    CRServo roata;
    CRServo Rroata;
    ElapsedTime elapsed_mip = new ElapsedTime();
    ElapsedTime elapsed_roata= new ElapsedTime();

    boolean mip_open = true;
    boolean specimene=false;

    final double TICKS_435 = 384.5;
    double MAX_LENGTH = TICKS_435 * 4-150;
    final double TICKS_30 = 5281.1;
    final double TICKS_312 = 537.7;
    final double MAX_ISLIDER = 3 * TICKS_435;
    final double MAX_PIVOTER = 0.3 * TICKS_312;
    boolean grip_open = true;
    boolean up   = true;
    double pozyaw = 0;
    public ColorSensor colorSensor;
    TouchSensor touch;
    TouchSensor touch2;
    ElapsedTime Ereset= new ElapsedTime();

    private void InitWheels() {
        front_left = hardwareMap.dcMotor.get("frontLeftMotor");
        front_right = hardwareMap.dcMotor.get("frontRightMotor");
        back_left = hardwareMap.dcMotor.get("backLeftMotor");
        back_right = hardwareMap.dcMotor.get("backRightMotor");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void InitDcs() {
        intake_slider  = hardwareMap.dcMotor.get("intake-slide");
        outtake_slider = hardwareMap.dcMotor.get("outtake-slide");
        pivoter        = hardwareMap.dcMotor.get("pivoter");
        touch          = hardwareMap.touchSensor.get("limit-intake");
        touch2         = hardwareMap.touchSensor.get("limit-pivoter");

        intake_slider.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivoter.setDirection(DcMotorSimple.Direction.REVERSE);
        intake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake_slider.setTargetPosition(50);
        intake_slider.setPower(1);
        intake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void InitServos() {
        pitch = hardwareMap.servo.get("pitch");
        Rpitch = hardwareMap.servo.get("rpitch");
        roata = hardwareMap.crservo.get("roata");
        Rroata = hardwareMap.crservo.get("rroata");

        milici  = hardwareMap.servo.get("milici");
        mip     = hardwareMap.servo.get("mip");

        Rpitch.setDirection(Servo.Direction.REVERSE);
        Rroata.setDirection(DcMotorSimple.Direction.REVERSE);

        Rpitch.setPosition(0.0);
        pitch.setPosition(0.0);
    }

    private void SetWheelsPower()
    {
        double left_x  =  gamepad1.left_stick_x;
        double left_y  = -gamepad1.left_stick_y;
        double right_x =  gamepad1.right_stick_x;

        double normalizer = Math.max(Math.abs(left_x) + Math.abs(left_y) + Math.abs(right_x), 1.0);

        double front_left_pw  = (left_y + left_x + right_x) / normalizer;
        double back_left_pw   = (left_y - left_x + right_x) / normalizer;
        double front_right_pw = (left_y - left_x - right_x) / normalizer;
        double back_right_pw  = (left_y + left_x - right_x) / normalizer;

        front_left.setPower(front_left_pw);
        back_left.setPower(back_left_pw);
        front_right.setPower(front_right_pw);
        back_right.setPower(back_right_pw);
    }

    private void MoveDcs() {
        double left_y = -gamepad2.left_stick_y;

        if (Math.abs(left_y) > 0.1) {
            double target = intake_slider.getCurrentPosition() + (int) (left_y * TICKS_435);
            if (target >= MAX_LENGTH)
                target = MAX_LENGTH;
            if (target <= 42)
                target = 42;
            intake_slider.setTargetPosition((int) target);
            intake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake_slider.setPower(0.4 * left_y);
        }
    }

    private void MoveServos() {
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        int total = red + green + blue;
        boolean isRedDominant = red > blue && red > total / 2;
        boolean isBlueDominant = blue > red && blue > total / 2;

        if (red > 50){
            elapsed_roata.reset();

            while (elapsed_roata.seconds() < 1 && opModeIsActive()) {
                roata.setPower(-1);
                Rroata.setPower(-1);
            }
            roata.setPower(0);
            Rroata.setPower(0);
        } else if (blue > 50) {
            elapsed_roata.reset();

            while (elapsed_roata.seconds() < 1 && opModeIsActive()) {
                roata.setPower(-1);
                Rroata.setPower(-1);
            }
            roata.setPower(0);
            Rroata.setPower(0);
        } else {
            if (gamepad2.dpad_right) {
                roata.setPower(1);
                Rroata.setPower(1);
            }

            if (gamepad2.dpad_left) {
                roata.setPower(-1);
                Rroata.setPower(-1);
            }

            if (gamepad2.dpad_up) {
                roata.setPower(0);
                Rroata.setPower(0);
            }
            if (gamepad2.circle)
            {
                roata.setPower(0);
                Rroata.setPower(0);
            }
        }
        telemetry.addData("Red", red);
        telemetry.addData("Blue", blue);

        if (gamepad2.square && elapsed_mip.seconds() > 0.5) {
            mip.setPosition(mip_open ? 0.0 : 0.4);
            mip_open = !mip_open;
            elapsed_mip.reset();
        }

    }

    public void pozitieInit() {
        milici.setPosition(0.60);
        mip.setPosition(0.4);
        pitch.setPosition(0.4);
        Rpitch.setPosition(0.4);
        pivoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivoter.setPower(-0.6);
        while (!touch2.isPressed());

        pivoter.setPower(0);
        pivoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void schimb() {
        if (gamepad2.circle) {
            outtake_slider.setTargetPosition(-500);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while (outtake_slider.getCurrentPosition()>-400 && opModeIsActive()) {}

            pitch.setPosition(0.33);
            Rpitch.setPosition(0.33);

            pivoter.setTargetPosition(27);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivoter.setPower(0.8);

            milici.setPosition(0.33);
            mip.setPosition(0.23);
            while (pivoter.isBusy() && opModeIsActive()) {}

            intake_slider.setTargetPosition(20);
            intake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake_slider.setPower(1);

            while (intake_slider.isBusy() && opModeIsActive()) {}

            outtake_slider.setTargetPosition(-47);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            roata.setPower(-1);
            Rroata.setPower(-1);
            while (outtake_slider.isBusy() && opModeIsActive()) {}

            ElapsedTime elapsed_mip = new ElapsedTime();

            mip.setPosition(0.35);
            mip_open = !mip_open;
            elapsed_mip.reset();
            roata.setPower(0.0);
            Rroata.setPower(0.0);

            while (elapsed_mip.seconds() < 0.3 && opModeIsActive()) {}



            outtake_slider.setTargetPosition(-500);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (outtake_slider.isBusy() && opModeIsActive()) {}


        }
    }

    private void pitchMovement() {
        if (gamepad2.dpad_up) {
            pitch.setPosition(0.33);
            Rpitch.setPosition(0.33);
        }
        if (gamepad2.dpad_down) {
            pitch.setPosition(0.43);
            Rpitch.setPosition(0.43);
        }
    }

    public void reset() {
        if (gamepad2.triangle   && specimene==false) {
            outtake_slider.setTargetPosition(-500);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            milici.setPosition(0.33);
            mip.setPosition(0.23);
            pitch.setPosition(0.33);
            Rpitch.setPosition(0.33);

            //  while (outtake_slider.isBusy() && opModeIsActive()) {}

            intake_slider.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake_slider.setPower(-0.5);
            while (!touch.isPressed() && opModeIsActive());

            intake_slider.setPower(0);
            intake_slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake_slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            outtake_slider.setTargetPosition(-500);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivoter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            pivoter.setPower(-0.8);
            while (!touch2.isPressed() && opModeIsActive());
            pivoter.setPower(0);

            elapsed_mip.reset();

            while (elapsed_mip.seconds() < 0.1 && opModeIsActive()) {}

            pivoter.setPower(-0.2);
            while (!touch2.isPressed());

            pivoter.setPower(0);
            pivoter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivoter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
    }
    private void score() {
        if (gamepad2.cross && specimene==false) {
            outtake_slider.setTargetPosition(-2200);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivoter.setTargetPosition(1600);
            pivoter.setPower(1);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            milici.setPosition(0.2);
        }
    }


    private Thread wheelsThread = new Thread(() -> {
        while (!isStopRequested() && opModeIsActive()) {
            SetWheelsPower();
        }
    });

    private Thread armsThread = new Thread(() -> {
        while (!isStopRequested() && opModeIsActive()) {
            MoveServos();
            MoveDcs();
            score();
            schimb();
            pitchMovement();
            scorespecimene();
        }
    });

    private Thread resetT = new Thread(() -> {
        while (!isStopRequested() && opModeIsActive()) {
            reset();
        }
    });

    public void runOpMode() {
        InitWheels();
        InitServos();
        InitDcs();
        pozitieInit();

        waitForStart();

        if (isStopRequested()) return;

        wheelsThread.start();
        armsThread.start();
        resetT.start();

        while (opModeIsActive()) {
            Telemetrie();
        }

        try {
            wheelsThread.join();
            armsThread.join();
            resetT.join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void Telemetrie() {
//        telemetry.addData("pozitie pitch", pitch.getPosition());
//        telemetry.addData("glisiera intake", intake_slider.getCurrentPosition());
//        //telemetry.addData("glisiera intake pow", intake_slider.getPower());
//        telemetry.addData("glisiera outtake", outtake_slider.getCurrentPosition());
//        telemetry.addData("pivoter", pivoter.getCurrentPosition());
//
//        telemetry.addData("milici", milici.getPosition());
//        telemetry.addData("mip", mip.getPosition());
//        telemetry.addData("Buton : ", touch2.isPressed());
//
//        telemetry.addData("atins",touch.isPressed());

        telemetry.update();
    }

    public void scorespecimene()
    {
//        if(gamepad2.circle && specimene==true)
        if (gamepad2.left_bumper)
        {
            pivoter.setTargetPosition(1100);//1050
            pivoter.setPower(1);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            milici.setPosition(0.53);

            outtake_slider.setTargetPosition(0);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.right_bumper)
        {
            outtake_slider.setTargetPosition(-490);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elapsed_mip.reset();

            while (elapsed_mip.seconds() < 0.8 && opModeIsActive())
            	;
            mip.setPosition(0.1);
            mip_open=true;
        }

        if (gamepad1.left_bumper)
        {
            outtake_slider.setTargetPosition(-200);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if (gamepad1.right_bumper)
        {
            outtake_slider.setTargetPosition(0);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (gamepad2.ps)
        {
            outtake_slider.setTargetPosition(0);
            outtake_slider.setPower(1);
            outtake_slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            pivoter.setTargetPosition(2190);
            pivoter.setPower(1);
            pivoter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            milici.setPosition(0.43);

            while(pivoter.isBusy()&&opModeIsActive()){}

            pitch.setPosition(0);
            Rpitch.setPosition(0);
        }
    }
}
//27 piv -44 outtake