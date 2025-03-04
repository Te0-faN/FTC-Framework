package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.intake.Intake;
import org.firstinspires.ftc.teamcode.utils.Hardware;
import org.firstinspires.ftc.teamcode.vision.Vision;

@TeleOp(name="Yaw Test")
public class YawTest extends LinearOpMode {
	@Override
	public void runOpMode()
	{
		Hardware.init(hardwareMap);
		Intake intake = new Intake();

		Vision v = new Vision(hardwareMap, telemetry);

		// face ca log urile de la temetrie sa apara si pe dashboard
		 telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		// proiecteaza camera pe dashboard
		FtcDashboard.getInstance().startCameraStream(v.portal, 0);

		waitForStart();

		while (opModeIsActive()) {
			intake.adjustTo(v.sample_orientation);

			telemetry.addData("Unghi: ", v.sample_orientation);
			telemetry.addData("Raport: ", v.ratio);

			v.update();
			intake.update();
			telemetry.update();
			idle();
		}
	}
}
