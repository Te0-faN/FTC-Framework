package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name="camera test")
public class cv extends LinearOpMode {
    @Override
    public void runOpMode()
    {
        BasicProcessor processor = new BasicProcessor();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "w"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(processor)
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                //.enableLiveView(true)
                .build();

        // face ca log urile de la temetrie sa apara si pe dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // proiecteaza camera pe dashboard
        FtcDashboard.getInstance().startCameraStream(portal, 0);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("FPS: ", portal.getFps());
            telemetry.update();
            idle();
        }

        portal.close();
    }
}
