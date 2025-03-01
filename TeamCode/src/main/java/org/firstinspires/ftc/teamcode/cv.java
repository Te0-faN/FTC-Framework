package org.firstinspires.ftc.teamcode;

import static java.lang.Math.min;
import static java.lang.Math.sqrt;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@TeleOp(name="camera test")
public class cv extends LinearOpMode {
    @Override
    public void runOpMode() {
        ColorBlobLocatorProcessor processor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.RED)
                .setTargetColorRange(ColorRange.YELLOW)
//                .setRoi(ImageRegion.asUnityCenterCoordinates(-0.3, 0.80, 0.3, -0.80))
                .setRoi(ImageRegion.entireFrame())
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .build();
        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "w"))
                .setCameraResolution(new Size(320, 240))
                .addProcessor(processor)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //.enableLiveView(true) pentru a vedea pe driver hub
                .build();

        // face ca log urile de la temetrie sa apara si pe dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        // proiecteaza camera pe dashboard
        FtcDashboard.getInstance().startCameraStream(portal, 0);

        waitForStart();

        while (opModeIsActive()) {
            List<ColorBlobLocatorProcessor.Blob> blobs = processor.getBlobs();

            //ColorBlobLocatorProcessor.Util.filterByArea(1000, 10000 , blobs);
            telemetry.addData("FPS: ", portal.getFps());
            for (ColorBlobLocatorProcessor.Blob b : blobs) {
                RotatedRect box_fit = b.getBoxFit();
                telemetry.addLine(String.format("Blobs\nArea: %5d\nDensity: %4.2f\nAspect Ratio: %5.2f\nbox center: (%3d, %3d)\nh, w: (%.2f, %.2f)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int)box_fit.center.x, (int)box_fit.center.y, box_fit.size.height, box_fit.size.width));
            }

            telemetry.update();
            idle();
        }

        portal.close();
    }

    enum Col {RED, BLUE, YELLOW, IGNORE} ;

    public Col GetColor(int rgb)
    {
        int r = (rgb >> 16) & 0xff;
        int g = (rgb >> 8) & 0xff;
        int b = rgb & 0xff;
        if (r + g + b < 50) return Col.IGNORE;
        // eventual poti compara direct patratul distantelor fara sqrt
        double red = sqrt((0xff - r) * (0xff - r) + g*g + b*b);
        double blue = sqrt(r*r + g*g + (0xff - b) * (0xff - b));
        double yellow = sqrt((0xff - r) * (0xff - r) + (0xff - g) * (0xff - g) + b*b);
        double closest = min(red, min(blue, yellow));

        if (closest == red) return Col.RED;
        if (closest == blue) return Col.BLUE;
        else return Col.YELLOW;
    }
}