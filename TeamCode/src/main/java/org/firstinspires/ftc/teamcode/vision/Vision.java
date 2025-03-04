package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

import static java.lang.Math.abs;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Vision {
	ColorBlobLocatorProcessor processor;
	List<ColorBlobLocatorProcessor.Blob> blobs;

	public VisionPortal portal;

	public static final int cam_width = 320;
	public static final int cam_height = 240;

	public double sample_orientation, ratio;

	public Vision(HardwareMap hardware, Telemetry telemetry)
	{
		processor = new ColorBlobLocatorProcessor.Builder()
				.setTargetColorRange(ColorRange.RED)
				.setTargetColorRange(ColorRange.YELLOW)
				.setRoi(ImageRegion.entireFrame())
				.setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
				.setDrawContours(true)
				.setBlurSize(5)
				.build();
		portal = new VisionPortal.Builder()
				.setCamera(hardware.get(WebcamName.class, "w"))
				.setCameraResolution(new Size(cam_width, cam_height))
				.addProcessor(processor)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.build();
		while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
			telemetry.addLine("Initializing camera....");
			telemetry.update();
		}
	}

	public void update()
	{
		blobs = processor.getBlobs();
		ColorBlobLocatorProcessor.Util.filterByArea(500, 20000, blobs);
		if (blobs.size() == 1) {
			ColorBlobLocatorProcessor.Blob b = blobs.get(0);
			ratio = b.getAspectRatio();
			sample_orientation = b.getBoxFit().angle;
			if (abs(sample_orientation) < 15 || abs(sample_orientation) > 65) {
				sample_orientation = (b.getAspectRatio() < 2.1) ? 0 : 90;
			}
		} else
			sample_orientation = 0;
	}
}
