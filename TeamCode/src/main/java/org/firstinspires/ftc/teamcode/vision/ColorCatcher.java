package org.firstinspires.ftc.teamcode.vision;

import static java.lang.Math.min;
import static java.lang.Math.round;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.vision.opencv.ColorSpace;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;
import org.opencv.core.Scalar;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class ColorCatcher {
	PredominantColorProcessor processor;

	VisionPortal portal;

	Telemetry t;

	public static final int cam_width = 320;
	public static final int cam_height = 240;

	public static volatile int bY0 =  16, bCr0 =   0, bCb0 = 155;
	public static volatile int bY1 = 255, bCr1 = 127, bCb1 = 255;

	public static ColorRange blue = new ColorRange(
			ColorSpace.YCrCb,
			new Scalar(bY0, bCr0, bCb0),
			new Scalar(bY1, bCr1, bCb1)
	);

	public static volatile int rY0 =  70, rCr0 = 190, rCb0 = 100;
	public static volatile int rY1 = 255, rCr1 = 255, rCb1 = 132;

	public static ColorRange red = new ColorRange(
			ColorSpace.YCrCb,
			new Scalar(rY0, rCr0, rCb0),
			new Scalar(rY1, rCr1, rCb1)
	);

	public static volatile int yY0 = 135, yCr0 = 170, yCb0 =  80;
	public static volatile int yY1 = 170, yCr1 = 170, yCb1 =  80;

	public static volatile ColorRange yellow = new ColorRange(
			ColorSpace.YCrCb,
			new Scalar(rY0, rCr0, yCb0),
			new Scalar(rY1, rCr1, yCb1)
	);

	static int col[] = new int[3];

	public ColorCatcher(HardwareMap hardware, Telemetry telemetry)
	{
		t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
		processor = new PredominantColorProcessor.Builder()
				.setRoi(ImageRegion.asUnityCenterCoordinates(-0.1, 0.1, 0.1, -0.1))
				.setSwatches(
						PredominantColorProcessor.Swatch.RED,
						PredominantColorProcessor.Swatch.YELLOW
				)
				.build();
		portal = new VisionPortal.Builder()
				.setCamera(hardware.get(WebcamName.class, "w"))
				.setCameraResolution(new Size(cam_width, cam_height))
				.addProcessor(processor)
				.setStreamFormat(VisionPortal.StreamFormat.MJPEG)
				.build();
		while (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
			t.addLine("Initializing camera....");
			t.update();
		}
	}

	public void update()
	{
		int rgb = processor.getAnalysis().rgb;
		col = rgb2YCrCb(Color.red(rgb), Color.green(rgb), Color.blue(rgb));
		t.addData("Culoare: ", processor.getAnalysis().closestSwatch);
		t.addLine(String.format("RGB:   (%3d, %3d, %3d)", Color.red(rgb), Color.green(rgb), Color.blue(rgb)));
		t.addLine(String.format("YCrCb: (%3d, %3d, %3d)", col[0], col[1], col[2]));
		t.update();
	}

	enum Col {IGNORE, RED, BLUE, YELLOW }

	public Col GetColor(int rgb)
	{
		int r = (rgb >> 16) & 0xff;
		int g = (rgb >> 8) & 0xff;
		int b = rgb & 0xff;

		// ignor culorile foarte deschise sau inchise
		int s = r + g + b;
		if (s < 50 || s > 600) return Col.IGNORE;

		double red = (0xff - r) * (0xff - r) + g*g + b*b;
		double blue = r*r + g*g + (0xff - b) * (0xff - b);
		double yellow = (0xff - r) * (0xff - r) + (0xff - g) * (0xff - g) + b*b;
		double closest = min(red, min(blue, yellow));

		if (closest == red) return Col.RED;
		if (closest == blue) return Col.BLUE;
		else return Col.YELLOW;
	}

	public static int[] rgb2YCrCb(int r, int g, int b)
	{
		float Y = 0.299f * r + 0.587f * g + 0.114f * b;
		float Cr = (r - Y) * 0.713f + 128f;
		float Cb = (b - Y) * 0.564f + 128f;

		return new int[]{ round(Y), round(Cr), round(Cb) };
	}
}
