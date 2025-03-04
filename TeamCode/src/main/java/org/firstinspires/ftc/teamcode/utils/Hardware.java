package org.firstinspires.ftc.teamcode.utils;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {
	 public static Servo sch0;
	 private static boolean INIT = false;
	public static void init(HardwareMap hardware)
	{
		if (INIT) return;
		INIT = true;
		sch0 = hardware.get(Servo.class, "sch0");
	}

}
