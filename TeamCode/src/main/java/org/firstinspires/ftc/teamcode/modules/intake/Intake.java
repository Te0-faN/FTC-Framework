package org.firstinspires.ftc.teamcode.modules.intake;

import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.utils.Hardware;

public class Intake {
	Servo roll;
	double angle = 0;

	public Intake()
	{
		roll = Hardware.sch0;
	}
	
	public void update()
	{
		roll.setPosition(angle);
	}

	public void adjustTo(double target)
	{
		if (target < 15)
			angle = 0;
		else if (target > 75 && target <= 90)
			angle = 90 / 300.0;
	}
}
