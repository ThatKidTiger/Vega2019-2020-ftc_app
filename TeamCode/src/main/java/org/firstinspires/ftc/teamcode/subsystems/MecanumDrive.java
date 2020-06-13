package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class MecanumDrive extends Subsystem{
	/*
	Motor Arrangement
			Front
			  ^
			  |
		0			3



		1			2
	 */
	private DcMotor[] motors = new DcMotor[4];
	private String[] motorNames = {"frontLeft, backLeft, backRight, frontRight"};

	/*
	Todo: Document
	Todo: Read mecanum kinematic analysis
	 */
	@Override
	public Map<String, Object> update() {
		return null;
	}

	public MecanumDrive(HardwareMap map) {
		for(int i = 0; i < 4; i++) {
			motors[i] = map.get(DcMotor.class, motorNames[i]);
		}
	}

	public void setMotorPowers(double[] powers) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].setPower(powers[i]);
		}
	}

	//return to this after understanding kinematic analysis
	/*converting unit circle gamepad input into polar coordinates to power + direction
	r = (sqrt(arccos(x)^2 + arcsin(y)^2), theta = arcsin(y)
	public void drivewithGamepad(double x, double y) {
		double r = sqrt(pow(acos(x), 2) + pow(asin(y),2 ));
		double theta = asin(y);
	}

	public void drivewithVelocity() {

	}
	*/
}
