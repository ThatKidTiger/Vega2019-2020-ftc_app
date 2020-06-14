package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class MecanumDriveAuto extends MecanumDrive{
	/*
	Motor Arrangement
			Front
			  ^
			  |
		0			3



		1			2
	 */

	public MecanumDriveAuto() {

	}

	/*
	Todo: Find out why the motor powers need to be reversed;
	 */
	public void forward(double power) {
		power = -power;
		double[] powers = {power, power, power, power};
		setMotorPowers(powers);
	}

	public void stop() {
		double[] powers = {0, 0, 0, 0};
		setMotorPowers(powers);
	}
}
