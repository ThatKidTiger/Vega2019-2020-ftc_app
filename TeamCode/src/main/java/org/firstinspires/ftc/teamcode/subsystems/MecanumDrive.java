package org.firstinspires.ftc.teamcode.subsystems;

import static java.lang.Math.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
	//hardware object references
	private DcMotor[] motors = new DcMotor[4];
	private BNO055IMU imu;

	//names to search the hardware map for
	private String[] motorNames = {"frontLeft", "backLeft", "backRight", "frontRight"};
	private String imuName = "imu";

	/*
	Todo: Document
	Todo: Read mecanum kinematic analysis
	 */
	@Override
	public Map<String, Object> update() {
		return null;
	}

	public MecanumDrive() {

	}

	public void init(HardwareMap hwMap) {
		for(int i = 0; i < 4; i++) {
			motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
		}

		motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

		imu = hwMap.get(BNO055IMU.class, imuName);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled = false;
		parameters.calibrationDataFile = "IMUCalibration.json";

		imu.initialize(parameters);
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
