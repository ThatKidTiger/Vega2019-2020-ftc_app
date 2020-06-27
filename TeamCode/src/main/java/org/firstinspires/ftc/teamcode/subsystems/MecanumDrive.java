package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.stormbots.MiniPID;

import java.util.HashMap;
import java.util.Map;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MAX;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;

/*
Todo: Document
Todo: Read mecanum kinematic analysis
Todo: add the update command functionality for telemetry
Todo: Migrate master update() function (the one that sends the TelemetryPacket) to the Robot class
Todo: Initialize imu in the robot class and pass it to MecanumDrive during initialization
Todo: Find out why the motor powers need to be reversed
Todo: Replace telemetry packets with the update command
Todo: Implement rotateToOrientation command
Todo: pass the field canvas to this class in the constructor and have each individual move command update
 */

public class MecanumDrive extends Subsystem {
	private static final String TAG = "MecanumDrive";
	private IMU imu = new IMU();

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
	GamepadController controllers;

	//map to update telemetry stats to
	private HashMap<String, Object> updates = new HashMap<>();
	FtcDashboard dashboard = FtcDashboard.getInstance();

	//names to search the hardware map for
	private String[] motorNames = {"frontLeft", "backLeft", "backRight", "frontRight"};

	@Override
	public Map<String, Object> update() {
		return updates;
	}

	public MecanumDrive() {

	}

	public void init(HardwareMap hwMap) {
		for(int i = 0; i < 4; i++) {
			motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
		}

		motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
		Log.d(TAG, "Initialization Complete");
	}

	public void setMotorPowers(double[] powers) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].setPower(powers[i]);
		}

		updates.put("FL", powers[0]);
		updates.put("BL", powers[1]);
		updates.put("BR", powers[2]);
		updates.put("FR", powers[3]);
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

	//region Auton methods
	public void forward(double power) {
		Log.d(TAG, String.format("Moving " + (power > 0 ? "forward" : "backward") + " at power %f.2", Math.abs(power)));
		power = -power;
		double[] powers = {power, power, power, power};
		setMotorPowers(powers);
	}

	public void strafe(double power) {
		Log.d(TAG, String.format("Strafing " + (power > 0 ? "right" : "left") + " at power %f.2", Math.abs(power)));
		power = -power;
		double[] powers = {power, -power, power, -power};
		setMotorPowers(powers);
	}

	public void rotateByPower(double power) {
		Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
		power = -power;
		double[] powers = {power, power, - power, - power};
		setMotorPowers(powers);
	}

	//uses PID controller to precisely turn a certain number of degrees
	public void rotateByAngle(double degrees) {
		ElapsedTime runtime = new ElapsedTime();
		runtime.startTime();
		MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
		double leftPower, rightPower, temppower;

		imu.resetAngularDistance();
		sleep(250);

		double lastAngle = 0;
		double angle = imu.getAngularDistance();

		//rotates until the imu returns that the robot is within a margin of error
		while(Math.abs(degrees - angle) > 0.1 && Math.abs(degrees - lastAngle) > 0.1) {
			updates.put("Reference", degrees);
			lastAngle = angle;
			angle = imu.getAngularDistance();
			temppower = imuPID.getOutput(angle, degrees);

			if(Math.abs(temppower) < ROT_MIN) {
				temppower *= ROT_MIN/Math.abs(temppower);
			} else if(Math.abs(temppower) > ROT_MAX) {
				temppower *= ROT_MAX/Math.abs(temppower);
			}

			if(angle == 0) {
				temppower = Math.signum(degrees) * ROT_MIN;
			}

			leftPower = temppower;
			rightPower = -temppower;

			double[] powers = {leftPower, leftPower, rightPower, rightPower};
			setMotorPowers(powers);
		}
		stop();
		Log.d(TAG, "Time to Complete: " + runtime.milliseconds());
	}

	public void stop() {
		double[] powers = {0, 0, 0, 0};
		setMotorPowers(powers);
	}
	//endregion
}
