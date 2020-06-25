package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stormbots.MiniPID;

import java.util.Map;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_MIN;
import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;

public class MecanumDriveAuto extends Subsystem {
	private static final String TAG = "MecanumDriveAuto";
	private IMU imu = new IMU();

	MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
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

	//names to search the hardware map for
	private String[] motorNames = {"frontLeft", "backLeft", "backRight", "frontRight"};

	@Override
	public Map<String, Object> update() {
		return null;
	}

	public MecanumDriveAuto() {

	}

	/*
	Todo: Find out why the motor powers need to be reversed
	Todo: Replace telemetry packets with the update command
	Todo: Implement rotateToOrientation command
	 */
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

	public void init(HardwareMap hwMap) {
		for(int i = 0; i < 4; i++) {
			motors[i] = hwMap.get(DcMotor.class, motorNames[i]);
		}

		motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
		motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
		imu.init(hwMap);
		Log.d(TAG, "Initialization Complete");
	}

	public void setMotorPowers(double[] powers) {
		for(int i = 0; i < motors.length; i++) {
			motors[i].setPower(powers[i]);
		}
	}

	public void rotateByPower(double power) {
		if(power <= 1) {
			Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
			power = - power;
			double[] powers = {power, power, - power, - power};
			setMotorPowers(powers);
		} else {

		}
	}

	//uses PID controller to precisely turn a certain number of degrees
	private void rotateByAngle(double degrees) {
		double leftPower, rightPower, temppower;

		imu.resetAngularDistance();
		sleep(250);

		double lastangle = 0;
		double angle = imu.getAngularDistance();

		//rotates until the imu returns that the robot is within a margin of error
		while(Math.abs(degrees - angle) > 0.1 && Math.abs(degrees - lastangle) > 0.1) {
			lastangle = angle;
			angle = imu.getAngularDistance();
			temppower = imuPID.getOutput(angle, degrees);
			//packet.put("Orientation", angle);
			//insert angular distance travelled and offset from goal packet

			if(Math.abs(temppower) < ROT_MIN) {
				temppower *= ROT_MIN/Math.abs(temppower);
			}

			if(angle == 0) {
				temppower = Math.signum(degrees) * ROT_MIN;
			}

			leftPower = temppower;
			rightPower = -temppower;

			double[] powers = {leftPower, leftPower, rightPower, rightPower};
			//Insert motor power telemetry packet
			setMotorPowers(powers);
		}
		stop();
	}

	public void stop() {
		double[] powers = {0, 0, 0, 0};
		setMotorPowers(powers);
	}
}
