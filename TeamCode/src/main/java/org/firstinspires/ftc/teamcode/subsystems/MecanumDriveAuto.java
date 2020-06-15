package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import static org.firstinspires.ftc.teamcode.hardware.BotConstants.IMU_CONSTANTS.ROT_PID;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.stormbots.MiniPID;

import java.util.Map;

/*
Todo: separate imu functionality from MecanumDrive class, import imu methods into the IMU class
 */

public class MecanumDriveAuto extends MecanumDrive {
	private static final String TAG = "MecanumDriveAuto";
	/*
	Motor Arrangement
			Front
			  ^
			  |
		0			3



		1			2
	 */

	public MecanumDriveAuto() {
		MiniPID imuPID = new MiniPID(ROT_PID.p, ROT_PID.i, ROT_PID.d);
	}

	/*
	Todo: Find out why the motor powers need to be reversed;
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

	public void rotate(double power) {
		if(power <= 1) {
			Log.d(TAG, String.format("Rotating " + (power < 0 ? "counterclockwise" : "clockwise") + " at power %f.2", Math.abs(power)));
			power = - power;
			double[] powers = {power, power, - power, - power};
			setMotorPowers(powers);
		} else {

		}
	}

	public void stop() {
		double[] powers = {0, 0, 0, 0};
		setMotorPowers(powers);
	}
}
