package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.HashMap;
import java.util.Map;

public class IMU extends Subsystem {
	private static final String TAG = "MecanumDriveAuto";
	private BNO055IMU imu;

	private String imuName = "imu";
	private Orientation lastAngles;
	private double relativeAngle, globalAngle;

	//map to update telemetry stats to
	private HashMap<String, Object> updates = new HashMap<>();

	@Override
	public Map<String, Object> update() {
		return updates;
	}

	public IMU() {

	}

	public void init(HardwareMap hwMap) {
		imu = hwMap.get(BNO055IMU.class, imuName);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

		parameters.mode = BNO055IMU.SensorMode.IMU;
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
		parameters.loggingEnabled = false;
		parameters.calibrationDataFile = "IMUCalibration.json";

		imu.initialize(parameters);

		resetAngularDistance();
		resetOrientation();
		Log.d(TAG, "Initialization Complete");
	}

	public void resetAngularDistance() {
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		relativeAngle = 0;
	}

	public void resetOrientation() {
		lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
		globalAngle = lastAngles.firstAngle;
	}

	//Orientation is measured as an angular position relative to the globalAngle variable
	//"What direction is the robot facing?"
	public double getOrientation() {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double deltaAngle = angles.firstAngle - globalAngle;

		deltaAngle = limitAxes(deltaAngle);

		updates.put("Orientation", deltaAngle);
		return deltaAngle;
	}

	//Angular distance is measured as a total distance travelled since the last
	//resetAngularDistance command. This is measured by repeatedly measuring increments
	// of angular travel, by saving the lastAngles Orientation variable.
	//"How much has the robot turned?"
	public double getAngularDistance() {
		Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

		double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

		deltaAngle = limitAxes(deltaAngle);

		relativeAngle += deltaAngle;

		lastAngles = angles;

		updates.put("Angular Distance", relativeAngle);
		return relativeAngle;
	}

	//implement getAbsolute if needed

	private double limitAxes(double orientation) {
		if (orientation < -180)
			orientation += 360;
		else if (orientation > 180)
			orientation -= 360;
		return orientation;
	}
}
