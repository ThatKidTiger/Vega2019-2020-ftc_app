package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashMap;
import java.util.Map;

public class LiftIntake extends Subsystem {
	private static final String TAG = "LiftIntake";

	private DcMotor lift;
	private DcMotor gripper;

	private String liftName = "lift";
	private String gripperName = "gripper";

	private HashMap<String, Object> updates = new HashMap<>();

	@Override
	public Map<String, Object> update() {
		updates.put("Lift Position", lift.getCurrentPosition());
		return updates;
	}

	public LiftIntake() {

	}

	public void init(HardwareMap hwMap) {
		lift = hwMap.get(DcMotor.class, liftName);
		gripper = hwMap.get(DcMotor.class, gripperName);

		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		Log.d(TAG, "Initialization Complete");
	}

	public void setLiftPower(double power) {
		lift.setPower(power);
	}

	public void setGripperPower(double power) {
		gripper.setPower(power);
	}
}
