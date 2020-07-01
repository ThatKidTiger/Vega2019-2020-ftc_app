package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Map;

public class Latch extends Subsystem{
	private static final String TAG = "Latch";
	private DcMotor latch;

	private String latchName = "latch";

	@Override
	public Map<String, Object> update() {
		return null;
	}

	public Latch() {

	}

	public void init(HardwareMap hwMap) {
		latch = hwMap.get(DcMotor.class, latchName);
		Log.d(TAG, "Initialization Complete");
	}

	public void setPower(double power) {
		latch.setPower(power);
	}
}
