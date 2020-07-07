package org.firstinspires.ftc.teamcode.hardware.BotConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class IMU_CONSTANTS {
	public static PIDCoefficients ROT_PID = new PIDCoefficients(0.0375, 0, 0.01);
	public static PIDCoefficients STRAIGHT_PID = new PIDCoefficients(0.0375, 0, 0.01);
	public static int angle = 90;
	public static double ROT_MIN = 0.1;
	public static double ROT_MAX = 0.8;
}