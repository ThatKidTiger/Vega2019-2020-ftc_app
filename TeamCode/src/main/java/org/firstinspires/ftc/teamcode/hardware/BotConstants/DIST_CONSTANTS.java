package org.firstinspires.ftc.teamcode.hardware.BotConstants;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class DIST_CONSTANTS {
    public static PIDCoefficients DIST_PID = new PIDCoefficients(0.02, 0, 0.004);
    public static double DIST_MIN = 0.1;
    public static int dist;
    public static double power;
}
