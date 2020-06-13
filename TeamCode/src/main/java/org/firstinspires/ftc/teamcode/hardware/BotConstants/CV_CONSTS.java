package org.firstinspires.ftc.teamcode.hardware.BotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class CV_CONSTS {
	@Config
	public enum CV_THRESH {;
		public static int UPH = 30;
		public static int UPS = 255;
		public static int UPV = 255;
		public static int LOH = 20;
		public static int LOS = 95;
		public static int LOV = 210;
	}
	public static int blur = 9;
	public static int threshold = 1;
	public static int sigX = 9;
}
