package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RC_Telescope {
    public static double kP = 0.01;
    public static double kI = 0;
    public static double kD = 0;
    public static double kG = 0.025;
    public static int maxVel = 10000;
    public static int maxAccel = 10000;
    public static int maxJerk = 2000;
    public static int stowPos = 0;
    public static int pickupPos = 1165;
    public static int dropOffPos = 1300;
    public static double speedMulti = 1.5;
}
