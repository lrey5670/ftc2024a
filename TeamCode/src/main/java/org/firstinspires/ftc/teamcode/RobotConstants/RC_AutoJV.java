package org.firstinspires.ftc.teamcode.RobotConstants;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Subsystems.Point;


@Config

public class RC_AutoJV {
    public static int x_center_board = -830;
    public static int y_center_board = 470;


    public static int x_center_spike = -101;
    public static int y_center_spike = -408;

    public static Point center_point = new Point(x_center_board, y_center_board);
    public static Point center_spike_point = new Point(x_center_spike, y_center_spike);

    public static int x_left_board = -845;
    public static int y_left_board = 470;


    public static int x_left_spike = -341;
    public static int y_left_spike = -104;
    public static int x_stack = -1470;
    public static int y_stack = -2050;
    public static Point stack_point = new Point(x_stack, y_stack);
    public static int x_safe = -1450;
    public static int y_safe = 40;
    public static Point point_safe = new Point(x_safe, y_safe);
    public static int cameraLine = 255;

    public static double angle = 0;

    public static double xPower = 0;
    public static  double yPower= 0;
    
    public static double stageOneX = 50;
    public static double stageOneY = -145;
    public static double stageTwoYStop = -1800;
    public static double stageThreeXStop = -600;
    public static double stageThreeYStop = -2000;
    public static double stageFourYStop = -2250;
    public static double stageFiveXStop = -30;
    public static double stageSixYStop = -2400;
    public static double target = -2400;



}
