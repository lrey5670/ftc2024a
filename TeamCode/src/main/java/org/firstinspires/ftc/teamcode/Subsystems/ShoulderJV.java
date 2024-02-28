package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class ShoulderJV {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;
    private boolean resetting = false;
    private boolean resetTriggered = false;

    /**
     * Class constructor
     * @param m motor obj
     * @param t touch sensor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public ShoulderJV(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorEx.Direction.REVERSE);
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        this.motor.setPower(0);
    }

    public void moveMotor(double input) {
        this.motor.setPower(input);
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
    }
}
