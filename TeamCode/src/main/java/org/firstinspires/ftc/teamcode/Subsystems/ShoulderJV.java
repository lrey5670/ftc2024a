package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
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
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        this.motor.setPower(0);
    }

    public void moveMotor(double input, boolean manualMove) {
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
        if (manualMove) {
            TelemetryData.shoulder_target = TelemetryData.shoulder_position;
        }
        if (this.touch.isPressed() && !this.resetTriggered) {
            TelemetryData.shoulder_position = 0;
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.resetTriggered = true;
        }
        if (this.touch.isPressed() && input < 0) {
            this.motor.setPower(0);
        } else if (TelemetryData.shoulder_position > 2850 && input > 0) {
            this.motor.setPower(0);
        } else {
            this.resetTriggered = false;
            this.motor.setPower(input * 0.5);
        }
    }


    /*
    public void update()
    {
        this.controller.setPID(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        TelemetryData.shoulder_position = this.motor.getCurrentPosition();
        double power = this.controller.calculate(TelemetryData.shoulder_position, TelemetryData.shoulder_target);
        power = power + RC_Shoulder.kF;
        TelemetryData.shoulder_power = power;
        this.motor.setPower(power);
    }

     */

    public void setTarget (int t) {
        TelemetryData.shoulder_target = t;
    }

    public void update() {
        //are we within a tolerance of where we want to be
        if (Math.abs(TelemetryData.shoulder_position - TelemetryData.shoulder_target) > 100) {
            if (TelemetryData.shoulder_position > TelemetryData.shoulder_target) {
                moveMotor(-RC_Shoulder.speedMulti, false);
            } else {
                moveMotor(RC_Shoulder.speedMulti, false);
            }
        } else {
            moveMotor(0, false);
        }
    }
}
