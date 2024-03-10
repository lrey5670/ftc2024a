package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class TelescopeJV {
    private DcMotorEx motor;
    private TouchSensor touch;
    private PIDController controller;
    private boolean resetting = false;
    private boolean resetTriggered = false;
    /**
     * Class constructor
     * @param m motor obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public TelescopeJV(DcMotorEx m, TouchSensor t, boolean fromAuto) {
        this.motor = m;
        this.touch = t;
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.controller = new PIDController(RC_Shoulder.kP, RC_Shoulder.kI, RC_Shoulder.kD);
        this.motor.setPower(0);
    }

    public void moveMotor(double input, boolean manualMoving) {
        TelemetryData.telescope_position = this.motor.getCurrentPosition();
        if (manualMoving) {
            TelemetryData.telescope_target = TelemetryData.telescope_position;
        }
        if (this.touch.isPressed()) {
            TelemetryData.telescope_position = 0;
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.resetTriggered = true;
        }

        if (this.touch.isPressed() && input < 0) {
            this.motor.setPower(0);
        }
        else if (TelemetryData.telescope_position > 1300 && input > 0){
            this.motor.setPower(0);
        }
        else {
            this.motor.setPower(input * 0.5);
        }
    }

    public void setTarget (int t) {
        TelemetryData.telescope_target = t;
    }

    public void update() {
        //are we within a tolerance of where we want to be
        if (Math.abs(TelemetryData.telescope_position - TelemetryData.telescope_target) > 100) {
            if (TelemetryData.telescope_position > TelemetryData.telescope_target) {
                moveMotor(-RC_Telescope.speedMulti, false);
            } else {
                moveMotor(RC_Telescope.speedMulti, false);
            }
        } else {
            moveMotor(0, false);
        }
    }
}
