package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class WristJV {
    public Servo leftWrist;
    public Servo rightWrist;
    private double target;
    private int time;
    private ElapsedTime timer = new ElapsedTime();
    private double moveIncrement;

    /**
     * Class constructor
     *
     * @param leftJoint servo obj
     * @param rightJoint servo obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public WristJV(Servo leftJoint, Servo rightJoint, boolean fromAuto) {
        this.leftWrist = leftJoint;
        this.rightWrist = rightJoint;
        this.rightWrist.setDirection(Servo.Direction.REVERSE);

        /*
        if (fromAuto) {
            this.leftWrist.setPosition(RC_Wrist.stowPos);
            this.rightWrist.setPosition(RC_Wrist.stowPos);
            TelemetryData.wrist_position = RC_Wrist.stowPos;
            this.target = RC_Wrist.stowPos;
        } else {
            this.leftWrist.setPosition(TelemetryData.wrist_position);
            this.rightWrist.setPosition(TelemetryData.wrist_position);
            this.target = TelemetryData.wrist_position;
        }
         */
    }

    /**
     * Will set desired target for the servo and the time it will take to get there
     * servo will move gradually to target in the update function
     *
     * @param input value between 0 and 1, make sure servo can handle it
     * @param t time it will take to go from current location to new location in milliseconds,
     *          a value of 0 will move at the max servo speed
     */
    public void setTarget(double input, int t) {
        if (input != TelemetryData.wrist_position) {
            if (t != 0) {
                this.target = input;
                this.time = t;
                this.timer.reset();
                this.moveIncrement = (input - TelemetryData.wrist_position) / t;
            } else {
                this.leftWrist.setPosition(input);
                this.rightWrist.setPosition(input);
                TelemetryData.wrist_position = input;
            }
        }
    }

    public void setPosition(double input) {
        this.leftWrist.setPosition(input);
        this.rightWrist.setPosition(input);
        TelemetryData.wrist_position = input;
    }
}
