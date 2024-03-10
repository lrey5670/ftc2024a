package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class ClawJV {
    public Servo clawTop;
    public Servo clawBottom;

    /**
     * Class constructor
     *
     * @param handTop servo obj
     * @param handBottom servo obj
     * @param fromAuto was the constructor called from auto or tele
     */
    public ClawJV(Servo handTop, Servo handBottom, boolean fromAuto) {
        this.clawTop = handTop;
        this.clawBottom = handBottom;
    }

    public void setPosition(double input) {
        this.clawTop.setPosition(input);
        this.clawBottom.setPosition(input);
        TelemetryData.clawTop_position = input;
        TelemetryData.clawBottom_position = input;
    }

    public void setPositionTop(double input) {
        this.clawTop.setPosition(input);
        TelemetryData.clawTop_position = input;
    }
    public void setPositionBottom(double input) {
        this.clawBottom.setPosition(input);
        TelemetryData.clawBottom_position = input;
    }

}
