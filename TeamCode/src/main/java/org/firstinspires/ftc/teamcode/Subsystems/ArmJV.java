package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Claw;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Telescope;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Wrist;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class ArmJV {
    ShoulderJV shoulder;
    TelescopeJV telescope;
    WristJV wrist;
    ClawJV claw;
    int state = 0;

    public ArmJV(ShoulderJV shoulder, TelescopeJV telescope, WristJV wrist, ClawJV claw) {
        this.shoulder  = shoulder;
        this.telescope = telescope;
        this.wrist     = wrist;
        this.claw      = claw;
    }

    public void moveStow() throws InterruptedException {
        if (state == 1) {
            state = 0;
            claw.setPositionBottom(RC_Claw.openBottom);
            Thread.sleep(500);
            claw.setPositionTop(RC_Claw.openTop);
            Thread.sleep(500);
        } else if (state == -1) {
            state = 0;
            claw.setPositionTop(RC_Claw.closeTop);
            claw.setPositionBottom(RC_Claw.closeBottom);
            Thread.sleep(500);
        }

        shoulder.setTarget(RC_Shoulder.stowPos);
        telescope.setTarget(RC_Telescope.stowPos);
        wrist.setPosition(RC_Wrist.stowPos);

    }

    public void movePickup() {
        if (state != 1) {
            state = -1;
            shoulder.setTarget(RC_Shoulder.pickupPos);
            telescope.setTarget(RC_Telescope.pickupPos);
            wrist.setPosition(RC_Wrist.pickupPos);
            claw.setPositionTop(RC_Claw.openTop);
            claw.setPositionBottom(RC_Claw.openBottom);
        }
    }

    public void autoDropFloor(){

        if (state != 1) {
            state = -1;
            shoulder.setTarget(RC_Shoulder.pickupPos);
            telescope.setTarget(RC_Telescope.pickupPos);
            wrist.setPosition(RC_Wrist.pickupPos);
//            claw.setPositionTop(RC_Claw.openTop);
            claw.setPositionBottom(RC_Claw.openBottom);
        }


    }

    public void updateAll() {
        shoulder.update();
        telescope.update();
    }

    public void moveDropOff() {
        if (state != -1) {
            state = 1;
            shoulder.setTarget(RC_Shoulder.dropOffPos);
            telescope.setTarget(RC_Telescope.dropOffPos);
            wrist.setPosition(RC_Wrist.dropOffPos);
        }
    }

    public void dropOne() {
        shoulder.setTarget(RC_Shoulder.pickupPos);
        telescope.setTarget(RC_Telescope.pickupPos);
        wrist.setPosition(RC_Wrist.pickupPos);
        claw.setPositionBottom(RC_Claw.openBottom);
    }
}
