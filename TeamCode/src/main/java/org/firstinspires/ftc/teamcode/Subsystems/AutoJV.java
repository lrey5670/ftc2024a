
package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_AutoJV;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

@Autonomous(name = "AUTOJV")
public class AutoJV extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        //this section allows us to access telemetry data from a browser
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        ShoulderJV shoulderJV = new ShoulderJV(
                hardwareMap.get(DcMotorEx.class, "armRot"),
                hardwareMap.get(TouchSensor.class, "armLimit"),
                true);

        TelescopeJV telescopeJV = new TelescopeJV(
                hardwareMap.get(DcMotorEx.class, "armExt"),
                hardwareMap.get(TouchSensor.class, "extLimit"),
                true);

        WristJV wristJV = new WristJV(hardwareMap.get(Servo.class, "wristLeft"),
                hardwareMap.get(Servo.class, "wristRight"),
                true);

        ClawJV clawJV = new ClawJV(hardwareMap.get(Servo.class, "handTop"),
                hardwareMap.get(Servo.class, "handBottom"),
                true);

        ArmJV armJV = new ArmJV(shoulderJV, telescopeJV, wristJV, clawJV);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);


        DriveTrain driveTrain = new DriveTrain(
                hardwareMap.get(DcMotorEx.class, "frontLeft"),
                hardwareMap.get(DcMotorEx.class, "frontRight"),
                hardwareMap.get(DcMotorEx.class, "backLeft"),
                hardwareMap.get(DcMotorEx.class, "backRight"),
                imu,
                true);


        GamepadEx myGamePad = new GamepadEx(gamepad1);

        double left_y = gamepad1.left_stick_y;
        double right_y = gamepad1.right_stick_y;
        double left_x = gamepad1.left_stick_x;
        double right_x = gamepad1.right_stick_x;

        RC_AutoJV.xPower = -0.4;
        RC_AutoJV.yPower = 0.4;

        double castleLocation;
        double autoRunStage = 0;

//        armJV.moveStow();

        waitForStart();
        TelemetryData.whatHeadingDo = 0;
        while (opModeIsActive()) {
            double curX = driveTrain.getXDistance();
            double curY = driveTrain.getYDistance();
//            Detect Where Castle with camera(Write Later)
//            Left Castle move to
            if (autoRunStage == 0){
//                TelemetryData.xTarget = RC_AutoJV.x_left_spike;
//                TelemetryData.yTarget = RC_AutoJV.y_left_spike;


//                driveTrain.drive(driveTrain.calcXPower(),driveTrain.calcYPower(),0, true);
//                if(!( curX <= RC_AutoJV.x_left_spike) || !( curY <= RC_AutoJV.y_left_spike) ){



//                }

                if( curX <= RC_AutoJV.x_left_spike) {
                    RC_AutoJV.xPower = 0;
//                    autoRunStage = 1;
//                    Thread.sleep(1000);
                }

                if ( curY <= RC_AutoJV.y_left_spike){
                    RC_AutoJV.yPower = 0;
                }

                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);

                if(curX <= RC_AutoJV.x_left_spike && curY <= RC_AutoJV.y_left_spike){
//                    RC_AutoJV.yPower = 0;
//                    RC_AutoJV.xPower = 0;
                    autoRunStage = 1;
//                    Thread.sleep(10000);
                }


//            Turn
            }else if(autoRunStage == 1){

                if (curY <= -40){
                    xpower(0);
                    autoRunStage = 2;
                }
                else{
                    ypower(-.2);
                }
                if(curX <= 0){
                    xpower(0);
                }else{
                    xpower(.2);
                }
//                driveTrain.drive(0,0,0,true);
//                driveTrain.drive(0,0.4,0, true);
//                RC_AutoJV.yPower = 0.4;
//                RC_AutoJV.xPower = 0.4;
                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                TelemetryData.whatHeadingDo = (-1.57);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);



            }else if(autoRunStage == 2){
                xpower(.4);
                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
            }

//
//            if (autoRunStage == 1) {
////
//
//            }


//            Middle Castle move to

//            Right Castle move to

//            Return to Start && retract

//            Move under and over

//            Turn to drop

//            Go to corner

            telemetry.addData("XPOS", curX);
            telemetry.addData("YPOS", curY);
            telemetry.addData("autoRunStage", autoRunStage);
            telemetry.addData("xpow", RC_AutoJV.xPower);
            telemetry.addData("ypow", RC_AutoJV.yPower);
            telemetry.addData("wrist position", TelemetryData.wrist_position);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("clawTop position", TelemetryData.clawTop_position);
            telemetry.addData("clawBottom position", TelemetryData.clawBottom_position);
//            telemetry.addData("heading", driveTrain.);
//            telemetry.addData("yaw to hold", TelemetryData.whatHeadingDo);
            telemetry.update();





        }

    }

    private void xpower(double input){
        RC_AutoJV.xPower = input;
    }
    private void ypower(double input){
        RC_AutoJV.yPower = input;
    }


}












