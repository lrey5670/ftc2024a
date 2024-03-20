
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
        int castlePos = 0;

//        armJV.moveStow();

        waitForStart();
        TelemetryData.whatHeadingDo = 0;
        while (opModeIsActive()) {
            double curX = driveTrain.getXDistance();
            double curY = driveTrain.getYDistance();
//            Detect Where Castle with camera(Write Later)



//            Left Castle move to
            if (autoRunStage == 0 && castlePos == 0){

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

//                armJV.dropOne();


                if(curX <= RC_AutoJV.x_left_spike && curY <= RC_AutoJV.y_left_spike) {

//                    armJV.dropOne();
//                    armJV.updateAll();
//                    Thread.sleep(3000);
//                    telemetry.update();

                    autoRunStage = 1;
//                    Thread.sleep(10000);
                }



                // move back to fit through hole
            }else if(autoRunStage == 0 && castlePos == 1){
                if( curX <= RC_AutoJV.x_center_spike) {
                    RC_AutoJV.xPower = 0;
//                    autoRunStage = 1;
//                    Thread.sleep(1000);
                }

                if ( curY <= RC_AutoJV.y_center_spike){
                    RC_AutoJV.yPower = 0;
                }

                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);

                if(curX <= RC_AutoJV.x_center_spike && curY <= RC_AutoJV.y_center_spike) {

                    autoRunStage = 1;
//                    Thread.sleep(10000);
                }

            }


            else if(autoRunStage == 1){

                if (curY >= RC_AutoJV.stageOneY ){
                    ypower(0);
                    autoRunStage = 2;
                }
                else{
                    ypower(-.4);
                }

//                armJV.moveStow();

                driveTrain.drive(0,RC_AutoJV.yPower,0,true);



//            ride through hole
            }else if(autoRunStage == 2){



                if(curY <= RC_AutoJV.stageTwoYStop){
                    //turning changes the x to y
                    xpower(0);
                    autoRunStage = 3;

                }else{
                    xpower(.6);
                }



                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                TelemetryData.whatHeadingDo = (-1.56);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);
//                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);

//            Go left to match bord
            }else if(autoRunStage == 3){
                if(castlePos == 1){
                    RC_AutoJV.stageThreeXStop = -500;
                }


                if (curX <=  RC_AutoJV.stageThreeXStop){
                    //goes left
                    ypower(0);
//                    autoRunStage = 4;

                }else{
                    ypower(0.4);
                }

                if(castlePos == 1){
                    RC_AutoJV.stageThreeYStop += 200;
                }

                if(curY <= RC_AutoJV.stageThreeYStop){

                    xpower(0);
//                    autoRunStage = 3;

                }else{
                    xpower(.4);
                }

                if(curX <=  RC_AutoJV.stageThreeXStop && curY <= RC_AutoJV.stageThreeYStop ){
                    autoRunStage = 4;
                }


                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                TelemetryData.whatHeadingDo = (-1.56);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);

//            extend arm, move forward and drop off
            }else if(autoRunStage == 4){
                if(castlePos == 1){
                    RC_AutoJV.stageFourYStop += 25;
                }
                    if(curY <= RC_AutoJV.stageFourYStop){
                        //forward
                        xpower(0);
                        autoRunStage = 5;

                    }else{
                        xpower(.3);
                    }



                    driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                    TelemetryData.whatHeadingDo = (-1.56);
                    driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);



//            return to corner
            }else if(autoRunStage == 5){
                if(castlePos == 1){
                    RC_AutoJV.stageFiveXStop += -50;
                }

                if(curX >= RC_AutoJV.stageFiveXStop){
                    //goes right
                    ypower(0);
                    autoRunStage = 6;

                }else{
                    ypower(-0.4);
                }



                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                TelemetryData.whatHeadingDo = (-1.56);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);


//            move right
            }else if(autoRunStage == 60){
                if(curY <= RC_AutoJV.stageSixYStop){
                    //forward
                    xpower(0);
                    autoRunStage = 7;

                }else{
                    xpower(.2);
                }



                driveTrain.drive(RC_AutoJV.xPower ,RC_AutoJV.yPower ,0, true);
                TelemetryData.whatHeadingDo = (-1.56);
                driveTrain.setHeadingToMaintain(TelemetryData.whatHeadingDo);

//           move to corner
            }else if(autoRunStage == 7){

            }





//            Middle Castle move to

//            Right Castle move to

//            Return to Start && retract

//            Move under and over

//            Turn to drop

//            Go to corner
//            armJV.updateAll();
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












