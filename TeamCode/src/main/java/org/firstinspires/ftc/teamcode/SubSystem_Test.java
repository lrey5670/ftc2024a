package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.RobotConstants.RC_Shoulder;
import org.firstinspires.ftc.teamcode.RobotConstants.RobotConstants;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;
import org.firstinspires.ftc.teamcode.Subsystems.ArmJV;
import org.firstinspires.ftc.teamcode.Subsystems.ClawJV;
import org.firstinspires.ftc.teamcode.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.Subsystems.Shoulder;
import org.firstinspires.ftc.teamcode.Subsystems.ShoulderJV;
import org.firstinspires.ftc.teamcode.Subsystems.TelescopeJV;
import org.firstinspires.ftc.teamcode.Subsystems.WristJV;

@TeleOp(name = "SubSystem_Test")
public class SubSystem_Test extends LinearOpMode {
    @Override
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
        waitForStart();
        while(opModeIsActive()) {
            myGamePad.readButtons();
            left_y = zeroAnalogInput(gamepad1.left_stick_y);
            right_y = zeroAnalogInput(gamepad1.right_stick_y);
            left_x = zeroAnalogInput(gamepad1.left_stick_x);
            right_x = zeroAnalogInput(gamepad1.right_stick_x);

            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.X)) {
                armJV.moveStow();
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.A)) {
                armJV.movePickup();
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.Y)) {
                armJV.moveDropOff();
            }
            if (!myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.wasJustReleased(GamepadKeys.Button.B)) {
                driveTrain.testMotors(1000,.5);
            }

            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.Y)) {
                driveTrain.setHeadingToMaintain(0);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.X)) {
                driveTrain.setHeadingToMaintain(1.57);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.A)) {
                driveTrain.setHeadingToMaintain(3.14);
            }
            if (myGamePad.isDown(GamepadKeys.Button.LEFT_BUMPER) && myGamePad.isDown(GamepadKeys.Button.B)) {
                driveTrain.setHeadingToMaintain(-1.57);
            }

            armJV.updateAll();


            //wristJV.setPosition(TelemetryData.wrist_position);
            //clawJV.setPosition(TelemetryData.clawTop_position);
            if (myGamePad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && left_y != 0) {
                shoulderJV.moveMotor(left_y, true);
            }

            if (myGamePad.isDown(GamepadKeys.Button.RIGHT_BUMPER) && right_y != 0) {
                telescopeJV.moveMotor(right_y, true);
            }

            if (!myGamePad.isDown(GamepadKeys.Button.RIGHT_BUMPER)) {
                driveTrain.drive(left_x, -left_y, right_x, false);
            }

            double curX = driveTrain.getXDistance();
            double curY = driveTrain.getYDistance();
            telemetry.addData("XPOS", curX);
            telemetry.addData("YPOS", curY);

            telemetry.addData("wrist position", TelemetryData.wrist_position);
            telemetry.addData("telescope position", TelemetryData.telescope_position);
            telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            telemetry.addData("clawTop position", TelemetryData.clawTop_position);
            telemetry.addData("clawBottom position", TelemetryData.clawBottom_position);


            //shoulderJV.update();
            //telemetry.addData("shoulder position", TelemetryData.shoulder_position);
            //telemetry.addData("shoulder target", TelemetryData.shoulder_target);
            //telemetry.addData("shoulder power", TelemetryData.shoulder_power);
            //telemetry.addData("shoulder velocity", TelemetryData.shoulder_velocity);
            telemetry.update();
        }
    }

    /**
     * removes the analog drift
     * @param input
     * @return
     */
    private double zeroAnalogInput(double input){
        if (Math.abs(input) < RobotConstants.analogTol){
            input = 0;
        } else if (input > 0) {
            input -= RobotConstants.analogTol;
        } else {
            input += RobotConstants.analogTol;
        }
        return input;
    }
}
