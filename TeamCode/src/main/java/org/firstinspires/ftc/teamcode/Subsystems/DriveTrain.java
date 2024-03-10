package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.RobotConstants.RC_Drive;
import org.firstinspires.ftc.teamcode.RobotConstants.TelemetryData;

public class DriveTrain {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    private IMU imu;
    private double headingToMaintain;

    public DriveTrain(DcMotorEx fL, DcMotorEx fR, DcMotorEx bL, DcMotorEx bR, IMU i, boolean fromAuto) {
        this.frontLeft = fL;
        this.frontRight = fR;
        this.backLeft = bL;
        this.backRight = bR;
        this.imu = i;

        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        this.backRight.setDirection(DcMotorEx.Direction.REVERSE);

        this.frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        this.backLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.backRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        this.frontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * RUNS A TEST OF THE MOTORS
     *
     * @param delay how long do you want to run each motor for in ms
     * @param power at what speed
     * @throws InterruptedException
     */
    public void testMotors(int delay, double power) throws InterruptedException {
        this.frontLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(power);
        Thread.sleep(delay);
        this.frontRight.setPower(0);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.backLeft.setPower(0);
        this.backRight.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(power);
        this.frontRight.setPower(power);
        this.backLeft.setPower(power);
        Thread.sleep(delay);
        this.frontLeft.setPower(0);
        this.frontRight.setPower(0);
        this.backLeft.setPower(0);
        this.backRight.setPower(0);
    }

    /** main thread for this class, commands the motors to do required movements
     *
     * @param leftStickX commands robot strafing movements
     * @param leftStickY commands robot forward and backward movements
     * @param rightStickX commands robot's rotation
     */
    public void drive(double leftStickX, double leftStickY, double rightStickX, boolean fromAuto) {
        double x = leftStickX * .75;
        double y = leftStickY * .75; // Counteract imperfect strafing
        double rx = rightStickX * 0.5; //what way we want to rotate

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        if(rx == 0){ //this means that we're trying to maintain our current heading
            double shorter = this.figureOutWhatIsShorter(botHeading);
            //prevents the motors from working when they realistically cant
            boolean isWithinAngularTolerance =
                    Math.abs(shorter) <  0.00872665;

            //we turn if we're not within a tolerance
            if(!isWithinAngularTolerance){
                rx = shorter; //retrieves direction, magnitude overwritten
                rx = this.setToMinimumTurningSpeed(rx); //overwrites magnitude to minimum speed
            }

            TelemetryData.whatHeadingDo = this.headingToMaintain;
        }else{
            //we're going to maintain our new heading once we've stopped turning.
            //not before we've turned
            this.headingToMaintain = botHeading;
        }

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);
    }

    public void setHeadingToMaintain(double input){ this.headingToMaintain = input; }


    /** Determines what direction would be shorter to turn in when trying to maintain our current
     *  heading.
     * @return the shorter difference in heading
     */
    public double figureOutWhatIsShorter(double reading) {
        double result;
        double oppositeButEqualReading;

        if (reading > 0) {
            oppositeButEqualReading = reading - 2 * Math.PI;
        } else {
            oppositeButEqualReading = reading + 2 * Math.PI;
        }

        double normalReadingDifference = Math.abs(this.headingToMaintain - reading);
        double oppositeReadingDifference = Math.abs(this.headingToMaintain - oppositeButEqualReading);

        boolean isOppositeReadingShorter =
                normalReadingDifference > oppositeReadingDifference;

        if (isOppositeReadingShorter) {
            result = this.headingToMaintain - oppositeButEqualReading;
        } else {
            result = this.headingToMaintain - reading;
        }
        return -result;
    }

    /** changes our current turning speed to a turning speed that allows us to rotate
     *
     * @param rx our current turning speed
     * @return modified turning speed
     */
    private double setToMinimumTurningSpeed(double rx){

        if(Math.abs(rx) < RC_Drive.MINIMUM_TURNING_SPEED) {
            if (rx < 0) {
                return -RC_Drive.MINIMUM_TURNING_SPEED;
            } else {
                return RC_Drive.MINIMUM_TURNING_SPEED;
            }
        }else{
            return rx;
        }
    }
}
