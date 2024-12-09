package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Auto With IMU", group = "teamcode")
public class BraydenAuto2 extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight, elevator;
    CRServo intakeL, intakeR;
    Servo extend, wristL, wristR, bucket;
    IMU imu;

    public void drive(int millis, double power) {
        frontLeft.setPower(-power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(-power);

        sleep(millis);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnLeft(int millis, double power) {
        frontLeft.setPower(power);
        backLeft.setPower(power);
        frontRight.setPower(-power);
        backRight.setPower(-power);

        sleep(millis);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void turnRight(int millis, double power) {
        frontLeft.setPower(-power);
        backLeft.setPower(-power);
        frontRight.setPower(power);
        backRight.setPower(power);

        sleep(millis);

        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }

    public void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void turnToAngle(double angle, String direction, double power) {
        double currentAngle;

        switch(direction) {
            case "left":
                frontLeft.setPower(power);
                backLeft.setPower(power);
                frontRight.setPower(-power);
                backRight.setPower(-power);

                while(opModeIsActive()) {
                    currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("IMU Current Angle", currentAngle);
                    telemetry.update();

                    if(Math.abs(angle - currentAngle) <= 2) {
                        telemetry.addData("It", "worked.");
                        telemetry.update();
                        stopMotors();
                        break;
                    }
                }
                break;

            case "right":
                frontLeft.setPower(-power);
                backLeft.setPower(-power);
                frontRight.setPower(power);
                backRight.setPower(power);

                while(opModeIsActive()) {
                    currentAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                    telemetry.addData("IMU Current Angle", currentAngle);
                    telemetry.update();

                    if(Math.abs(angle - currentAngle) <= 2) {
                        telemetry.addData("It", "worked.");
                        telemetry.update();
                        stopMotors();
                        break;
                    }
                }
                break;
        }
    }

    public void raiseElevator() {
        extend.setPosition(0);

        sleep(800);

        elevator.setTargetPosition(Constants.MecanumConstants.ELEVATOR_HIGH_BASKET);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);

        if(elevator.getCurrentPosition() >= 2300 && elevator.getCurrentPosition() <= 2500) {
            elevator.setPower(0);
        }
        sleep(1000);
        bucket.setPosition(0.1);
    }

    public void lowerElevator() {
        bucket.setPosition(0.83);
        sleep(900);

        elevator.setTargetPosition(Constants.MecanumConstants.ELEVATOR_LOW_BASKET);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);
        if(elevator.getCurrentPosition() >= 200 && elevator.getCurrentPosition() <= 50) {
            elevator.setPower(0);
        }
        sleep(1500);
    }

    public void putDownIntakeAndExtend() {
        intakeL.setPower(-1);
        intakeR.setPower(1);
        wristL.setPosition(-0.04);
        wristR.setPosition(1.4);
        extend.setPosition(0);

        sleep(600);
    }

    public void extendBackAndPutUpIntake() {
        intakeL.setPower(0);
        intakeR.setPower(0);
        wristL.setPosition(0.80);
        wristR.setPosition(0.34);
        extend.setPosition(0.83);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backRightMotor);

        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        extend = hardwareMap.get(Servo.class, "extend");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        bucket = hardwareMap.get(Servo.class, "bucket");
        elevator = hardwareMap.get(DcMotor.class, "erect");

        frontLeft.setDirection(Constants.MecanumConstants.invertLeft);
        frontRight.setDirection(Constants.MecanumConstants.invertRight);
        backLeft.setDirection(Constants.MecanumConstants.invertLeft);
        backRight.setDirection(Constants.MecanumConstants.invertRight);

        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        /*Reset the yaw of the Gyro system.*/
        imu.resetYaw();

        /*Drive forward a tiny bit to get into position to turn toward the basket.*/
        drive(550, 0.5);

        //Extend and put down intake to warm up the extend and wrist servos.
        putDownIntakeAndExtend();

        turnToAngle(Constants.MecanumConstants.TURN_TO_BASKET, "left", 0.5);

        /*Extend the elevator up and raise the basket to score piece.*/
        raiseElevator();

        sleep(2000);

        //Raise the intake back up after the elevator back up and extend
        extendBackAndPutUpIntake();

        /*Lower the elevator and the bucket back to their original position after scoring piece.*/
        lowerElevator();

        sleep(500);

        //Turn 70 degrees to the left to align robot with the first piece.
        turnToAngle(70, "left", 0.3);

        //Puts down the intake and extends the arm to grab the first piece.
        putDownIntakeAndExtend();

        sleep(800);

        //Drive to the first piece.
        drive(400, -0.5);

        /*Constantly update the Telemetry with the Gyro system's current angle to calibrate the
        * the robot's positioning on the field.*/
        while(opModeIsActive()) {
            telemetry.addData("Imu Angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();
        }
    }
}


