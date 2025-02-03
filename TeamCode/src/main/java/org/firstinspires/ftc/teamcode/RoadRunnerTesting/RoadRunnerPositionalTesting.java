package org.firstinspires.ftc.teamcode.RoadRunnerTesting;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ConstantsPackage.Constantss;

class Driving extends RoadRunnerPositionalTesting implements Action {

    int targetX;
    int targetY;

    public void resetEncoders() {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public Driving(HardwareMap hardwareMap, Telemetry telemetry, int targetX, int targetY) {
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameter = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameter);
        imu.resetYaw();

        this.targetX = targetX;
        this.targetY = targetY;

        telemetry.addLine("Driving Motors");
        telemetry.addData("Right Front Initialized", rightFront != null);
        telemetry.addData("Right Back Initialized", rightBack != null);
        telemetry.addData("Left Front Initialized", leftFront != null);
        telemetry.addData("Left Back Initialized", leftBack != null);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveToPose2D(int targetX, int targetY) {
        Pose2d pose;

        resetEncoders();

        sleep(500);

        double y;
        double x;

        double targetAngle = Math.toDegrees(Math.atan2(targetY, targetX));

        calculateAndDrivePart2(45, 0, 5);

        while(true) {

            pose = updateCurrentPose2D();
            y =  pose.position.y;
            x = pose.position.x;

            Log.i(RoadRunnerPositionalTesting.class.getName(), "X: " + x + ", Y: " + y);

            if(Math.abs(y) < 1 && Math.abs(x) < 1) {
                break;
            }
        }

        Log.i(RoadRunnerPositionalTesting.class.getName(), "X: " + x + ", Y: " + y);

        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
    }

    public void calculateAndDrive(double targetX, double targetY, double heading) {
        double targetAngle = Math.atan2(targetY, targetX);

        double distance = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));

        double xPower = Math.cos(targetAngle) * distance;
        double yPower = Math.sin(targetAngle) * distance;

        // Calculate motor powers based on mechanic kinematics
        double leftFrontPower = xPower + yPower + heading;
        double leftBackPower = -xPower + yPower + heading;
        double rightFrontPower = -xPower + yPower - heading;
        double rightBackPower = xPower + yPower - heading;

        // Find the maximum power value to normalize all motor powers
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(leftBackPower),
                Math.max(Math.abs(rightFrontPower), Math.abs(rightBackPower))));

        // Normalize the motor powers if they exceed 1.0
        if (maxPower > 1) {
            leftFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightFrontPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // Apply the calculated motor powers
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);

//        // Log the calculated powers for debugging
//        Log.i(RoadRunnerPositionalTesting.class.getName(),
//                "Left Front Power: " + leftFrontPower +
//                        ", Left Back Power: " + leftBackPower +
//                        ", Right Front Power: " + rightFrontPower +
//                        ", Right Back Power: " + rightBackPower);
//
//        sleep(1000000000);
    }

    public void calculateAndDrivePart2(double angle, double heading, double targetDistance) {
        double[] xAndY = getXAndY(angle, targetDistance);

        double distance = Math.sqrt(Math.pow(xAndY[0], 2) + Math.pow(xAndY[1], 2));

        double xPower = Math.cos(Math.toRadians(angle)) * distance;
        double yPower = Math.sin(Math.toRadians(angle)) * distance;

        double frontLeftPower = xPower + yPower + heading;
        double backLeftPower = -xPower + yPower + heading;
        double frontRightPower = -xPower + yPower - heading;
        double backRightPower = xPower + yPower - heading;

        double maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(backLeftPower), Math.max(Math.abs(frontRightPower), Math.abs(backRightPower))));

        if(maxPower > 1) {
            frontLeftPower /= maxPower;
            backLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backRightPower /= maxPower;
        }

        leftFront.setPower(frontLeftPower);
        leftBack.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightBack.setPower(backRightPower);
    }

    public double[] getXAndY(double angle, double targetDistance) {
        double[] xAndY = new double[2];

        double x = Math.cos(Math.toRadians(angle));
        double y = Math.sin(Math.toRadians(angle));

        double percentage = x/y;

        double transformedX = targetDistance * percentage;

        xAndY[0] = transformedX;
        xAndY[1] = targetDistance;

        return xAndY;
    }

    public Pose2d updateCurrentPose2D() {
        double x = targetX - getDisplacement().position.x;
        double y = targetY - getDisplacement().position.y;

        return new Pose2d(x, y, 0);
    }

    public Pose2d getDisplacement() {
        // Get motor distances in inches
        double rightFrontInches = rightFront.getCurrentPosition() / Constantss.MotorConstants.TICKS_TO_INCH;
        double rightBackInches = rightBack.getCurrentPosition() / Constantss.MotorConstants.TICKS_TO_INCH;
        double leftFrontInches = leftFront.getCurrentPosition() / Constantss.MotorConstants.TICKS_TO_INCH;
        double leftBackInches = leftBack.getCurrentPosition() / Constantss.MotorConstants.TICKS_TO_INCH;

        // Calculate forward/backward (y) and lateral (x) displacement
        double yDisplacement = (rightFrontInches + rightBackInches + leftFrontInches + leftBackInches) / 4.0;
        double xDisplacement = (-rightFrontInches + rightBackInches + leftFrontInches - leftBackInches) / 4.0;

        return new Pose2d(xDisplacement, yDisplacement, 0);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        driveToPose2D(targetX, targetY);
        return false;
    }
}

@Autonomous(name = "RoadRunnerPositionalTesting", group = "teamcode")
public class RoadRunnerPositionalTesting extends LinearOpMode {
    DcMotorEx rightFront, rightBack, leftFront, leftBack;
    IMU imu;
    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addLine("RoadRunnerPositionalTesting Motors");
        telemetry.addData("Right Front Initialized", rightFront != null);
        telemetry.addData("Right Back Initialized", rightBack != null);
        telemetry.addData("Left Front Initialized", leftFront != null);
        telemetry.addData("Left Back Initialized", leftBack != null);

        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);

        IMU.Parameters parameter = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameter);
        imu.resetYaw();

        telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

        Driving driving = new Driving(hardwareMap, telemetry, 5, 5);

        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        t -> {
                            rightFront.setPower(0.2);
                            rightBack.setPower(0.2);
                            leftFront.setPower(0.2);
                            leftBack.setPower(0.2);

                            sleep(1000);

                            rightFront.setPower(0);
                            rightBack.setPower(0);
                            leftFront.setPower(0);
                            leftBack.setPower(0);
                           return false;
                        },
                       driving
                )
        );
    }
}
