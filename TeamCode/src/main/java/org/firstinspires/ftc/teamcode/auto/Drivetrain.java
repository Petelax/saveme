package org.firstinspires.ftc.teamcode.auto;

import static com.acmerobotics.roadrunner.ftc.OTOSKt.RRPoseToOTOSPose;
import static org.firstinspires.ftc.teamcode.ConstantsPackage.Constants.MecanumConstants;

import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.TheoCode.Utilities;

public class Drivetrain extends SubsystemBase {
    private DcMotor frontLeft0, frontRight1, backLeft2, backRight3;
    //private IMU imu;
    private SparkFunOTOS otos;

    double deadband = 0.3;
    double deadbandSpeed = 0.05;

    boolean slowMode;

    public static double linearScalar = 1.0;
    public static double angularScalar = 1.0;
    public static SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-2.0, 0.0, Math.PI);

    public Pose2d pose;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft0 = hardwareMap.get(DcMotor.class, MecanumConstants.frontLeftMotor);
        frontRight1 = hardwareMap.get(DcMotor.class, MecanumConstants.frontRightMotor);
        backLeft2 = hardwareMap.get(DcMotor.class, MecanumConstants.backLeftMotor);
        backRight3 = hardwareMap.get(DcMotor.class, MecanumConstants.backRightMotor);

        frontLeft0 = motorConfig(frontLeft0);
        frontRight1 = motorConfig(frontRight1);
        backLeft2 = motorConfig(backLeft2);
        backRight3 = motorConfig(backRight3);

        frontLeft0.setDirection(MecanumConstants.invertLeft);
        frontRight1.setDirection(MecanumConstants.invertRight);
        backLeft2.setDirection(MecanumConstants.invertLeft);
        backRight3.setDirection(MecanumConstants.invertRight);

        frontLeft0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        otos = hardwareMap.get(SparkFunOTOSCorrected.class,"sensor_otos");
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);

        otos.setOffset(offset);
        System.out.println("OTOS calibration beginning!");
        System.out.println(otos.setLinearScalar(linearScalar));
        System.out.println(otos.setAngularScalar(angularScalar));

        otos.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
    }

    public void resetGyro() {
        setPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(0.0)));

    }

    public void setPose(Pose2d pose) {
        otos.setPosition(new SparkFunOTOS.Pose2D(pose.getX(), pose.getY(), pose.getRotation().getDegrees()));
    }

    public Pose2d getPose() {
        return pose;
    }

    public void drive(double ySpeed, double xSpeed, double rot, boolean fieldOriented) {

        double rotY = ySpeed;
        double rotX = xSpeed;

        if (fieldOriented) {
            double botHeading = Math.toRadians(otos.getPosition().h);

            // Rotate the movement direction counter to the bot's rotation
            rotX = xSpeed * Math.cos(-botHeading) - ySpeed * Math.sin(-botHeading);
            rotY = xSpeed * Math.sin(-botHeading) + ySpeed * Math.cos(-botHeading);
        }

        double denominator = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        setPower(frontLeftPower, backLeftPower, frontRightPower, backRightPower);
    }

    public void setPower(double frontLeft, double backLeft, double frontRight, double backRight) {
        frontLeft0.setPower(frontLeft);
        backLeft2.setPower(backLeft);
        frontRight1.setPower(frontRight);
        backRight3.setPower(backRight);
    }

    public double[] getPower() {
        double[] power = new double[4];

        power[0] = frontLeft0.getPower();
        power[1] = backLeft2.getPower();
        power[2] = frontRight1.getPower();
        power[3] = backRight3.getPower();

        return power;
    }


    public double getLeftDist() {
        return (frontLeft0.getCurrentPosition()) * MecanumConstants.ticksToInch;
    }

    public double getRightDist() {
        return (frontRight1.getCurrentPosition()) * MecanumConstants.ticksToInch;
    }

    public double getAvgDist() {
        return (getLeftDist() + getRightDist()) / 2;
    }

    /**
     * @return deg
     */
    public double getYaw() {
        return pose.getRotation().getDegrees();
    }

    public void resetEncoders() {
        motorConfig(frontLeft0);
        motorConfig(frontRight1);
        motorConfig(backLeft2);
        motorConfig(backRight3);
    }

    public void driveToPos(int dist) {
        frontLeft0.setTargetPosition(dist);
        frontRight1.setTargetPosition(dist);
        backLeft2.setTargetPosition(dist);
        backRight3.setTargetPosition(dist);
        positionConfig();

    }

    public boolean atTarget() {
        return Utilities.withinBounds(frontLeft0.getCurrentPosition(), frontLeft0.getTargetPosition(), 5) &&
                Utilities.withinBounds(frontRight1.getCurrentPosition(), frontRight1.getTargetPosition(), 5);
    }


    public void periodicPrint(Telemetry telemetry) {
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("heading (deg)", pose.getRotation().getDegrees());
    }

    /**
     * Standard motor config for all drivetrain motors
     *
     * @param motor DcMotor to  configure
     * @return configured DcMotor
     */
    public DcMotor motorConfig(DcMotor motor) {
        motor.setZeroPowerBehavior(MecanumConstants.neutralMode);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        return motor;
    }

    public void positionConfig() {
        frontLeft0.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void periodic() {
        SparkFunOTOS.Pose2D sparkfunPose = otos.getPosition();
        pose = new Pose2d(sparkfunPose.x, sparkfunPose.y, new Rotation2d(sparkfunPose.h));

    }

}