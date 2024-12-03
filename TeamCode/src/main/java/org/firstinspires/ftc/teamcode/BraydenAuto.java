package org.firstinspires.ftc.teamcode;
//
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Objects;


@Autonomous(name = "Auto with Vision", group = "teamcode")
public class BraydenAuto extends LinearOpMode {
    private DcMotor front_left, front_right, back_left, back_right;
    Servo extend, wristL, wristR, bucket;
    DcMotor erect;
    CRServo intakeL, intakeR;
    Vision vision;
    IMU imu;
    Orientation angle;

    public void drive(int millis, double power) {
        front_left.setPower(-power);
        front_right.setPower(-power);
        back_left.setPower(-power);
        back_right.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void turnLeft(int millis, double power) {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void turnRight(int millis, double power) {
        front_left.setPower(-power);
        back_left.setPower(-power);
        front_right.setPower(power);
        back_right.setPower(power);

        sleep(millis);

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void strafeLeft(int millis, double power) {
        front_left.setPower(power);
        back_left.setPower(-power);
        front_right.setPower(-power);
        back_right.setPower(power);

        sleep(millis);

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void strafeRight(int millis, double power) {
        front_left.setPower(-power);
        back_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void runVision() {
        while (opModeIsActive()) {
            if (!vision.initiated) {
                break;
            }
//            vision.elapsedTime.reset();
            telemetry.addData("IsRight", vision.isRight);
            telemetry.addData("IsLeft", vision.isLeft);
            telemetry.addData("IsMiddle", vision.isMiddle);
            telemetry.update();

            vision.update(telemetry, this);
            if (vision.elapsedTime.seconds() >= 0.7) {
                vision.elapsedTime.reset();
            }
        }
    }

    public void driveToPiece() {
        vision.phase++;

        switch (vision.phase) {
            case 1:
                intakeL.setPower(-1);
                intakeR.setPower(1);

                sleep(500);

                wristR.setPosition(1.4);
                wristL.setPosition(-0.04);

                sleep(1000);

                extend.setPosition(0);

                drive(1100, -0.4);

                sleep(900);

                intakeL.setPower(0);
                intakeR.setPower(0);

                sleep(400);

                wristR.setPosition(0.25);
                wristL.setPosition(0.75);

                sleep(500);

                extend.setPosition(0.28);

                sleep(700);

                intakeL.setPower(1);
                intakeR.setPower(-1);

                sleep(500);

                intakeR.setPower(0);
                intakeL.setPower(0);

                wristL.setPosition(-0.04);
                wristR.setPosition(1.4);

                sleep(1000);

                extend.setPosition(0);

                drive(720, 0.5);
                turnRight(160, 0.6);

                erect.setTargetPosition(2500);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(1);

                if (erect.getCurrentPosition() >= 2300 && erect.getCurrentPosition() <= 2500) {
                    erect.setPower(0);
                }

                turnRight(60, 0.5);

                sleep(750);

                bucket.setPosition(0);

                sleep(1400);

                bucket.setPosition(0.83);

                sleep(500);

                erect.setTargetPosition(100);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(-1);
                if (erect.getCurrentPosition() >= 80 && erect.getCurrentPosition() <= 100) {
                    erect.setPower(0);
                }

                wristL.setPosition(0.05);
                wristR.setPosition(0.95);
                intakeL.setPower(-1);
                intakeR.setPower(1);

                vision.initiated = true;
                strafeLeft(50, 0.5);
                turnLeft(320, 0.5);
                extend.setPosition(0.20);
                runVision();
                break;
            case 2:
                intakeL.setPower(-1);
                intakeR.setPower(1);

                sleep(500);

                wristR.setPosition(1.4);
                wristL.setPosition(-0.04);

                sleep(1000);

                extend.setPosition(0);

                drive(1100, -0.4);

                sleep(900);

                intakeL.setPower(0);
                intakeR.setPower(0);

                sleep(400);

                wristR.setPosition(0.25);
                wristL.setPosition(0.75);

                sleep(500);

                extend.setPosition(0.28);

                sleep(700);

                intakeL.setPower(1);
                intakeR.setPower(-1);

                sleep(500);

                intakeR.setPower(0);
                intakeL.setPower(0);

                wristL.setPosition(-0.04);
                wristR.setPosition(1.4);

                sleep(1000);

                extend.setPosition(0);

                sleep(500);

                erect.setTargetPosition(2500);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(1);

                drive(730, 0.5);
                turnRight(160, 0.6);

                if (erect.getCurrentPosition() >= 2300 && erect.getCurrentPosition() <= 2500) {
                    erect.setPower(0);
                }

                turnRight(60, 0.5);

                sleep(750);

                bucket.setPosition(0);

                sleep(1400);

                bucket.setPosition(0.83);

                sleep(500);

                erect.setTargetPosition(100);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(-1);
                if (erect.getCurrentPosition() >= 80 && erect.getCurrentPosition() <= 100) {
                    erect.setPower(0);
                }

                wristL.setPosition(0.05);
                wristR.setPosition(0.95);
                intakeL.setPower(-1);
                intakeR.setPower(1);

                vision.initiated = true;
                strafeLeft(50, 0.5);
                turnLeft(320, 0.5);
                extend.setPosition(0.20);
                runVision();
                break;
            case 3:
                intakeL.setPower(-1);
                intakeR.setPower(1);

                sleep(500);

                wristR.setPosition(1.4);
                wristL.setPosition(-0.04);

                sleep(1000);

                extend.setPosition(0);

                drive(1100, -0.4);

                sleep(900);

                intakeL.setPower(0);
                intakeR.setPower(0);

                sleep(400);

                wristR.setPosition(0.25);
                wristL.setPosition(0.75);

                sleep(500);

                extend.setPosition(0.28);

                sleep(700);

                intakeL.setPower(1);
                intakeR.setPower(-1);

                sleep(500);

                intakeR.setPower(0);
                intakeL.setPower(0);

                wristL.setPosition(-0.04);
                wristR.setPosition(1.4);

                sleep(1000);

                extend.setPosition(0);

                drive(720, 0.6);

                erect.setTargetPosition(2500);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(1);

                if (erect.getCurrentPosition() >= 2300 && erect.getCurrentPosition() <= 2500) {
                    erect.setPower(0);
                }

                turnRight(60, 0.5);

                sleep(750);

                bucket.setPosition(0);

                sleep(1400);

                bucket.setPosition(0.83);

                sleep(500);

                erect.setTargetPosition(100);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(-1);
                if (erect.getCurrentPosition() >= 80 && erect.getCurrentPosition() <= 100) {
                    erect.setPower(0);
                }

                wristL.setPosition(0.05);
                wristR.setPosition(0.95);
                intakeL.setPower(-1);
                intakeR.setPower(1);

                vision.initiated = true;
//                strafeLeft(50, 0.5);
//                turnLeft(180, 0.5);
                extend.setPosition(0.20);
                runVision();
                break;
        }
    }

    public void stopMotors() {
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void goToAngle(double angle, boolean leftOrRight) {
        double setAngle;
        if (leftOrRight) {
            front_left.setPower(0.5);
            back_left.setPower(0.5);
            front_right.setPower(-0.5);
            back_right.setPower(-0.5);
            while (opModeIsActive()) {
                telemetry.addData("imu angle", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                setAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Objects.equals(angle, setAngle)) {
                    stopMotors();
                    break;
                }
            }
        } else {
            turnRight(10000, 0.5);
            while (opModeIsActive()) {
                front_left.setPower(-0.5);
                back_left.setPower(-0.5);
                front_right.setPower(0.5);
                back_right.setPower(0.5);
                setAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Objects.equals(angle, setAngle)) {
                    stopMotors();
                    break;
                }
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontLeftMotor);
        front_right = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontRightMotor);
        back_left = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backLeftMotor);
        back_right = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backRightMotor);
        extend = hardwareMap.get(Servo.class, "extend");
        wristL = hardwareMap.get(Servo.class, "wristL");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        wristR = hardwareMap.get(Servo.class, "wristR");
        bucket = hardwareMap.get(Servo.class, "bucket");
        erect = hardwareMap.get(DcMotor.class, "erect");
        imu = hardwareMap.get(IMU.class, "imu");

        front_right.setDirection(Constants.MecanumConstants.invertRight);
        front_left.setDirection(Constants.MecanumConstants.invertLeft);
        back_right.setDirection(Constants.MecanumConstants.invertRight);
        back_left.setDirection(Constants.MecanumConstants.invertLeft);

        erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        erect.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        vision = new Vision(hardwareMap, telemetry);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);

        waitForStart();

        bucket.setPosition(0.83);


        drive(600, 0.5);
        turnLeft(300, 0.5);
        drive(180, 0.5);

        erect.setTargetPosition(2500);
        erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        erect.setPower(1);

        extend.setPosition(0);

        if (erect.getCurrentPosition() > 2300 && erect.getCurrentPosition() <= 2500) {
            erect.setPower(0);
        }

        wristL.setPosition(-0.04);
        wristR.setPosition(1.4);

        sleep(1000);

        bucket.setPosition(0);

        sleep(2000);

        bucket.setPosition(0.75);

        sleep(400);

        erect.setTargetPosition(100);
        erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        erect.setPower(-1);

        extend.setPosition(0);

        wristL.setPosition(0.75);
        wristR.setPosition(0.25);

        sleep(1000);

        intakeL.setPower(1);
        intakeR.setPower(-1);

        strafeLeft(100, 0.5);
        turnLeft(240, 0.5);

        runVision();

//        imu.resetYaw();
//        drive(500, 0.5);
//        goToAngle(90, true);
    }
}