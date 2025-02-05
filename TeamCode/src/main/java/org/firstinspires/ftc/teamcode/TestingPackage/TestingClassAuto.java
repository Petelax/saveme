package org.firstinspires.ftc.teamcode.TestingPackage;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TheoCode.SparkFunOTOSDrive;

@Autonomous(name = "TestingClassAuto", group = "teamcode")
public class TestingClassAuto extends LinearOpMode {

    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    DcMotor elevator;
    Servo leftExtend, rightExtend, intake, intakeRotate;
    HuskyLens huskyLens;

    public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
    }

    public static class Constants {
        public static class MotorConstants {

        }
        public static class ServoConstants {

            //EXTENSION CONSTANTS
            public static final double maxExtension = 1;
            public static final double minExtension = 0;

            //INTAKE CONSTANTS
            public static final double intakeClosed = 0.7;
            public static final double intakeOpen = 0.1;
            public static final double intakeDown = 0.55;
            public static final double intakeHover = 0.43;
            public static final double intakeTransfer = 0.12;
        }
    }

    public class Extension implements Action {
        private String direction;
        public Extension(String direction) {
            this.direction = direction;
        }

        public void extendOrRetract(String direction) {
            switch(direction) {
                case "extend":
                    leftExtend.setPosition(Constants.ServoConstants.maxExtension);
                    rightExtend.setPosition(Constants.ServoConstants.maxExtension);
                    break;

                case "retract":
                    leftExtend.setPosition(Constants.ServoConstants.minExtension);
                    rightExtend.setPosition(Constants.ServoConstants.minExtension);
                    break;
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendOrRetract(direction);
            return false;
        }
    }

    public class LiftOrLowerElevator implements Action {
        String action;
        public LiftOrLowerElevator(String action) {
            this.action = action;
        }

        public void liftOrLowerElevator() {
            switch(action) {
                case "lift":
                    elevator.setTargetPosition(-3000);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);

                    if(elevator.getCurrentPosition() <= 3200 && elevator.getCurrentPosition() >= 2800) {
                        elevator.setPower(0);
                    }

                    sleep(200);
                    break;

                case "lower":
                    elevator.setTargetPosition(-100);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(-1);

                    if(elevator.getCurrentPosition() >= -50 && elevator.getCurrentPosition() <= -100) {
                        elevator.setPower(0);
                    }

                    sleep(200);
                    break;
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return false;
        }
    }

    public class GoForPiece implements Action {
        private HuskyLens.Block block;
        public void lookForAndCollectPiece() {
            while(opModeIsActive()) {
                if(huskyLens.blocks().length > 0) {
                    block = huskyLens.blocks()[0];

                    if(block.x < 140) {
                        setPower(-0.34, 0.34, 0.34, -0.34);
                    } else if(block.x > 200) {
                        setPower(0.34, -0.34, -0.34, 0.34);
                    } else {
                        setPower(0.4, 0.4, 0.4, 0.4);

                        while (opModeIsActive()) {
                            if (huskyLens.blocks().length > 0) {
                                block = huskyLens.blocks()[0];
                                if (block.y < 100) {
                                    break;
                                }
                            }
                        }
                        setPower(0, 0., 0, 0);
                    }
                }
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            lookForAndCollectPiece();
            return false;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        leftExtend = hardwareMap.get(Servo.class, "leftExtend");
        rightExtend = hardwareMap.get(Servo.class, "rightExtend");
        intake = hardwareMap.get(Servo.class, "intake");
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        leftExtend.setDirection(Servo.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Pose2d initialPose = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab = drive.actionBuilder(initialPose);

        Extension retract = new Extension("retract");
        Extension extend = new Extension("extend");
        GoForPiece goForPiece = new GoForPiece();
        LiftOrLowerElevator liftElevator = new LiftOrLowerElevator("lift");
        LiftOrLowerElevator lowerElevator = new LiftOrLowerElevator("lower");

        waitForStart();

        Action goToBasket = tab.fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();

        Action goToFirstPiece = tab.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();

        Action goToSecondPiece = tab.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();

        Action goToThirdPiece = tab.endTrajectory().fresh()
                .strafeTo(new Vector2d(0, 0))
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        goToBasket,
                        liftElevator,
                        lowerElevator,
                        goToFirstPiece,
                        goForPiece,
                        goToBasket,
                        liftElevator,
                        lowerElevator,
                        goToSecondPiece,
                        goForPiece,
                        goToBasket,
                        liftElevator,
                        lowerElevator,
                        goToThirdPiece,
                        goForPiece,
                        goToBasket,
                        liftElevator,
                        lowerElevator
                )
        );
    }
}

@Disabled()
class TestingClassTeleop extends LinearOpMode {
    GamepadEx gamepadEx = new GamepadEx(gamepad1);
    DcMotorEx frontLeft, frontRight, backLeft, backRight;
    Servo rightExtend, leftExtend, intake;
    DcMotor elevator;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        rightExtend = hardwareMap.get(Servo.class, "rightExtend");
        leftExtend = hardwareMap.get(Servo.class, "leftExtend");
        intake = hardwareMap.get(Servo.class, "intake");
        elevator = hardwareMap.get(DcMotor.class, "elevator");

        
    }
}
