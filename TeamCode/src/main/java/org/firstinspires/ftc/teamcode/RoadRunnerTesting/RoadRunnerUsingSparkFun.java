package org.firstinspires.ftc.teamcode.RoadRunnerTesting;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantsPackage.Constants;
import org.firstinspires.ftc.teamcode.TheoCode.SparkFunOTOSDrive;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous(name = "RoadRunnerUsingSparkFun", group = "teamcode")
public class RoadRunnerUsingSparkFun extends LinearOpMode {

    public class DriveToPiece implements Action {

        double power;
        int millis;
        String direction;

        public DriveToPiece(double power, int millis, String direction) {
            this.power = power;
            this.millis = millis;
            this.direction = direction;
        }

        public void drive(String direction, double power, int millis) {
            switch(direction) {
                case "forward":
                    rightFront.setPower(power);
                    rightBack.setPower(power);
                    leftFront.setPower(power);
                    leftBack.setPower(power);
                    sleep(millis);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    break;

                case "backward":
                    rightFront.setPower(-power);
                    rightBack.setPower(-power);
                    leftFront.setPower(-power);
                    leftBack.setPower(-power);
                    sleep(millis);
                    rightFront.setPower(0);
                    rightBack.setPower(0);
                    leftFront.setPower(0);
                    leftBack.setPower(0);
                    break;
            }
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            drive(direction, power, millis);
            return false;
        }
    }

    public class LiftOrLowerElevator implements Action {
        String position;
        public LiftOrLowerElevator(String position) {
            this.position = position;
        }

        public void raiseOrLower(String position) {
            switch(position) {
                case "raise":
                    elevator.setTargetPosition(-2960); //-2790
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(1);

                    sleep(1500);

                    bucketServo.setPosition(0);

                    sleep(1500);
                    break;

                case "lower":
                    telemetry.addData("Ran", "this");
                    telemetry.update();
                    bucketServo.setPosition(1);

                    sleep(800);

                    elevator.setTargetPosition(-110);
                    elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevator.setPower(-1);

                    sleep(1400);

                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            raiseOrLower(position);
            return false;
        }
    }

    public class ExtensionAndRetraction implements Action {

        String position;

        public ExtensionAndRetraction(String position) {
            this.position = position;
        }

        public void extendOrRetract(String position) {
            switch(position) {
                case "extend":
                    leftExtend.setPosition(1);
                    rightExtend.setPosition(1);
                    break;

                case "retract":
                    leftExtend.setPosition(0.55);
                    rightExtend.setPosition(0.55);
                    break;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            extendOrRetract(position);
            return false;
        }
    }

    public class SearchForPiece implements Action {

        public void setPower(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
            leftFront.setPower(frontLeftPower);
            rightFront.setPower(frontRightPower);
            leftBack.setPower(backLeftPower);
            rightBack.setPower(backRightPower);
        }

        public void searchForPiece() {
            //        while(opModeIsActive()) {
//            if(huskyLens.blocks().length > 0) {
//                HuskyLens.Block block = huskyLens.blocks().length == 0 ? null : huskyLens.blocks()[0];
//                assert block != null;
//                telemetry.addData("Block X", block.x);
//
//               if(block.x < 160) {
//                   telemetry.addData("Block", "is too far to the left.");
////                   leftFront.setPower(-0.35);
////                   rightFront.setPower(0.35);
////                   leftBack.setPower(0.35);
////                   rightBack.setPower(-0.35);
//               } else if(block.x > 170) {
//                   telemetry.addData("Block", "is too far to the right.");
////                   leftFront.setPower(0.35);
////                   rightFront.setPower(-0.35);
////                   leftBack.setPower(-0.35);
////                   rightBack.setPower(0.35);
//               } else if(block.x > 160 && block.x < 170) {
////                   leftWrist.setPosition(0.27);
////                   rightWrist.setPosition(0.27);
//                   telemetry.addData("Block is centered! --> " + block.x, "Second Alignment beginning shortly.");
//                   telemetry.update();
//                   sleep(5000);
//
//                   while(opModeIsActive()) {
//                       if(huskyLens.blocks().length > 0) {
//                           block = huskyLens.blocks().length == 0 ? null : huskyLens.blocks()[0];
//                           assert block != null;
//                           telemetry.addData("Block Pos", block.y);
//                           telemetry.update();
//                           if(block.y > 200  ) {
//                               break;
//                           }
//                       }
//                   }
//
//                   while(opModeIsActive()) {
//                       if(huskyLens.blocks().length > 0) {
//                           block = huskyLens.blocks().length ==  0 ? null : huskyLens.blocks()[0];
//                           assert block != null;
//                           telemetry.addData("Block 2nd X", block.x);
//
//                           if(block.x < 152) { //138
//                               telemetry.addData("Block", "is too far to the left.");
//                           } else if(block.x > 162) { //146
//                               telemetry.addData("Block", "is too far to the right.");
//                           } else {
//                               telemetry.addData("Block", "is centered!");
//                               telemetry.update();
//
//                               sleep(100000);
//                           }
//
//                           telemetry.addData("Block Y", block.y);
//                       }
//
//                       telemetry.update();
//                   }
//
////                   rightFront.setPower(0);
////                   rightBack.setPower(0);
////                   leftFront.setPower(0);
////                   leftBack.setPower(0);
//
//                   telemetry.addData("Piece", "centered!");
//                   telemetry.update();
//                   sleep(5000);
//               }
//
//               telemetry.update();
//            }
//        }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            rightExtend.setPosition(1);
            leftExtend.setPosition(1);
            gear.setPosition(Constants.ServoConstants.gearDown);
            intake.setPosition(Constants.ServoConstants.clawOpen);

            HuskyLens.Block block;

//            while(opModeIsActive()) {
//               if(huskyLens.blocks().length > 0) {
//                   block = huskyLens.blocks()[0];
//                   telemetry.addData("Block X", block.x);
//                   telemetry.addData("Block Y:", block.y);
//                   telemetry.update();
//
//                   sleep(5000);
//
//                   if(block.x < 160) { //140//0.34
//                       setPower(-0.28, 0.28, 0.28, -0.28);
//                   } else if(block.x > 170) {
//                       setPower(0.28, -0.28, -0.28, 0.28);
//                   } else {
//                     setPower(0.3,0.3, 0.3, 0.3);
//
//                     while(opModeIsActive()) {
//                       if(huskyLens.blocks().length > 0)  {
//                           block = huskyLens.blocks()[0];
//                           telemetry.addData("Y", block.y);
//                           telemetry.update();
//                           if(block.y > 180) {
//                               break;
//                           }
//                       }
//                     }
//
//                     setPower(0, 0, 0, 0);
//
//                     while(opModeIsActive()) {
//                         if(huskyLens.blocks().length > 0) {
//                             block = huskyLens.blocks().length == 0 ? null : huskyLens.blocks()[0];
//
//                             assert block != null;
//                             if(block.x < 152) {
//                                 rightFront.setPower(0.28);
//                                 rightBack.setPower(0.28);
//                                 leftFront.setPower(-0.28);
//                                 leftBack.setPower(-0.28);
//                             } else if(block.x > 162) {
//                                 rightFront.setPower(-0.28);
//                                 rightBack.setPower(-0.28);
//                                 leftFront.setPower(0.28);
//                                 leftBack.setPower(0.28);
//                             } else {
//                                 break;
//                             }
//                         }
//                     }
//
//                     gear.setPosition(Constants.ServoConstants.gearDownDown);
//                     sleep(500);
//
//                     leftWrist.setPosition(Constants.ServoConstants.wristDown);
//                     rightWrist.setPosition(Constants.ServoConstants.wristDown);
//                     sleep(500);
//
//                     intake.setPosition(Constants.ServoConstants.clawClosed);
//                     sleep(500);
//
//                     leftWrist.setPosition(Constants.ServoConstants.wristTransfer);
//                     rightWrist.setPosition(Constants.ServoConstants.wristTransfer);
//                     sleep(200);
//
//                     leftExtend.setPosition(Constants.ServoConstants.minExtension);
//                     rightExtend.setPosition(Constants.ServoConstants.minExtension);
//                     gear.setPosition(Constants.ServoConstants.gearTransfer);
//                     sleep(500);
//
//                     intake.setPosition(Constants.ServoConstants.clawOpen);
//                     break;
//                   }
//               }
//            }

            searchForPiece();
            return false;
        }
    }

    DcMotorEx leftFront, leftBack, rightFront, rightBack;
    DcMotor elevator;
    HuskyLens huskyLens;
    Servo leftExtend, rightExtend, bucketServo, gear, intake, leftWrist, rightWrist, spin, sweeper;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    AtomicBoolean isRaised = new AtomicBoolean(false);

    public void runAsync() {
        isRaised.set(false);

        gear.setPosition(Constants.ServoConstants.gearDown);
        rightExtend.setPosition(Constants.ServoConstants.maxExtension);
        leftExtend.setPosition(Constants.ServoConstants.maxExtension);
        leftWrist.setPosition(Constants.ServoConstants.wristHover);
        rightWrist.setPosition(Constants.ServoConstants.wristHover);
        spin.setPosition(Constants.ServoConstants.spinCenter);

        sleep(500);

        elevator.setTargetPosition(-2960); //-2790
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(1);

        while(opModeIsActive()) {
            if(elevator.getCurrentPosition() >= elevator.getTargetPosition() -50) {
                break;
            }
        }

        sleep(1000);

        bucketServo.setPosition(0);

        sleep(1500);

        bucketServo.setPosition(1);

        sleep(300);

        elevator.setTargetPosition(-110);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(-1);

        isRaised.set(true);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        elevator = hardwareMap.get(DcMotor.class, "erect");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        leftExtend = hardwareMap.get(Servo.class, "leftEx");
        rightExtend = hardwareMap.get(Servo.class, "rightEX");
        bucketServo = hardwareMap.get(Servo.class, "bust");
        gear = hardwareMap.get(Servo.class, "gear");
        intake = hardwareMap.get(Servo.class, "claw");
        leftWrist = hardwareMap.get(Servo.class, "leftWR");
        rightWrist = hardwareMap.get(Servo.class, "rightWR");
        spin = hardwareMap.get(Servo.class, "spin");
        sweeper = hardwareMap.get(Servo.class, "sweeper");

        elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftExtend.setDirection(Servo.Direction.REVERSE);
        leftWrist.setDirection(Servo.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d initialPose = new Pose2d(41.9, 63, Math.PI);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose);

        LiftOrLowerElevator raiseElevator = new LiftOrLowerElevator("raise");
        LiftOrLowerElevator lowerElevator = new LiftOrLowerElevator("lower");
        ExtensionAndRetraction retract = new ExtensionAndRetraction("retract");
        ExtensionAndRetraction extend = new ExtensionAndRetraction("extend");
        SearchForPiece searchForPiece = new SearchForPiece();

        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        telemetry.addData("HuskyLens Initialized", huskyLens.knock());
        telemetry.update();

        waitForStart();

        Action goToBasket = tab1.endTrajectory().fresh()
//                .strafeTo(new Vector2d(51.6544, 54.9196)) //Y: 54.9196, X: 50.6544
//                .strafeTo(new Vector2d(49.6812, 51.4353))
                .strafeTo(new Vector2d(47, 51))
                .turnTo(-141)
                .build();

        Action goToFirstPiece = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(39, 53))
                .waitSeconds(0.5)
                .turnTo(-108.2257)
                .build();
//
////        new Vector2d(39.1923, 55.0998)
//
        Action goToSecondPiece = tab1.endTrajectory().fresh()
                .splineTo(new Vector2d(0, 0), Math.toRadians(35))
                .build();

        Action goToThirdPiece = tab1.endTrajectory().fresh()
                .splineTo(new Vector2d(0, 0), Math.toRadians(35))
                .build();

//        CompletableFuture.runAsync(() -> {
//            while(opModeIsActive()) {
//                telemetry.addData("Elevator Pos", elevator.getCurrentPosition());
//                telemetry.update();
//            }
//        });

        Actions.runBlocking(
                new SequentialAction(
                        t -> {
                            CompletableFuture.runAsync(this::runAsync);
                            return false;
                        },
                        goToBasket,
                        t -> {
                            Log.i(RoadRunnerUsingSparkFun.class.getName(), "IsRaised: " + isRaised.get());
                            while (opModeIsActive()) {
                                if (isRaised.get()) {
                                    break;
                                } else {
                                    telemetry.addData("Elevator", "has not raised");
                                    telemetry.update();
                                }
                            }

                            telemetry.addData("Elevator Raised", "true");
                            telemetry.update();
                            return false;
                        },
                        goToFirstPiece,
                        searchForPiece,
                        t ->  {CompletableFuture.runAsync(this::runAsync); return false;},
                        goToBasket,
                        goToSecondPiece,
                        searchForPiece,
                        t ->  {CompletableFuture.runAsync(this::runAsync); return false;},
                        goToBasket,
                        goToThirdPiece,
                        searchForPiece,
                        t ->  {CompletableFuture.runAsync(this::runAsync); return false;},
                        goToBasket

                )
        );
    }
}
