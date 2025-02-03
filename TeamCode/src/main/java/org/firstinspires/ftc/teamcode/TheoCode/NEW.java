package org.firstinspires.ftc.teamcode.TheoCode;

//import com.huskycode.huskylib.HuskyLens;
//import com.huskycode.huskylib.HuskyLensColorBlock;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantsPackage.Constants;


@Config
@Autonomous(name = "NEW", group = "Autonomous")
public class NEW extends LinearOpMode {
    public class lift {
        private DcMotorEx erect;
        public lift(HardwareMap hardwareMap) {
            erect = hardwareMap.get(DcMotorEx.class, "erect");
            erect.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            erect.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class liftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    erect.setPower(1);
                    initialized = true;
                }

                double pos = erect.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    erect.setPower(0);
                    return false;
                }
            }
        }

        public Action liftUp() {
            return new liftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    erect.setPower(-1);
                    initialized = true;
                }

                double pos = erect.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    erect.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }

    private HuskyLens huskyLens;

    public class Extension {
        Servo leftEx, rightEX, intake, spin, rightWR, leftWR, gear;
        DcMotor leftFront,leftBack,rightFront,rightBack;
        ElapsedTime timer = new ElapsedTime();
        double x = 0;
        double y = 0;

        public Extension(HardwareMap hardwareMap) {
            gear = hardwareMap.get(Servo.class, "gear");
            leftWR = hardwareMap.get(Servo.class, "leftWR");
            rightWR = hardwareMap.get(Servo.class, "rightWR");
            spin = hardwareMap.get(Servo.class, "spin");
            intake = hardwareMap.get(Servo.class, "claw");
            leftEx = hardwareMap.get(Servo.class, "leftEx");
            rightEX = hardwareMap.get(Servo.class, "rightEX");
            leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
            leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
            rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
            rightFront = hardwareMap.get(DcMotorEx.class, "right_front");

            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            leftWR.setDirection(Servo.Direction.REVERSE);
            leftEx.setDirection(Servo.Direction.REVERSE);

            huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
        }

        public class Pickup implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
//                telemetry.addData("Pickup Timer", timer.seconds());
                rightEX.setPosition(1);
                leftEx.setPosition(1);
                gear.setPosition(Constants.ServoConstants.gearTransfer);
                telemetry.addData("Gear Position", gear.getPosition());
                telemetry.update();
//                leftWR.setPosition(Constants.ServoConstants.wristHover);
//                rightWR.setPosition(Constants.ServoConstants.wristHover);

                sleep(2500);

                HuskyLens.Block[] blocks;

                while(opModeIsActive()) {
                    blocks = huskyLens.blocks();
                    if(blocks.length > 0) {
                        x = blocks[0].x;
                        y = blocks[0].y;
                        telemetry.addData("X Position", x);
                        telemetry.update();

                        if(x < 140) { //Robot will turn right.
                            rightFront.setPower(0.34);
                            rightBack.setPower(-0.34);
                            leftFront.setPower(-0.34);
                            leftBack.setPower(0.34);


                        } else if(x > 200) {
                            rightFront.setPower(-0.34);
                            rightBack.setPower(0.34);
                            leftFront.setPower(0.34);
                            leftBack.setPower(-0.34);
                        } else {
                            telemetry.addData("Robot", "will drive forward");
                            telemetry.update();

                            rightFront.setPower(0.4);
                            rightBack.setPower(0.4);
                            leftFront.setPower(0.4);
                            leftBack.setPower(0.4);

                            while(opModeIsActive()) {
                                if(huskyLens.blocks() .length > 0) {
                                    y = huskyLens.blocks()[0].y;
                                    if(y < 100) {
                                        rightFront.setPower(0);
                                        rightBack.setPower(0);
                                        leftBack.setPower(0);
                                        leftFront.setPower(0);
                                        break;
                                    }
                                }
                            }

                            intake.setPosition(Constants.ServoConstants.clawOpen);

                            sleep(5000);

                            gear.setPosition(Constants.ServoConstants.gearDown);

                            sleep(1000);

                            intake.setPosition(Constants.ServoConstants.clawClosed);

                            sleep(1000);
                            break;
                        }
                    }
                }


                return false;
            }
        }

        public Action Pickup() {
            timer.reset();
            timer.startTime();
            return new Pickup();
        }

        public class Retract implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightEX.setPosition(0.55);
                leftEx.setPosition(0.55);
                leftWR.setPosition(0.1);
                rightWR.setPosition(0.1);
                gear.setPosition(0);

                return false;
            }
        }

        public Action Retract() {
            return new Retract();
        }

        public class Extend implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                rightEX.setPosition(1.0);
                leftEx.setPosition(1.0);
                leftWR.setPosition(0.3);
                rightWR.setPosition(0.3);
                intake.setPosition(0.2);
                gear.setPosition(0.6);

                return false;
            }
        }

        public Action Extend() {
            return new Extend();
        }


    }




    @Override
    public void runOpMode() {

        // Initialize robot components
        Pose2d initialPose = new Pose2d(41.5, 63, Math.PI);
        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);
        Extension extension = new Extension(hardwareMap);
        lift lift = new lift(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Define trajectories and actions
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(41.5, 63))
                .turn(Math.PI)
                .waitSeconds(3);

        Action Bucket1 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, 55))
                .turnTo(135)
                .waitSeconds(5)
                .build();
        Action Bucket2 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, 55))
                .turnTo(135)
                .waitSeconds(5)
                .build();
        Action Bucket3 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, 55))
                .turnTo(135)
                .waitSeconds(1.5)
                .build();
        Action Bucket4 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, 55))
                .turnTo(135)
                .waitSeconds(1.5)
                .build();
        Action Bucket5 = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(55, 55))
                .turnTo(135)
                .waitSeconds(1.5)
                .build();

        Action PieceOne = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(49.5, 52.5))
                .turnTo(180)
                .waitSeconds(5)
                .build();
        Action PieceTwo = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d( 58.82443405511811, 51.35123185285433))
                .turnTo(180)
                .waitSeconds(2)
                .build();
        Action PieceThree = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d( 58.5,49.5 ))
                .turnTo(-64)
                .waitSeconds(2)
                .build();
        Action SubmersableIntake = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(49.5, 52.5))
                .turnTo(Math.toRadians(90))
                .waitSeconds(2)
                .build();

        // Wait for the start of the match
        while (!isStopRequested() && !opModeIsActive()) {
            // You can add initialization actions here if needed
        }

        if (isStopRequested()) return;

        // Start the match actions
        Actions.runBlocking(
                new SequentialAction(
                        extension.Retract(),
                        lift.liftUp(),
                        Bucket1,
                        extension.Extend(),
                        lift.liftDown(),
//                        PieceOne,
                        extension.Pickup(),
                        extension.Retract(),
//                        lift.liftUp(),
//                        Bucket2,
//                        lift.liftDown(),
                        extension.Extend(),
//                        PieceTwo,
                        extension.Pickup(),
                        extension.Retract(),
//                        lift.liftUp(),
//                        Bucket3,
//                        lift.liftDown(),
                        extension.Extend(),
//                        PieceThree,
                        extension.Pickup(),
                        extension.Retract()
//                        lift.liftUp()
//                        Bucket4
                )
        );

    }

}