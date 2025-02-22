package org.firstinspires.ftc.teamcode.TheoCode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ConstantsPackage.Constants;

import java.util.concurrent.CompletableFuture;


@TeleOp(name = "basicteliop", group = "teamcode")
public class TeleOP extends OpMode {

    GamepadEx driver;

    org.firstinspires.ftc.teamcode.TheoCode.Drive Drive;
    Servo leftEX, rightEX, intake, spin, rightWR, leftWR, gear, bust, sweeper;
    DcMotor erect;
    HuskyLens huskyLens;
    DistanceSensor distanceSensor;

    boolean slowMode = false;
    boolean slowModeLogic = false;
    boolean clawIsOpen = false;
    boolean intaking = false;
    boolean firstExtension = true;
    boolean clawRaised = false;

    ElapsedTime intakeTimer;
    int intakePhase;
    boolean intakeOpen = false;
    boolean isNotRaising = true;
    boolean notCenter = false;

    public TeleOP() {
        super();
    }


    @Override
    public void init() {
        Drive = new Drive(hardwareMap);

        gear = hardwareMap.get(Servo.class,"gear");
        leftWR = hardwareMap.get(Servo.class,"leftWR");
        rightWR = hardwareMap.get(Servo.class,"rightWR");
        spin = hardwareMap.get(Servo.class, "spin");
        intake = hardwareMap.get(Servo.class, "claw");
        leftEX = hardwareMap.get(Servo.class, "leftEx");
        rightEX = hardwareMap.get(Servo.class, "rightEX");
        erect = hardwareMap.get(DcMotor.class, "erect");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
        bust = hardwareMap.get(Servo.class, "bust");
        sweeper = hardwareMap.get(Servo.class, "sweeper");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        sweeper.setPosition(0);

        leftWR.setDirection(Servo.Direction.REVERSE);
        leftEX.setDirection(Servo.Direction.REVERSE);

        driver = new GamepadEx(gamepad1);

        intakeTimer = new ElapsedTime();
        intakePhase = 10;

        gear.setPosition(Constants.ServoConstants.gearDown);
        setWrist(Constants.ServoConstants.wristTransfer);
        setExtension(Constants.ServoConstants.minExtension);
        spin.setPosition(Constants.ServoConstants.spinCenter);
        intake.setPosition(Constants.ServoConstants.clawClosed);

        erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        erect.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        erect.setTargetPosition(-122);
        erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        erect.setPower(1);

        if(erect.getCurrentPosition() >= -122) {
            erect.setPower(0);
        }
    }

    @Override
    public void loop() {


        //Reads buttons
        driver.readButtons();

        if(driver.wasJustPressed(GamepadKeys.Button.Y)) {
            clawIsOpen = !clawIsOpen;
            if(intake.getPosition() == Constants.ServoConstants.clawClosed) {
                intake.setPosition(Constants.ServoConstants.clawOpen);
            } else {
                intake.setPosition(Constants.ServoConstants.clawClosed);
            }
            telemetry.addData("Claw Open", clawIsOpen);
            telemetry.update();
        }

        //WHen pressing A, toggles between intaking and transferring.
        if(driver.wasJustPressed(GamepadKeys.Button.A)) {
            intaking = !intaking;
            intakeTimer.reset();
            intakePhase = 0;
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
        }

        if(gamepad1.dpad_up) {
            bust.setPosition(0.15);
        } else {
            bust.setPosition(1);
        }

        if(intaking) {

            switch(intakePhase) {
                //Shoots out extension, leaves claw closed for outtaking
                case 0:

                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristHover);
                    gear.setPosition(Constants.ServoConstants.gearDown);
                    spin.setPosition(Constants.ServoConstants.spinCenter);
                    clawIsOpen = false;

                    intakePhase += intakeTimer.seconds() >= 0.3 ? 1 : 0;
                    break;

                //Opens claw and hovers, waits for driver to press B before intaking deez nuts kekw
                case 1:

                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristHover);
                    gear.setPosition(Constants.ServoConstants.gearDown);

//                    if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) && spin.getPosition() >= 0.1){
//                        spin.setPosition(spin.getPosition() - 0.2);
//                    }
//
//                    if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT) && spin.getPosition() <= 0.9){
//                        spin.setPosition(spin.getPosition() + 0.2);
//                    }

                    if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                        spin.setPosition(spin.getPosition() + 0.2);
                    }

                    if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                        spin.setPosition(spin.getPosition() -0.2);
                    }

                    if(firstExtension) {
                        clawIsOpen = true;
                    } else {
                        clawIsOpen = false;
                    }

                    if(driver.wasJustPressed(GamepadKeys.Button.B)){
                        huskyLens.selectAlgorithm(HuskyLens.Algorithm.NONE);
                        intakeTimer.reset();
                        intakePhase = 2;
                    }
                    break;
                case 2:
                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristHover - 0.25);
                    gear.setPosition(Constants.ServoConstants.gearDownDown - 0.19);
                    clawIsOpen = true;
                    intakePhase += intakeTimer.seconds() > 0.25 ? 1 : 0;
                    //Slams the claw into the piece and waits for a moment to close claw
                case 3:

                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristDown);
                    gear.setPosition(Constants.ServoConstants.gearDownDown);

                    intakePhase += intakeTimer.seconds() > 0.75 ? 1 : 0;
                    break;

                //closes the claw and waits a moment before raising the arm
                case 4:

                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristDown);
                    gear.setPosition(Constants.ServoConstants.gearDownDown);
                    clawIsOpen = false;

                    intakePhase += intakeTimer.seconds() > 1.25 ? 1 : 0;
                    break;

                //raises the arm and waits for driver to press A to retract the intake, or
                // press B to intake again
                case 5:

                    setExtension(Constants.ServoConstants.maxExtension);
                    setWrist(Constants.ServoConstants.wristHover);
                    gear.setPosition(Constants.ServoConstants.gearDown);
                    clawIsOpen = false;
                    firstExtension = false;

                    intakePhase = 1;
                    break;

            }

            if(clawIsOpen) {
                intake.setPosition(Constants.ServoConstants.clawOpen);
            } else {
                intake.setPosition(Constants.ServoConstants.clawClosed);
            }

        } else {

            firstExtension = true;
            setExtension(Constants.ServoConstants.minExtension);
            if(isNotRaising) {
                setWrist(Constants.ServoConstants.wristTransfer);
            }
            gear.setPosition(Constants.ServoConstants.gearTransfer);
            spin.setPosition(Constants.ServoConstants.spinCenter);

            if(driver.wasJustPressed(GamepadKeys.Button.Y)) {
                intake.setPosition(intake.getPosition() == Constants.ServoConstants.clawClosed ? Constants.ServoConstants.clawOpen : Constants.ServoConstants.clawClosed);
            }

        }
// MANUAL COMMANDS ARE HERE
//        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
//            setWrist(leftWR.getPosition()+0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
//            setWrist(leftWR.getPosition()-0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)){
//            setExtension(leftEX.getPosition()+0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
//            setExtension(leftEX.getPosition()-0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.X)) {
//            gear.setPosition(gear.getPosition()+0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.Y)) {
//            gear.setPosition(gear.getPosition()-0.05);
//        }
//
        if(driver.wasJustPressed(GamepadKeys.Button.A)) {
            spin.setPosition(spin.getPosition()+0.05);
        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.B)) {
//            spin.setPosition(spin.getPosition()-0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//            intake.setPosition(intake.getPosition()+0.05);
//        }
//
//        if(driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
//            intake.setPosition(intake.getPosition()-0.05);
//        }

        Drive.teleop(gamepad1, slowMode);
        Drive.periodic(telemetry);

        if (gamepad1.right_bumper) {
            isNotRaising = false;
            CompletableFuture.runAsync(() -> {
               intake.setPosition(Constants.ServoConstants.clawOpen);
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                leftWR.setPosition(Constants.ServoConstants.wristHover);
                rightWR.setPosition(Constants.ServoConstants.wristHover);
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                erect.setTargetPosition(-3000);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(1);
            });
        } else if (gamepad1.left_bumper) {
            //Position: -120

            CompletableFuture.runAsync(() -> {
                erect.setTargetPosition(-55);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                erect.setPower(-1);

                clawRaised = true;

//                while(true) {
//                    if(erect.getCurrentPosition() >= erect.getTargetPosition()) {
//                        break;
//                    }
//                }
//
//                gear.setPosition(Constants.ServoConstants.gearTransfer);
//                isNotRaising = true;

            });

        }

        if(gamepad1.x) {
            CompletableFuture.runAsync(() -> {
                isNotRaising = false;
                leftWR.setPosition(Constants.ServoConstants.wristHover);
                rightWR.setPosition(Constants.ServoConstants.wristHover);

                    try {
                        Thread.sleep(600);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                erect.setTargetPosition(-2200);
                erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if(erect.getCurrentPosition() < -2300 && erect.getCurrentPosition() > 2100) {
                    erect.setPower(0);
                } else {
                    erect.setPower(1);
                }
            });
        }

        if(clawRaised && erect.getCurrentPosition() >= erect.getTargetPosition()) {
            CompletableFuture.runAsync(() -> {
                gear.setPosition(Constants.ServoConstants.gearTransfer);
                isNotRaising = true;
                clawRaised = false;
            });
        }

        if (gamepad1.dpad_down && !slowModeLogic) {
            slowMode = !slowMode;
            slowModeLogic = true;
        } else if (!gamepad1.dpad_down) {
            slowModeLogic = false;
        }



        telemetry.addLine("DEBUGGING");
        telemetry.addData("SPIN POSITION", spin.getPosition());
        telemetry.addData("WRIST POSITION", leftWR.getPosition());
        telemetry.addData("GEAR POSITION", gear.getPosition());
        telemetry.addData("INTAKE POSITION", intake.getPosition());
        telemetry.addData("EXTENSION POSITION", leftEX.getPosition());
        telemetry.addData("INTAKE PHASES", intakePhase);
        telemetry.addData("UNTAKE TIMER", intakeTimer.seconds());
        telemetry.addData("erect Position", erect.getCurrentPosition());
        telemetry.update();

    }

    public void setExtension(double extend) {
        leftEX.setPosition(extend);
        rightEX.setPosition(extend);
    }

    public void setWrist(double wrist) {
        leftWR.setPosition(wrist);
        rightWR.setPosition(wrist);

        telemetry.addData("Wrist R", rightWR.getPosition());
        telemetry.addData("Wrist L", leftWR.getPosition());
        telemetry.update();
    }

    public void measureWidth() {
        if(huskyLens.blocks().length > 0) {
            telemetry.addData("Block Width", huskyLens.blocks()[0].width);
            telemetry.update();
        }
    }
    public void rotateToPiece() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
        if(huskyLens.blocks().length > 0) {
            if(huskyLens.blocks()[0].id == 1) {
                telemetry.addData("Piece", "is vertical.");
            } else if(huskyLens.blocks()[0].id == 2) {
                telemetry.addData("Piece", "is horizontal.");
            } else if(huskyLens.blocks()[0].id == 3) {
                telemetry.addData("Piece", "is DIagonal first way.");
            } else if(huskyLens.blocks()[0].id == 4) {
                telemetry.addData("Piece", "is diagonal other way.");
            }
            //telemetry.update();
        }

    }

}
