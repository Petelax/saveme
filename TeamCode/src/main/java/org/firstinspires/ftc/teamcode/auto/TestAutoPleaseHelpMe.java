package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ConstantsPackage.Constants;
import org.firstinspires.ftc.teamcode.TheoCode.Drive;

@Autonomous
public class TestAutoPleaseHelpMe extends OpMode {
    Drivetrain drive;
    Servo leftEX, rightEX, intake, spin, rightWR, leftWR, gear, bust, sweeper;
    DcMotor erect;
    HuskyLens huskyLens;

    @Override
    public void init() {
        drive = new Drivetrain(hardwareMap);

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

        sweeper.setPosition(0);

        leftWR.setDirection(Servo.Direction.REVERSE);
        leftEX.setDirection(Servo.Direction.REVERSE);

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
    public void start() {
        new P2P(drive, new Pose2d(24.0, 0.0, new Rotation2d()));
    }

    @Override
    public void loop() {

        CommandScheduler.getInstance().run();
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

}
