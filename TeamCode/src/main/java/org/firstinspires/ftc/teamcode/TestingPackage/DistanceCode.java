package org.firstinspires.ftc.teamcode.TestingPackage;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TheoCode.SparkFunOTOSDrive;
@TeleOp(name = "RoadRunnerTuning", group = "teamcode")
public class DistanceCode extends LinearOpMode {

    Servo spin;
    @Override
    public void runOpMode() throws InterruptedException {
        spin = hardwareMap.get(Servo.class, "spin");
        spin.setPosition(spin.getPosition() + 0.2);
        waitForStart();
    }
}