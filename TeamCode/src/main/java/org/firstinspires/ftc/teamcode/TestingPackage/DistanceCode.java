package org.firstinspires.ftc.teamcode.TestingPackage;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TheoCode.SparkFunOTOSDrive;
@Autonomous(name = "RoadRunnerTuning", group = "teamcode")
public class DistanceCode extends LinearOpMode {

    DcMotor leftFront, rightFront, leftBack, rightBack;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");

        SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, new Pose2d(41.5, 63, Math.PI));
        TrajectoryActionBuilder tab1 = drive.actionBuilder(drive.pose);

        Action forwardTesting = tab1.endTrajectory().fresh()
                        .strafeTo(new Vector2d(51.5, 63))
                        .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                forwardTesting
        ));
    }
}