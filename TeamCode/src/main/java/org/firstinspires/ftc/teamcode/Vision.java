package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Vision {

    HuskyLens vision;
    ElapsedTime elapsedTime;
    BraydenAuto auto = new BraydenAuto();
    HuskyLens.Block[] blocks;
    int phase = 0;
    //   IMU imu;
    boolean isLeft = false;
    boolean isRight = false;
    boolean isMiddle = false;
    boolean initiated = true;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
        vision = hardwareMap.get(HuskyLens.class, "vision");
        elapsedTime = new ElapsedTime();

        telemetry.addData(">>", vision.knock() ? "Touch start to continue" : "Problem with HuskyLens connection");
        telemetry.update();
//       imu = hardwareMap.get(IMU.class, "imu");
//       IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
//       imu.initialize(parameters);
    }

    public void update(Telemetry telemetry, BraydenAuto auto) {
        auto.sleep(280);
        isRight = false;
        isMiddle = false;
        isLeft = false;

        // Fetch the latest blocks
        blocks = vision.blocks();

        // Check if blocks array is valid and has data
        if (blocks != null && blocks.length > 0) {
            telemetry.addData("Value", "Yes");
            telemetry.addData("X", blocks[0].x); // Print x-coordinate of the first block
            telemetry.addData("Y", blocks[0].y); // Print y-coordinate of the first block
        } else {
            telemetry.addData("Value", "None");
        }
//
        if(blocks != null && blocks.length > 0) {
            getPiece(telemetry);
        }

        if(isRight) {
            telemetry.addData("Robot", "will drive right.");
            initiated = false;
            auto.turnRight(25, 0.5);
//            initiated = true;
            auto.runVision();
//           isRight = false;
//           getPiece();
        } else if(isLeft) {
            telemetry.addData("Robot", "will drive left.");
            initiated = false;
            auto.turnLeft(25, 0.5);
//            initiated = true;
            auto.runVision();
//           isLeft = false;
//           getPiece();
        } else if(isMiddle) {
            telemetry.addData("Robot", "will drive forward");
            initiated = false;
//           auto.sleep(1000);
            auto.driveToPiece();
        }

        telemetry.update();
    }

    public void getPiece(Telemetry telemetry) {
        if(blocks[0].x < 130) {
            isLeft = true;
        } else if(blocks[0].x >= 280) {
            isRight = true;
        } else {
            isMiddle = true;
        }
        telemetry.addData("IsRight", isRight);
        telemetry.addData("isLeft", isLeft);
        telemetry.addData("isMiddle", isMiddle);
        telemetry.update();
    }


}