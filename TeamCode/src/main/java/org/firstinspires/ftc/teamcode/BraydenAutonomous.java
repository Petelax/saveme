package org.firstinspires.ftc.teamcode;

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "BraydenAutonomous", group = "teamcode")
public class BraydenAutonomous extends LinearOpMode {
    DcMotor front_left;
    DcMotor front_right;
    DcMotor back_left;
    DcMotor back_right;
    private Vision vision;
//    BNO055IMU imu;
//    Orientation angles;

    int piecesScored = 0;

//    public boolean isAtPosition() {
//        return front_left.getCurrentPosition() >= front_left.getTargetPosition() &&
//                front_right.getCurrentPosition() >= front_right.getTargetPosition() &&
//                back_left.getCurrentPosition() >= back_left.getTargetPosition() &&
//                back_right.getCurrentPosition() >= back_right.getTargetPosition();
//    }

    public void drive(int millis, double power) {
        front_left.setPower(power);
        front_right.setPower(power);
        back_left.setPower(power);
        back_right.setPower(power);

        sleep(millis);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void turnRight(int millis, double power) {
        front_left.setPower(power);
        back_left.setPower(power);
        front_right.setPower(-power);
        back_right.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }

    public void turnLeft(int millis, double power) {
        front_right.setPower(power);
        back_right.setPower(power);
        front_left.setPower(-power);
        back_left.setPower(-power);

        sleep(millis);

        front_right.setPower(0);
        back_right.setPower(0);
        front_left.setPower(0);
        back_left.setPower(0);
    }

    public void strafeRight(int millis, double power) {
        front_right.setPower(-power);
        back_right.setPower(power);
        front_left.setPower(power);
        back_left.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void strafeLeft(int millis, double power) {
        front_left.setPower(-power);
        back_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(-power);

        sleep(millis);

        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }

    public void auto() {
        vision.update(telemetry);

        strafeRight(500, 0.7);
        drive(2500, 0.5);

        positionToPiece();
    }

    public void positionToPiece() {
        while(opModeIsActive()) {
            sleep(300);
            if(vision.getPiecePosition() == Vision.Position.LEFT) {
                turnLeft(100, 0.5);
            } else if(vision.getPiecePosition() == Vision.Position.RIGHT) {
                turnRight(100, 0.5);
            } else {
                break;
            }
        }
        goAndScorePiece();
        if(piecesScored < 3) {
            positionToPiece();
        }
    }

    public void goAndScorePiece() {
        //Todo Make code to pick up the piece using the encoder-based claws and extending arm.
        piecesScored++;

        /*Put the drive code in a switch statement so that depending on what piece the robot has picked up,
        *it will drive a different distance to the basket to score.*/

       switch(piecesScored) { //Todo Use more accurate drive measurements that are measured on actual field.
           case 1:
               drive(1000, -0.5);
               turnLeft(450, 0.5);
               drive(1000, 0.5);
               break;

           case 2:
               drive(1000, -0.5);
               turnLeft(450, 0.5);
               drive(1000, 0.5);
               break;

           case 3:
               drive(1000, -0.5);
               turnLeft(450, 0.5);
               drive(1000, 0.5);
               break;
       }

       //Todo Put code to score the piece here, under the drive code.
    }

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
//        imu = hardwareMap.get(BNO055IMU.class, "imu");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        imu.initialize(parameters);

        waitForStart();

        vision = new Vision(hardwareMap, telemetry);

        auto();
    }
}
