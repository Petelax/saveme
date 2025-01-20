package org.firstinspires.ftc.teamcode;

//
import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class StopMotors extends RoadRunnerTestingUsingDcMotorEx implements Action {
    public StopMotors(HardwareMap hardwareMap, Telemetry telemetry) {
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");

        telemetry.addLine("StopMotors Class");
        telemetry.addData("rightFront initialized", rightFront != null);
        telemetry.addData("rightBack initialized", rightBack != null);
        telemetry.addData("leftFront initialized", leftFront != null);
        telemetry.addData("leftBack initialized", leftBack != null);
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        rightFront.setPower(0);
        rightBack.setPower(0);
        leftFront.setPower(0);
        leftBack.setPower(0);
        return false;
    }
}

class Drive extends RoadRunnerTestingUsingDcMotorEx implements Action {

    int ticksPerSecond;
    String direction;
    Telemetry telemetry;
    public Drive(Telemetry telemetry, HardwareMap hardwareMap, int ticksPerSecond, String direction) {
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");

        double[] pid = Constants.DriveConstants.pidCoefficients;

        this.ticksPerSecond = ticksPerSecond;
        this.direction= direction;
        this.telemetry = telemetry;

        telemetry.addLine("Drive Motors");
        telemetry.addData("rightFront initialized", rightFront != null);
        telemetry.addData("rightBack initialized", rightBack != null);
        telemetry.addData("leftFront initialized", leftFront != null);
        telemetry.addData("leftBack initialized", leftBack != null);

        rightFront.setVelocityPIDFCoefficients(pid[0], pid[1], pid[2], pid[3]);
        rightBack.setVelocityPIDFCoefficients(pid[0], pid[1], pid[2], pid[3]);
        leftFront.setVelocityPIDFCoefficients(pid[0], pid[1], pid[2], pid[3]);
        leftBack.setVelocityPIDFCoefficients(pid[0], pid[1], pid[2], pid[3]);
    }

    public void resetEncoders() {
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void setVelocity(int velocity, int leftDirection, int rightDirection) {
        if(leftDirection == 2) {
            leftFront.setVelocity(-rightDirection);
            leftBack.setVelocity(rightDirection);
            rightFront.setVelocity(rightDirection);
            rightBack.setVelocity(-rightDirection);
            return;

        } else if(rightDirection == 2) {
            leftFront.setVelocity(leftDirection);
            leftBack.setVelocity(-leftDirection);
            rightFront.setVelocity(-leftDirection);
            rightBack.setVelocity(leftDirection);
            return;

        }

        rightBack.setVelocity(velocity * rightDirection);
        rightFront.setVelocity(velocity * rightDirection);
        leftFront.setVelocity(velocity * leftDirection);
        leftBack.setVelocity(velocity * leftDirection);
    }

    public void drive(String direction) {

        switch(direction) {
            case "forward":
                setVelocity(ticksPerSecond,1, 1);
                break;

            case "backward":
                setVelocity(ticksPerSecond, -1, -1);
                break;

            case "left":
                setVelocity(ticksPerSecond, -1, 1);
                break;

            case "right":
                setVelocity(ticksPerSecond, 1, -1);
                break;

            case "strafe left":
                setVelocity(ticksPerSecond, 2, ticksPerSecond);
                break;

            case "strafe right":
                setVelocity(ticksPerSecond, ticksPerSecond, 2);
                break;
        }
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        drive(direction);
        return false;
    }
}

@Autonomous(name = "RoadRunnerTestingUsingDcMotorEx", group = "teamcode")
public class RoadRunnerTestingUsingDcMotorEx extends LinearOpMode implements Action {
    DcMotorEx rightFront, rightBack, leftFront, leftBack;
    int roadRunnerSetPoint;
    String action;
    HuskyLens huskyLens;
    public RoadRunnerTestingUsingDcMotorEx(String action, int setPoint, HardwareMap hardwareMap, Telemetry telemetry) {
        this.action = action;
        roadRunnerSetPoint = setPoint;
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");

        telemetry.addLine("RoadRunnerTestingUsingDcMotorEx");
        telemetry.addData("rightFront initialized", rightFront != null);
        telemetry.addData("rightBack initialized", rightBack != null);
        telemetry.addData("leftFront initialized", leftFront != null);
        telemetry.addData("leftBack initialized", leftBack != null);
    }

    public RoadRunnerTestingUsingDcMotorEx() {

    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        switch(action) {
            case "drive sensing":
                while(true) {
                    Log.i(RoadRunnerTestingUsingDcMotorEx.class.getName(), "RightBack Position: " + rightBack.getCurrentPosition());
                    if(rightBack.getCurrentPosition() >= roadRunnerSetPoint * 0.85) {
                        break;
                    }
                }
                break;

            case "vision sensing":
                long currentTime = System.currentTimeMillis();
                while(System.currentTimeMillis() - currentTime < 20) {
                    if(huskyLens.blocks().length > 0) {
                        Log.i(RoadRunnerTestingUsingDcMotorEx.class.getName(), "Block Info: " + huskyLens.blocks()[0].toString());
                    }
                }
                break;

            case "stop sensing":
                while(true) {
                    Log.i(RoadRunnerTestingUsingDcMotorEx.class.getName(), "RightBack Position: " + rightBack.getCurrentPosition());
                    if(rightBack.getCurrentPosition() >= roadRunnerSetPoint) {
                        break;
                    }
                }
                break;
        }
        return false;
    }

    @Override
    public void runOpMode() {
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");

        telemetry.addLine("RoadRunnerTestingUsingDcMotorEx");
        telemetry.addData("rightFront initialized", rightFront != null);
        telemetry.addData("rightBack initialized", rightBack != null);
        telemetry.addData("leftFront initialized", leftFront != null);
        telemetry.addData("leftBack initialized", leftBack != null);

        Drive drive = new Drive(telemetry, hardwareMap, 1000, "forward");
        Drive slowDown = new Drive(telemetry, hardwareMap, 20, "forward");
        RoadRunnerTestingUsingDcMotorEx roadRunner = new RoadRunnerTestingUsingDcMotorEx("drive sensing", 8000, hardwareMap, telemetry);
        RoadRunnerTestingUsingDcMotorEx slowing = new RoadRunnerTestingUsingDcMotorEx("stop sensing", 8000, hardwareMap, telemetry);
        StopMotors stopMotors = new StopMotors(hardwareMap, telemetry);

        telemetry.update();

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        (t) -> {drive.resetEncoders();
                            telemetry.addData("Roadrunner set point", roadRunnerSetPoint);
                            telemetry.update();
                            sleep(3000); return false;},
                        drive,
                        roadRunner,
                        slowDown,
                        slowing,
                        stopMotors
                )
        );

    }
}