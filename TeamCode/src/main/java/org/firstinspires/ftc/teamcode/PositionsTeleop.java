package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop For Robot Positions", group = "teamcode")
public class PositionsTeleop extends LinearOpMode {
    private DcMotor frontLeft, frontRight, backLeft, backRight, elevator;
    private Servo extend, bucket, wristL, wristR, stop;
    private CRServo intakeL, intakeR;
    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.MecanumConstants.backRightMotor);
        elevator = hardwareMap.get(DcMotor.class, "erect");

        extend = hardwareMap.get(Servo.class, "extend");
        bucket= hardwareMap.get(Servo.class, "bucket");
        wristL = hardwareMap.get(Servo.class, "wristL");
        wristR = hardwareMap.get(Servo.class, "wristR");
        stop = hardwareMap.get(Servo.class, "stop");

        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");

        waitForStart();

        while(opModeIsActive()) {
            telemetry.addLine("DcMotor Positions:");
            telemetry.addData("Front Left Encoder Position", frontLeft.getCurrentPosition());
            telemetry.addData("Front Right Encoder Position", frontRight.getCurrentPosition());
            telemetry.addData("Back Left Encoder Position", backLeft.getCurrentPosition());
            telemetry.addData("Back Right Encoder Position", backRight.getCurrentPosition());
            telemetry.addData("Elevator Encoder Position", elevator.getCurrentPosition());

            telemetry.addLine("Servo Positions:");
            telemetry.addData("Extend Servo Position", extend.getPosition());
            telemetry.addData("Bucket Servo Position", bucket.getPosition());
            telemetry.addData("WristL Servo Position", wristL.getPosition());
            telemetry.addData("WristR Servo Position", wristR.getPosition());
            telemetry.addData("Stop Servo Position", stop.getPosition());

            telemetry.addLine("CRServo Powers:");
            telemetry.addData("IntakeL CRServo Power", intakeL.getPower());
            telemetry.addData("IntakeR CRServo Power", intakeR.getPower());

            telemetry.update();
        }
    }
}
