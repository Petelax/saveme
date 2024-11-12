package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "BraydenTeleop", group = "teamcode")
public class BraydenTeleop extends LinearOpMode {

     DcMotor front_left;
     DcMotor front_right;
     DcMotor back_left;
     DcMotor back_right;

//     DcMotor elevator;
//     Servo  extend, wrist, bucket, side;
//     CRServo intake;
//     TouchSensor bottom;

    private double front_left_power;
    private double front_right_power;
    private double back_left_power;
    private double back_right_power;
    double x;
    double y;
    double rx;

    public void teleop() {
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;

        front_left_power = x + y + rx;
        front_right_power = -x + y - rx;
        back_left_power = -x + y + rx;
        back_right_power = x + y - rx;

        double maxPower = Math.max(1.0, Math.abs(front_left_power));
        maxPower = Math.max(maxPower, Math.abs(front_right_power));
        maxPower = Math.max(maxPower, Math.abs(back_left_power));
        maxPower = Math.max(maxPower, Math.abs(back_right_power));

        front_left_power /= maxPower;
        front_right_power /= maxPower;
        back_left_power /= maxPower;
        back_right_power /= maxPower;

        front_left.setPower(front_left_power);
        front_right.setPower(front_right_power);
        back_left.setPower(back_left_power);
        back_right.setPower(back_right_power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        teleop();
    }
}
