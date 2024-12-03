package org.firstinspires.ftc.teamcode;

import  com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;


//
@TeleOp(name = "basicteliop", group = "teamcode")
public class bruh extends OpMode {

    drive driver ;
    DcMotor erect;
    Servo extend, wristL, wristR, bucket, stop;
    CRServo intakeL, intakeR;
    TouchSensor bottom;
    ColorSensor colorSensor;
    BraydenAuto auto = new BraydenAuto();
    LED led;

    public bruh() {
        super();
    }

    @Override
    public void init() {
        driver = new drive(hardwareMap);

        extend = hardwareMap.get(Servo.class, "extend");
        wristL = hardwareMap.get(Servo.class, "wristL");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        bottom = hardwareMap.get(TouchSensor.class, "bottom");
        erect = hardwareMap.get(DcMotor.class, "erect");
        bucket = hardwareMap.get(Servo.class, "bucket");
        stop = hardwareMap.get(Servo.class, "stop");
        wristR = hardwareMap.get(Servo.class,"wristR");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        led = hardwareMap.get(LED.class, "LED");

        telemetry.addData("WristL Position", wristL.getPosition());
        telemetry.addData("WristR Position", wristR.getPosition());
        telemetry.update();

    }


    @Override
    public void loop() {

        telemetry.addData("Is Light On", led.getDeviceName());

        driver.teleop(gamepad1,false);
        telemetry.addData("WristL Position", wristL.getPosition());
        telemetry.addData("WristR Position", wristR.getPosition());
        driver.periodic(telemetry);

        telemetry.addData("Current Position", erect.getCurrentPosition());
        //erect.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //erect.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (gamepad2.right_bumper) {
            erect.setTargetPosition(2500);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (erect.getCurrentPosition() < 2600 && erect.getCurrentPosition() > 2300) {
                erect.setPower(0);
            } else {
                erect.setPower(1);

            }


        } else if (gamepad2.left_bumper) {

            erect.setTargetPosition(100);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (erect.getCurrentPosition() < 200 && erect.getCurrentPosition() > 50) {
                erect.setPower(0);
            } else {
                erect.setPower(-1);
                //erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            telemetry.addData("Status", "Motor Running");

        } else if (gamepad2.x) {
            erect.setTargetPosition(1100);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (erect.getCurrentPosition() < 1050 && erect.getCurrentPosition() > 950) {
                erect.setPower(0);

            } else {
                erect.setPower(-1);
            }

        }
//            if() { //TODO This is going to be the color sensor if statement.
//
//            }



        if (bottom.isPressed()) {
            erect.setPower(0);

        } else {

        }

        if(gamepad1.dpad_up) {
            erect.setTargetPosition(2500);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            erect.setPower(1);

            extend.setPosition(0);

            if (erect.getCurrentPosition() > 2300 && erect.getCurrentPosition() <= 2500) {
                erect.setPower(0);
            }
            wristL.setPosition(-0.04);
            wristR.setPosition(1.4);
            auto.sleep(1000);
            bucket.setPosition(0.1);
            auto.sleep(2000);
            bucket.setPosition(0.75);
            auto.sleep(400);

            erect.setTargetPosition(100);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            erect.setPower(-1);

            extend.setPosition(0);
        }


        if (gamepad2.a) {
            stop.setPosition(10);
        } else {
            stop.setPosition(45);
        }

        if (gamepad1.dpad_down) {
            bucket.setPosition(0.1);
        } else {
            bucket.setPosition(0.83);
        }


        if (gamepad1.b) {
            wristL.setPosition(-0.04);
            wristR.setPosition(1.4);
            intakeL.setPower(-1);
            intakeR.setPower(1);
        } else if (gamepad1.y) {
            intakeL.setPower(1);
            intakeR.setPower(-1);
        } else {
            wristR.setPosition(0.25);
            wristL.setPosition(0.75);
            intakeR.setPower(0);
            intakeL.setPower(0);
        }


        if (gamepad1.a) {
            extend.setPosition(0);
        }
        else {
            extend.setPosition(0.23);
        }
    }
}










//   public void setBottom(TouchSensor bottom) {
//     this.bottom = bottom;


// public TouchSensor getBottom() {
// return bottom;



//}

//private Object stop(DcMotor erect)