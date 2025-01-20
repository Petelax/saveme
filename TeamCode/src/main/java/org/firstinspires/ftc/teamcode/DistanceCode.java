package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
/**This is for Owen's robot. You don't need to worry about the code under this class.*/
@Disabled()
public class DistanceCode extends LinearOpMode {
    DistanceSensor sensor;
    HuskyLens huskyLens;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    HuskyLens.Block block;
    @Override
    public void runOpMode() {
        frontRight = hardwareMap.get(DcMotor.class, "driveFR");
        frontLeft = hardwareMap.get(DcMotor.class, "driveFL");
        backLeft = hardwareMap.get(DcMotor.class, "driveBL");
        backRight = hardwareMap.get(DcMotor.class, "driveBR");
        sensor = hardwareMap.get(DistanceSensor.class, "frontDistsncer");
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");

        waitForStart();

        while(opModeIsActive()) {
            frontRight.setPower(0.6);
            backRight.setPower(0.6);
            frontLeft.setPower(0.6);
            backLeft.setPower(0.6);

            if (sensor.getDistance(DistanceUnit.CM ) < 30) {
                frontRight.setPower(-0.5);
                backRight.setPower(-0.5);
                frontLeft.setPower(0.5);
                backLeft.setPower(0.5);

                sleep(600);
            }

            if(huskyLens.blocks().length > 0) {
                block = huskyLens.blocks()[0];
                telemetry.addData("Block ID", block.toString());
                telemetry.update();
            }
        }

    }
}
