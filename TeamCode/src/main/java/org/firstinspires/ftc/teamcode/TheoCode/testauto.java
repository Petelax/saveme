package org.firstinspires.ftc.teamcode.TheoCode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

@Autonomous(name = "testauto", group = "teamcode")
public class testauto extends LinearOpMode {

    org.firstinspires.ftc.teamcode.TheoCode.Drive Drive;
    Servo leftEX, rightEX;
    DcMotor erect;



    public class SCORE implements Action {
//        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            erect.setTargetPosition(-3200);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (erect.getCurrentPosition() < -3100 && erect.getCurrentPosition() > -3300) {
                erect.setPower(0);
            } else {
                erect.setPower(-1);

            }
            return (erect.getCurrentPosition() < -3100 && erect.getCurrentPosition() > -3300);

        }
    }

    public Action SCOREING() {
        return new SCORE();
    }


    public class Down implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            erect.setTargetPosition(-50);
            erect.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (erect.getCurrentPosition() < -60 && erect.getCurrentPosition() > -40) {
                erect.setPower(0);
            } else {
                erect.setPower(-1);

            }
            return (erect.getCurrentPosition() < -60 && erect.getCurrentPosition() > -40);

        }
    }

    public Action DOWN() {
        telemetry.addData("Going", "Down");
        telemetry.update();
        return new Down();
    }

    public class intake implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            leftEX.setPosition(0);
            rightEX.setPosition(0.5);

        return false;
        }
    }

    public Action INTAKE() {
        return new intake();
    }
    public class transfer implements Action {


        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            leftEX.setPosition(0.5);
            rightEX.setPosition(0);
            return false;
        }
    }

    public Action TRANSFER() {
        return new transfer();
    }

    @Override

    public void runOpMode() throws InterruptedException {
        leftEX = hardwareMap.get(Servo.class, "leftEx");
        rightEX = hardwareMap.get(Servo.class, "rightEX");
        erect = hardwareMap.get(DcMotor.class, "erect");


        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {


            Pose2d initialPose = new Pose2d(41.5, 63,Math.PI);
            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, initialPose);


            waitForStart();

            {
                Actions.runBlocking(new SequentialAction(

                        //extend intake and raise elevator
                        SCOREING(),
                        INTAKE(),

                                drive.actionBuilder(new Pose2d(52, 50, (5*Math.PI)/4))

                                        //drive to basket
                                        .strafeTo(new Vector2d(52, 50))
                                        .waitSeconds(5)
                                        .build(),
                                        DOWN(),



                                //drive to first piece
                                drive.actionBuilder(new Pose2d(50, 48, (5*Math.PI)/4))

                                        .strafeTo(new Vector2d(50, 48))
                                        .turnTo(-Math.PI/2)
                                        .waitSeconds(5)
                                        .build(),
                                        SCOREING(),

                                         //brings intake back in and transfers
                                         TRANSFER(),
                                drive.actionBuilder(new Pose2d(50, 48, (5*Math.PI)/4))

                                //drives to bucket with 2nd piece
                                         .strafeTo(new Vector2d(50, 50))

                                .waitSeconds(2)
                                .build(),
                                DOWN(),



                        //drive to seccond piece
                        drive.actionBuilder(new Pose2d(60, 60, (5*Math.PI)/4))

                                .strafeTo(new Vector2d(60, 48))
                                .turnTo(-Math.PI/2)
                                .waitSeconds(2)
                                .build(),
                                SCOREING(),

                            //brings intake back in and transfers
                               // TRANSFER(),
                        drive.actionBuilder(new Pose2d(30, 48, (5*Math.PI)/4))

                                //drives to bucket with 3rd piece
                                .strafeTo(new Vector2d(50, 50))
                                .waitSeconds(2)
                                .build(),
                                DOWN(),



                        //drive to fourth piece
                        drive.actionBuilder(new Pose2d(60, 60,(5*Math.PI)/4))

                                .strafeTo(new Vector2d(65, 48))
                                .turnTo(-Math.PI/2)
                                .waitSeconds(2)
                                .build(),
                                SCOREING(),
                                TRANSFER(),

                        drive.actionBuilder(new Pose2d(50, 50, (5*Math.PI)/4))

                                //drives to bucket with 3rd piece
                                .strafeTo(new Vector2d(50, 50))
                                .waitSeconds(2)
                                .build(),
                        DOWN(),

                        drive.actionBuilder(new Pose2d(30, 0, (Math.PI)))


                                .strafeTo(new Vector2d(30, 0))

                                .waitSeconds(2)
                                .build(),
                        DOWN()


                        )

                );


            }


        } else {
            throw new RuntimeException();
        }
    }
}
