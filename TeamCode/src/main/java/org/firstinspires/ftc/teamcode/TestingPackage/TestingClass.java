package org.firstinspires.ftc.teamcode.TestingPackage;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.TheoCode.MecanumDrive;

/*Your class that you want to be used for FTC programming has to extend
* the LinearOpMode class. It also has to have either the @Autonomous(name = "", group = "")
* or @Teleop(name = "", group = "") annotation to make the Driver Station App decide
* whether your program is going to be for the Auto or Teleop periods.*/
@Autonomous(name = "TestingClass", group = "teamcode")
public class TestingClass extends LinearOpMode {

    public static class Constants {
        static class MotorConstants {
            public static final String frontLeftMotor = "frontLeft";
            public static final String frontRightMotor = "frontRight";
            public static final String backLeftMotor = "backLeft";
            public static final String backRightMotor = "backRight";
        }

        static class ServoConstants {
            public static final double leftClawOpen = 0.5;
            public static final double leftClawClosed = 0.1;
            public static final double rightClawOpen = -0.5;
            public static final double rightClawClosed = -0.1;
        }
    }

    /*You put all the variables for your hardware components here under
    * your class name to create them.*/
    DcMotor frontLeft, frontRight, backLeft, backRight;

    Servo leftClaw, rightClaw;

    /*You can use this method in your code to make the robot
    * drive where you want it to for a certain amount of time.*/
    public void drive(String direction, double power, int millis) {
        switch(direction) {
            case "forward":
                frontLeft.setPower(power);
                frontRight.setPower(power);
                backLeft.setPower(power);
                backRight.setPower(power);

                sleep(millis);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                break;

            case "backward":
                frontLeft.setPower(-power);
                frontRight.setPower(-power);
                backLeft.setPower(-power);
                backRight.setPower(-power);

                sleep(millis);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                break;

            case "left":
                frontLeft.setPower(-power);
                frontRight.setPower(power);
                backLeft.setPower(-power);
                backRight.setPower(power);

                sleep(millis);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                break;

            case "right":
                frontLeft.setPower(power);
                frontRight.setPower(-power);
                backLeft.setPower(power);
                backRight.setPower(-power);

                sleep(millis);

                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                break;
        }
    }

    public void setPosition(double leftClawPosition, double rightClawPosition) {
        leftClaw.setPosition(leftClawPosition);
        rightClaw.setPosition(rightClawPosition);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        /*Any variable that you declare under your class name has to be initialized here.
        The way you initialize them is by using the get() method, used by your local
        HardwareMap variable, to call the physical components on your robot by providing the
        class of the component you want to reference and then the name of the component configured
        within the Driver Station App itself.*/

        frontLeft = hardwareMap.get(DcMotor.class, Constants.MotorConstants.frontLeftMotor);
        frontRight = hardwareMap.get(DcMotor.class, Constants.MotorConstants.frontRightMotor);
        backLeft = hardwareMap.get(DcMotor.class, Constants.MotorConstants.backLeftMotor);
        backRight = hardwareMap.get(DcMotor.class, Constants.MotorConstants.backRightMotor);

        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        /*This waitForStart() method has to be called in order to give your program time
        * to initialize all your hardware components and to make it so that your program doesn't
        * automatically run until you press the PLAY button.*/
        waitForStart();

        /*Making the robot drive to a specific location using the drive() method multiple times to
        * make the robot drive in different directions.*/
        drive("forward", 0.7, 1000);
        drive("left", 0.7, 1000);
        drive("backward", 0.7, 1500);

        /*Closing both claws by setting both their positions to closed at the same time, using one method.*/
        setPosition(Constants.ServoConstants.leftClawClosed, Constants.ServoConstants.rightClawClosed);
    }
}

@Disabled()
class TestingRoadRunner extends LinearOpMode {

    /*Initializing DcMotorEx variables instead of DcMotor because
    * DcMotorEx's come with their own PID controller and they can manage
    * their speed better, allowing for a smoother drive on your robot.*/
    DcMotorEx frontLeft, backLeft, frontRight, backRight;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0,0); //This will be the starting position of your robot in x and y coordinates.
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose); //This is the class that will control your robots drive and trajectory.

        /*Referencing the DcMotorEx variables to real hardware components on your robot.*/
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose); //This variable you will you use to tell your robot where to drive.

        Action strafeToBasket = tab1 //You will use this action to make the robot strafe to the basket using those x andy coordinates.
                .strafeTo(new Vector2d(41, 69))
                .build();

        Action strafeToFirstPiece = tab1.endTrajectory().fresh() //You will this action to make the robot strafe to the first piece using those x and y coordinates.
                .strafeTo(new Vector2d(50,50))
                .build();

        Actions.runBlocking( //Use this method to run all actions, one after another.
                new SequentialAction(
                      strafeToBasket,
                      strafeToFirstPiece
                )
        );
    }
}
