package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {

    public static final class OpModes {
        public static final String linearOp = "Linear Opmode";
        public static final String teleop = "Teleop";

    }
    //
    public static final class MecanumConstants {

        public static final String frontLeftMotor = "FrontLeft";
        public static final String frontRightMotor = "FrontRight";
        public static final String backLeftMotor = "BackLeft";
        public static final String backRightMotor = "BackRight";

        // public static final PIDCoefficients yPID = new PIDCoefficients(0.8, 0, 0);
        //  public static final PIDCoefficients xPID = new PIDCoefficients(0.8, 0, 0);
        // public static final PIDCoefficients yawPID = new PIDCoefficients(0.4, 0, 0);

        public static final DcMotor.Direction invertLeft = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction invertRight = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior neutralMode = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final double ticksPerRev = 8192;
        public static final double wheelD = 38 / 25.4; //38mm in inches
        public static final double gearRatio = 1;
        public static final double ticksToInch = 1;//(wheelD * Math.PI) / ticksPerRev;


        public static double LATERAL_DISTANCE = 0; // in - distance between left and right
        public static double FORWARD_OFFSET = 0; // in - distance between the forward center

        public static final int ELEVATOR_HIGH_BASKET = 2500;
        public static final int TURN_TO_BASKET = 27;
        public static final int ELEVATOR_LOW_BASKET = 10;

    }
}