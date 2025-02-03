package org.firstinspires.ftc.teamcode.ConstantsPackage;

//import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Constants {



    public static final class ServoConstants {
        //EXTENSION CONSTANTS
        public static final double maxExtension = 1;
        public static final double minExtension = 0.575;

        //WRIST CONSTANTS
        public static final double wristDown = 0.4;
        public static final double wristHover = 0.2;
        public static final double wristTransfer = 0;

        //GEAR SERVO CONSTANTS
        public static final double gearDown = 0.4;
        public static final double gearDownDown = 0.6;
        public static final double gearTransfer = 0;

        //INTAKE CLAW CONSTANTS
        public static final double clawClosed = 0.525;
        public static final double clawOpen = 0.2;

        //SPIN SERVO CONSTANTS
        public static final double spinCenter = 0.5;
    }


    public static final class OpModes {
        public static final String linearOp = "Linear Opmode";
        public static final String teleop = "Teleop";

    }
    //
    public static final class MecanumConstants {

        public static final String  frontLeftMotor = "left_front";
        public static final String frontRightMotor = "right_front";
        public static final String backLeftMotor = "left_back";
        public static final String backRightMotor = "right_back";

        // public static final PIDCoefficients yPID = new PIDCoefficients(0.8, 0, 0);
        //  public static final PIDCoefficients xPID = new PIDCoefficients(0.8, 0, 0);
         //public static final PIDCoefficients yawPID = new PIDCoefficients(0.4, 0, 0);

        public static final DcMotor.Direction invertLeft = DcMotor.Direction.FORWARD;
        public static final DcMotor.Direction invertRight = DcMotor.Direction.REVERSE;
        public static final DcMotor.ZeroPowerBehavior neutralMode = DcMotor.ZeroPowerBehavior.BRAKE;

        public static final double ticksPerRev = 8192;
        public static final double wheelD = 38 / 25.4; //38mm in inches
        public static final double gearRatio = 1;
        public static final double ticksToInch = 1;//(wheelD * Math.PI) / ticksPerRev;


        public static double LATERAL_DISTANCE = 0; // in - distance between left and right
        public static double FORWARD_OFFSET = 0; // in - distance between the forward center
    }
}