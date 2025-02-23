package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.TheoCode.Drive;

@Config
public class P2P extends CommandBase {
    private final Drivetrain drive;
    private final Pose2d setpoint;
    public static double translationKP = 0.0;
    public static double translationKI = 0.0;
    public static double translationKD = 0.0;

    public static double rotationKP = 0.0;
    public static double rotationKI = 0.0;
    public static double rotationKD = 0.0;

    public static double translationPositionTolerance = 1.0; //inch
    public static double translationVelocityTolerance = 100.0; //inch

    public static double rotationPositionTolerance = 0.05; //rads
    public static double rotationVelocityTolerance = 10.0; //rads

    private PIDFController xController = new PIDFController(translationKP, translationKI, translationKD, 0.0);
    private PIDFController yController = new PIDFController(translationKP, translationKI, translationKD, 0.0);
    private PIDController headingController = new PIDController(rotationKP, rotationKI, rotationKD);


    public P2P(Drivetrain drive, Pose2d setpoint) {
        this.drive = drive;
        this.setpoint = setpoint;

        headingController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(translationPositionTolerance, translationVelocityTolerance);
        yController.setTolerance(translationPositionTolerance, translationVelocityTolerance);
        headingController.setTolerance(rotationPositionTolerance, rotationVelocityTolerance);
    }

    @Override
    public void initialize() {
        Pose2d pose = drive.getPose();
        xController.calculate(pose.getX(), setpoint.getX());
        yController.calculate(pose.getY(), setpoint.getY());
        headingController.calculate(pose.getRotation().getRadians(), setpoint.getRotation().getRadians());

    }

    @Override
    public void execute() {
        Pose2d pose = drive.getPose();
        double xSpeed = xController.calculate(pose.getX(), setpoint.getX());
        double ySpeed = yController.calculate(pose.getY(), setpoint.getY());
        double headingSpeed = headingController.calculate(pose.getRotation().getRadians(), setpoint.getRotation().getRadians());

        drive.drive(ySpeed, xSpeed, headingSpeed, true);

    }

    @Override
    public boolean isFinished() {
        return xController.atSetPoint() && yController.atSetPoint() && headingController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0.0, 0.0, 0.0, true);
    }

}
