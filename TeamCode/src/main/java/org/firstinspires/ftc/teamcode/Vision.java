package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class Vision {

     HuskyLens huskyLens;
     ElapsedTime elapsedTime;
//     private final BraydenAutonomous autonomous = new BraydenAutonomous();

     public Vision(@NonNull HardwareMap hardwareMap, Telemetry telemetry) {
         huskyLens = hardwareMap.get(HuskyLens.class, "HuskyLens");
         elapsedTime = new ElapsedTime();
         telemetry.addData(">>", huskyLens.knock() ? "Touch start to continue." : "Problem connecting to HuskyLens.");
         telemetry.update();
     }

     public void update(Telemetry telemetry) {
         if(elapsedTime.seconds() >= 1) {
             elapsedTime.reset();
             HuskyLens.Block[] blocks = getBlocks();
             for(HuskyLens.Block block : blocks) {
                 telemetry.addData("Block", block.toString());
             }
             telemetry.update();
         }

//         autonomous.angles = autonomous.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//         double angle = autonomous.angles.firstAngle;

         telemetry.addData("X Position", getX()); //This information is only for the first block.
         telemetry.addData("Y Position", getY());
         telemetry.addData("Piece Position", getPiecePosition());
         telemetry.addData("Has Block", hasBlock());
//         telemetry.addData("IMU Orientation", angle + " degrees");
         telemetry.update();
     }

     public HuskyLens.Block[] getBlocks() {
         return huskyLens.blocks();
     }

     public int getX() {
         return hasBlock() ? getBlocks()[0].x : 0;
     }

     public int getY() {
         return hasBlock() ? getBlocks()[0].y : 0;
     }

     public boolean hasBlock() {
         return getBlocks().length > 0;
     }

     public Position getPiecePosition() {
         double segment = 240 / 9.0;

         if(getX() < segment) {
             return Position.LEFT;
         } else if(getX() >= (segment * 7)) {
             return Position.RIGHT;
         } else {
             return Position.MIDDLE;
         }
     }

     public enum Position {
         LEFT,
         MIDDLE,
         RIGHT
     }


}
