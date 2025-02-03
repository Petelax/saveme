package org.firstinspires.ftc.teamcode.TestingPackage;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "AutomaticPieceDetection", group = "teamcode")
public class AutomaticPieceDetection extends LinearOpMode {

    private HuskyLens huskyLens;

    public void measureWidth() {
        while(opModeIsActive()) {
            if(huskyLens.blocks().length > 0) {
                telemetry.addData("Block Width", huskyLens.blocks()[0].width);
                telemetry.update();
            }
        }
    }

    public void rotateToPiece() {
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION);
        while(opModeIsActive()) {
            if(huskyLens.blocks().length > 0) {
                if(huskyLens.blocks()[0].id == 1) {

                    telemetry.addData("Piece", "is vertical.");
                } else if(huskyLens.blocks()[0].id == 2) {
                    telemetry.addData("Piece", "is horizontal.");
                } else if(huskyLens.blocks()[0].id == 4) {
                    telemetry.addData("Piece", "is diagonal either way.");
                }
                telemetry.update();
            }
        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);

        waitForStart();

        while(opModeIsActive()) {
            if(huskyLens.blocks().length > 0) {
                rotateToPiece();
            }
        }
    }
}
