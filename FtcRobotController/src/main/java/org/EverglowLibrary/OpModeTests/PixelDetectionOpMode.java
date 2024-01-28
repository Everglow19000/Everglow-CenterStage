package org.EverglowLibrary.OpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

//more imports

@TeleOp(name = "Pixel")
public class PixelDetectionOpMode extends LinearOpMode {

    //make more attributes & functions

    @Override
    public void runOpMode(){

        CameraSystem cameraSystem = new CameraSystem(this);
        List<Recognition> recognitions;
        waitForStart();

        while (opModeIsActive()) {
            recognitions = cameraSystem.DetectProp();
            if(gamepad1.cross) {
                for (Recognition rec : recognitions) {
                    telemetry.addData("x:", cameraSystem.ConvertRecognitionToPos(rec, true));
                    telemetry.addData("y:", cameraSystem.ConvertRecognitionToPos(rec, false));
                }
                telemetry.update();
            }
        }
    }
}
