package org.EverglowLibrary.OpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

//more imports

@TeleOp(name = "PropDetectionComplex")
public class PixelDetectionOpMode extends LinearOpMode {

    //make more attributes & functions

    @Override
    public void runOpMode(){

        CameraSystem cameraSystem = new CameraSystem(this, true);
        List<Recognition> recognitions;

        //CameraSystem.DetectionLocation blue =  cameraSystem.DetectAndFindPropLocation(false);
        CameraSystem.DetectionLocation red = cameraSystem.DetectAndFindPropLocation();
        //telemetry.addData("location blue:",  blue);
        telemetry.addData("location red:", red);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            recognitions = cameraSystem.DetectProp();
            recognitions.addAll(cameraSystem.DetectProp());
            for (Recognition rec : recognitions) {
                telemetry.addData("x:", cameraSystem.ConvertRecognitionToPos(rec, true));
                telemetry.addData("y:", cameraSystem.ConvertRecognitionToPos(rec, false));
                telemetry.addData("location:", cameraSystem.RecognitionToLocation(rec));
                telemetry.addLine();
            }
            telemetry.update();
        }
    }
}
