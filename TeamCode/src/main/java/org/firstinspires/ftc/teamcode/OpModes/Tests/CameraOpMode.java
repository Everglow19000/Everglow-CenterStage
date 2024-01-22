package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

@TeleOp(name = "Camera")
public class CameraOpMode extends LinearOpMode {

    //make more attributes & functions
    VisionPortal camera;
    AprilTagProcessor aprilTag;

    @Override
    public void runOpMode(){
        CameraSystem cs = new CameraSystem(this);
        waitForStart();
        /*
        Dictionary<AprilTagDetection, CameraSystem.AprilTagLocation> DTL;
        AprilTagDetection Key;
        Enumeration<AprilTagDetection> AE;
        List<AprilTagDetection> detections;
        while (opModeIsActive()){
            try {
                detections = cs.DetectAprilTags();
                DTL = cs.GetAprilTagLocation(detections);
                AE = DTL.keys();

                if(AE.hasMoreElements())
                    Key = AE.nextElement();
                else
                    continue;

                telemetry.addData("amount places:", DTL.size());
                telemetry.addData("amount total detection:", detections.size());
                while (AE.hasMoreElements()) {
                    telemetry.addData(DTL.get(Key).name() + ":", Key.id);
                    Key = AE.nextElement();
                }
                telemetry.update();
                sleep(200);
            }
            catch (Exception e){
                telemetry.addData("Exception:", e);
                telemetry.update();
                cs.CloseCamera();
                break;
            }

        }
     */
        if(gamepad1.cross) {
            List<Recognition> recognitions = cs.DetectProp();

            for (Recognition rec :
                    recognitions) {
                telemetry.addData("rec -> x: ", CameraSystem.ConvertInchToCm(cs.ConvertRecognitionToPos(rec, true)));
                telemetry.addData("y: ", CameraSystem.ConvertInchToCm(cs.ConvertRecognitionToPos(rec, false)));
            }
            telemetry.update();
        }
    }
}