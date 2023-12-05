package org.firstinspires.ftc.teamcode.OpModes.Tests;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Systems.CameraSystem;
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

        Dictionary<AprilTagDetection, CameraSystem.AprilTagLocation> DTL;
        AprilTagDetection Key;
        Enumeration<AprilTagDetection> AE;
        while (opModeIsActive()){
            DTL = cs.GetAprilTagLocation();
            AE = DTL.keys();
            Key = AE.nextElement();
            while (AE.hasMoreElements()) {
                telemetry.addData(DTL.get(Key).name() + ":", Key.id);
                Key = AE.nextElement();
            }
            telemetry.update();
            sleep(50);
        }

        camera.close();
    }
}