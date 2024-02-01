package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;

@TeleOp(group = "autonomous", name = "BackLeftAutonomous")
public class BackLeftAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        CameraSystem cameraSystem = new CameraSystem(this, false);
        CameraSystem.DetectionLocation detectionLocation = cameraSystem.DetectAndFindPropLocation();

        SimplerOpMode.runAfterInput(SimplerOpMode.StartPosition.BACKLEFT ,detectionLocation);
    }
}
