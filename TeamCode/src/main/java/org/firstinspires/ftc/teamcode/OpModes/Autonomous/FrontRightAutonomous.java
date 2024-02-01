package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;

@TeleOp(group = "Autonomous", name = "FrontRightAutonomous")
public class FrontRightAutonomous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CameraSystem cameraSystem = new CameraSystem(this, true);
        CameraSystem.DetectionLocation detectionLocation = cameraSystem.DetectAndFindPropLocation();

        SimplerOpMode.runAfterInput(SimplerOpMode.StartPosition.FRONTRIGHT ,detectionLocation);
    }
}
