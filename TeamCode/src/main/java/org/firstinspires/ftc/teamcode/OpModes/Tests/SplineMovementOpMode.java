package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class SplineMovementOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        final double x = 0, y =0;
        final double heading = 0, finalX = 0, finalY = 0;
        Pose2d initialPos = new Pose2d(x,y);
        SampleMecanumDrive sampleMecanumDrive = new SampleMecanumDrive(hardwareMap);

        waitForStart();

        if(isStopRequested()) return;

        //todo: need to find out where the prop is and change the final position accordingly and change the heading to prepare for the unload of the pixel
        //todo: need to put the purple pixel in place (where the prop is)
        Trajectory trajectory = sampleMecanumDrive.trajectoryBuilder(initialPos)
                .splineTo(new Vector2d(finalX, finalY), heading)
                .build();

        sampleMecanumDrive.followTrajectory(trajectory);

        //todo: put the orange pixel in place
    }
}
