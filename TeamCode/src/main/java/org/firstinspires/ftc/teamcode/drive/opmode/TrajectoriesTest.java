package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.drive.TankDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.Path;
import com.acmerobotics.roadrunner.path.PathSegment;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name = "Diffrent Trajectories tests")
public class TrajectoriesTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d())
                .splineTo(new Vector2d(100,0), 0)
                .splineTo(new Vector2d(100,100), Math.toRadians(90))
                .build();

        Trajectory trajectory = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(100,0), 0)
                .splineTo(new Vector2d(100,100), Math.toRadians(90))
                .build();

        Trajectory justForword =  drive.trajectoryBuilder(new Pose2d())
                .forward(100)
                .build();
        //check if the robot return to the original place

        waitForStart();

        telemetry.addLine("x for straight path");
        telemetry.addLine("o for spline path");
        telemetry.addLine("sqoure for spline path on Trajectory sequance");

        telemetry.update();
        while (opModeIsActive()) {
            if(gamepad1.square)
                drive.followTrajectorySequence(trajectorySequence);
            if(gamepad1.circle)
                drive.followTrajectory(trajectory);

            if(gamepad1.cross)
                drive.followTrajectory(justForword);

        }
    }
}
