package org.firstinspires.ftc.teamcode.Systems;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.SequenceSegment;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

import java.util.ArrayList;
import java.util.List;

public class DrivingSystem {
    private SampleMecanumDrive drive;
    private Pose2d lastedUpdatedPose2d;
    private OpMode opMode;

    public DrivingSystem(OpMode opMode, HardwareMap hardwareMap){
        this.opMode = opMode;
        drive = new SampleMecanumDrive(hardwareMap);
        updateLastPose();
    }

    public void updateLastPose(){
        lastedUpdatedPose2d = drive.getPoseEstimate();
    }

    public void driveSpline(Vector2d vector, double endAngle){
        Trajectory toDrive = drive.trajectoryBuilder(lastedUpdatedPose2d)
                .splineTo(vector,endAngle)
                .build();

        drive.followTrajectory(toDrive);
    }
}
