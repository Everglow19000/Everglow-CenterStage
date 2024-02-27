

package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.utils.Pose;
import org.firstinspires.ftc.teamcode.OpModes.DriverControl.SequenceControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ExecutorUtils.ExecutorTrajectories;

@Autonomous(name = "MTCAFtontRight")
public class MTCAFtontRight extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FourtyFivePoints autonumous = new FourtyFivePoints();
        autonumous.firstCall(this, FourtyFivePoints.StartPosition.FRONT_RIGHT);


    }

}
