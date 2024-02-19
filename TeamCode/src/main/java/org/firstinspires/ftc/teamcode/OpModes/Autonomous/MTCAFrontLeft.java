package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.MoveToConusAutonumous.SQUARE_SIZE;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceInSequence;
import org.EverglowLibrary.utils.Pose;
import org.firstinspires.ftc.teamcode.OpModes.DriverControl.SequenceControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ExecutorUtils.ExecutorTrajectories;

@Autonomous(name = "MTCAFrontLeft")
public class MTCAFrontLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        boolean oneRun = false;
        FourBarSystem fourBarSystem = new FourBarSystem(this);
        ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        ClawSystem clawSystem = new ClawSystem(this);
        clawSystem.ChangePos(true);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        CameraSystem.DetectionLocation location =
                MoveToConusAutonumous.AutonumousGeneral(this, MoveToConusAutonumous.StartPosition.FRONTLEFT
                ,drive);

        if(isStopRequested()) return;
        Trajectory secDrop;

        //double moveNearBoard = 0;
        while (opModeIsActive()) {
            if(!oneRun) {
                switch (location){
                    case RIGHT:
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE*2).build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeRight(20).build());
                        break;
                    case MIDDLE:
                        Trajectory backAndTurn = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .splineTo(new Vector2d(-10,0), -PI/1.5).build();
                        drive.turn(Math.toRadians(-90));
                        drive.followTrajectory(backAndTurn);
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE*2).build();
                        break;
                    default:
                    case LEFT:
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE).build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeLeft(15).build());

                }

                if (isStopRequested()) return;



                //getUp = new Sequence(false, clawSystem.getExecutor(true))
                Sequence getUp = sequenceControl.GetReadyToDropSeq();
                getUp.startSequence();
                //to run parallel

                while (!getUp.isDone()){
                    fourBarSystem.updateP(0.35);
                }
                drive.followTrajectory(secDrop);

                Sequence openClaw = new Sequence(false, clawSystem.getExecutor(true));
                openClaw.startSequence();
                sleep(800);
                Sequence drop = sequenceControl.DropAndRetreatSeq();
                drop.startSequence();
                while (!drop.isDone()){
                    fourBarSystem.updateP(0.35);
                }
                oneRun = true;
            }
        }
    }
}
