/*
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import static org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils.MoveToConusAutonumous.SQUARE_SIZE;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils.MoveToConusAutonumous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "avivTestingSomeShit")
public class avivTestingSomeShit extends LinearOpMode {
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
                                .back(SQUARE_SIZE*2)
                                .build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeRight(20).build());
                        break;
                    case MIDDLE:
                        Trajectory backAndTurn = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineTo(new Vector2d(-10,0), -PI/1.5).build();
                        drive.turn(Math.toRadians(-90));
                        drive.followTrajectory(backAndTurn);
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToConstantHeading(new Vector2d(SQUARE_SIZE*2,drive.getPoseEstimate().getY()))
                                .build();
                        break;
                    default:
                    case LEFT:
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE)
                                .build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeLeft(15).build());

                }

                if (isStopRequested()) return;


                Sequence getUp = sequenceControl.GetReadyToDropSeq();
                //getUp = new Sequence(false, clawSystem.getExecutor(true))
                getUp.startSequence();
                drive.followTrajectory(secDrop);//to run parallel
                while (!getUp.isDone()){
                    fourBarSystem.updateP(0.35);
                }

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
 */