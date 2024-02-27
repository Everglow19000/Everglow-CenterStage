package org.firstinspires.ftc.teamcode.OpModes.Autonomous;



import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;
import org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils.MoveToConusAutonumous;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "MTCAFrontLeft", group = "Main")
public class MTCAFrontLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        CameraSystem.DetectionLocation location =
                MoveToConusAutonumous.AutonumousGeneral(this, MoveToConusAutonumous.StartPosition.FRONTLEFT
                ,drive);
        /*
        if(isStopRequested()) return;
        Trajectory secDrop;

        //double moveNearBoard = 0;
        while (opModeIsActive()) {
            if(!oneRun) {
                switch (location){
                    case RIGHT:
                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineTo(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE*2, -40)).vec()
                                        ,drive.getPoseEstimate().headingVec().angle()).build();
                        /*secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE*2).build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeLeft(20).build());


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
                        /*secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .back(SQUARE_SIZE).build();
                        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .strafeRight(40).build());


                        secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(-SQUARE_SIZE, 40)).vec()
                                        ,0).build();

                }

                if (isStopRequested()) return;



                //getUp = new Sequence(false, clawSystem.getExecutor(true))
                Sequence getUp = new Sequence(false, clawSystem.getExecutor(false)
                        ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                        , fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.LOW));
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

                Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE, 30))
                                ,Math.toRadians(-90))
                        .build();
                drive.followTrajectory(park);
                oneRun = true;
            }
        }
        */
    }
}
