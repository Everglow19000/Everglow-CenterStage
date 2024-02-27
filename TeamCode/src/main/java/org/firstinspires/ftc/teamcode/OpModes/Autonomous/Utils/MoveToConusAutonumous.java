package org.firstinspires.ftc.teamcode.OpModes.Autonomous.Utils;


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
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class MoveToConusAutonumous{
    public static final double SQUARE_SIZE = 60.5;

    public enum StartPosition{
        FRONTLEFT(new Vector2d(0, 0)), FRONTRIGHT(new Vector2d(0, 0))
        , BACKLEFT(new Vector2d(0, SQUARE_SIZE*2)), BACKRIGHT(new Vector2d(0, SQUARE_SIZE*2));

        public final Vector2d startPositionVector;
        StartPosition(Vector2d startPosition) {
            this.startPositionVector = startPosition;
        }

        public boolean isLeft() {
            if(this == FRONTLEFT || this == BACKLEFT)
                return true;
            else
                return false;
        }

        public boolean isFront() {
            if(this == FRONTLEFT || this == FRONTRIGHT)
                return true;
            else
                return false;
        }
    }

    public static CameraSystem.DetectionLocation AutonumousGeneral(LinearOpMode opMode
            , StartPosition startPosition, SampleMecanumDrive drive){
        double angleRatio = 1;
        GWheelSystem gWheelSystem = new GWheelSystem(opMode);
        CameraSystem cameraSystem = new CameraSystem(opMode, !startPosition.isLeft());
        FourBarSystem fourBarSystem = new FourBarSystem(opMode);
        ElevatorSystem elevatorSystem = new ElevatorSystem(opMode);
        ClawSystem clawSystem = new ClawSystem(opMode);
        clawSystem.ChangePos(true);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Sequence closeClaw = new Sequence(false, clawSystem.getExecutor(false));
        closeClaw.startSequence();
        boolean oneRun = false;

        //Sequence drop = sequenceControl.DropAndRetreatSeq();
        double xPoint = 85, yPoint = -10;
        if((!startPosition.isLeft() && startPosition.isFront()) ||
                (startPosition.isLeft() && !startPosition.isFront()))
            yPoint += 5;

        Trajectory splneToMiddle = drive.trajectoryBuilder(new Pose2d(0,0, 0))
                .splineTo(new Vector2d(xPoint,yPoint), 0)
                .build();

        double moveInYAxis = 0;
        double moveInXAxis = 0;
        double angle = Math.toRadians(-90);

        opMode.telemetry.addLine("build trajectories...");
        opMode.telemetry.update();

        Trajectory leftTrag =drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(drive.getPoseEstimate().plus(new Pose2d(25, SQUARE_SIZE+10)).vec(), angle)
                .build();
        Trajectory MiddleTrag = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE,0)).vec(),2.1*angle)
                .build();
        Trajectory rightTrag = drive.trajectoryBuilder(drive.getPoseEstimate())
                .splineTo(drive.getPoseEstimate().plus(new Pose2d(25, -(SQUARE_SIZE+10))).vec(), -(Math.toRadians(5)+angle))
                .build();
        Sequence getUp = new Sequence(false, clawSystem.getExecutor(false)
                , elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.LOW));

        opMode.telemetry.addLine("ready");
        opMode.telemetry.update();
        opMode.waitForStart();
        fourBarSystem.setMotorPower(0.5);
        CameraSystem.DetectionLocation location  = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Target", location);
        opMode.telemetry.addData("startPos", startPosition.name());
        opMode.telemetry.update();

        if (opMode.isStopRequested()) return location;

        //first deployment
        drive.followTrajectory(splneToMiddle);

        // locate and put the first pixel //
        switch (location){
            case RIGHT:
                if (!startPosition.isLeft() && startPosition.isFront()) {
                    //if you star in the front and right
                    moveInYAxis += SQUARE_SIZE+10;
                    moveInYAxis = -moveInYAxis;
                    moveInXAxis += 30;
                    drive.followTrajectory(rightTrag);
                    break;
                }
                drive.turn(angle);
                break;

            case MIDDLE:
                if (!startPosition.isFront()) {
                    //if in the back
                    moveInXAxis += SQUARE_SIZE+10;
                    drive.followTrajectory(MiddleTrag);
                }
                break;

            case LEFT:{
                if (startPosition.isLeft() && startPosition.isFront()) {
                    moveInYAxis += SQUARE_SIZE+10;
                    moveInXAxis += 30;
                    drive.followTrajectory(leftTrag);
                    break;
                }
                angle = -1*angle;
                drive.turn(angle);
                break;
            }
        }
        opMode.telemetry.addData("estimate pos", drive.getPoseEstimate());
        opMode.telemetry.update();

        gWheelSystem.setPower(0.4);
        if (opMode.isStopRequested()) return location;
        long time = System.currentTimeMillis();
        while(!opMode.isStopRequested()){
            if(System.currentTimeMillis()-time > 1000)
                break;
        }
        gWheelSystem.setPower(0);

        switch (startPosition){
            case FRONTLEFT:
            {

                if(opMode.isStopRequested()) return location;
                Trajectory secDrop;

                //double moveNearBoard = 0;
                while (opMode.opModeIsActive()) {
                    if(!oneRun) {
                        switch (location){
                            case RIGHT:
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .splineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE, SQUARE_SIZE*2)).vec(),0)
                                        .build();
                              break;
                            case MIDDLE:
                                Trajectory backAndTurn = drive.trajectoryBuilder(new Pose2d())
                                        .lineToConstantHeading(new Vector2d(-SQUARE_SIZE,SQUARE_SIZE*2-5)).build();
                                //drive.turn(Math.toRadians(-90));
                                drive.followTrajectory(backAndTurn);
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE*2+15,SQUARE_SIZE*2-10,angle))).build();
                                break;
                            default:
                            case LEFT:
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .splineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(-SQUARE_SIZE, 40)).vec(),0)
                                        .build();

                        }

                        if (opMode.isStopRequested()) return location;

                        //getUp = new Sequence(false, clawSystem.getExecutor(true))
                        getUp.startSequence();
                        //to run parallel

                        while (!getUp.isDone()){
                            fourBarSystem.updateP(0.35);
                        }
                        drive.followTrajectory(secDrop);

                        Sequence openClaw = new Sequence(false, clawSystem.getExecutor(true));
                        openClaw.startSequence();
                        opMode.sleep(800);
                        Sequence drop = sequenceControl.DropAndRetreatSeq();
                        drop.startSequence();
                        while (!drop.isDone()){
                            fourBarSystem.updateP(0.35);
                        }

                        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE+moveInXAxis,
                                                30+moveInYAxis))
                                        ,Math.toRadians(-90))
                                .build();
                        //drive.followTrajectory(park);
                        oneRun = true;
                    }
                }
            }
            case FRONTRIGHT:
            {

                if(opMode.isStopRequested()) return location;
                Trajectory secDrop;

                //double moveNearBoard = 0;
                while (opMode.opModeIsActive()) {
                    if (!oneRun) {
                        switch (location) {
                            case RIGHT:
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .splineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(-SQUARE_SIZE, -40)).vec()
                                                , 0).build();
                                break;

                            case MIDDLE:
                                Trajectory backAndTurn = drive.trajectoryBuilder(new Pose2d())
                                        .lineToConstantHeading(new Vector2d(-SQUARE_SIZE, -SQUARE_SIZE * 2)).build();
                                //drive.turn(Math.toRadians(-90));
                                drive.followTrajectory(backAndTurn);
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(drive.getPoseEstimate().plus(new Pose2d((SQUARE_SIZE * 2 + 15), -(SQUARE_SIZE * 2 - 10)
                                                , -angle))).build();
                                break;
                            default:
                            case LEFT:
                                secDrop = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .splineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(SQUARE_SIZE, -SQUARE_SIZE*2)).vec(),0)
                                        .build();
                                break;
                        }

                        if (opMode.isStopRequested()) return location;


                        //getUp = new Sequence(false, clawSystem.getExecutor(true))

                        getUp.startSequence();
                        //to run parallel

                        while (!getUp.isDone()) {
                            fourBarSystem.updateP(0.35);
                        }
                        drive.followTrajectory(secDrop);

                        Sequence openClaw = new Sequence(false, clawSystem.getExecutor(true));
                        openClaw.startSequence();
                        opMode.sleep(800);
                        Sequence drop = sequenceControl.DropAndRetreatSeq();
                        drop.startSequence();
                        while (!drop.isDone()) {
                            fourBarSystem.updateP(0.35);
                        }

                        Trajectory park = drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(drive.getPoseEstimate().plus(new Pose2d((SQUARE_SIZE + moveInXAxis),
                                                        -(30 + moveInYAxis)))
                                        , Math.toRadians(90))
                                .build();
                        //drive.followTrajectory(park);
                        oneRun = true;
                    }
                }
            }
        }

        //start the second deployment//
        return location;
    }
}