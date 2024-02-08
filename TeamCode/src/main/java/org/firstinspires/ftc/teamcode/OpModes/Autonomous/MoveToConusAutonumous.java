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

@Autonomous(name = "Basic Autunumous Red")
public class MoveToConusAutonumous extends LinearOpMode {
    public static final double SQUARE_SIZE = 60.5;

    static final double North = 0;
    static final double East = -PI/2;
    static final double West = PI/2;
    static final double South = PI;

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
    @Override
    public void runOpMode() throws InterruptedException {
       //AutonumousGeneral(this, StartPosition.FRONTLEFT);
    }

    public static void AutonumousGeneral(LinearOpMode opMode, StartPosition startPosition){
        GWheelSystem gWheelSystem = new GWheelSystem(opMode);
        CameraSystem cameraSystem = new CameraSystem(opMode, startPosition.isLeft());
        FourBarSystem fourBarSystem = new FourBarSystem(opMode);
        ElevatorSystem elevatorSystem = new ElevatorSystem(opMode);
        ClawSystem clawSystem = new ClawSystem(opMode);
        SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Sequence getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        Sequence drop = sequenceControl.DropAndRetreatSeq();

        Pose2d startLocation = new Pose2d();
        if(startPosition.isLeft()) startLocation = new Pose2d(0, 0, East);
        else startLocation = new Pose2d(0, 0, West);

        opMode.waitForStart();


        CameraSystem.DetectionLocation location  = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Target", location);
        opMode.telemetry.addData("startPos", startPosition.name());


        // drop the p
        double xPoint = 82, yPoint = -15, angle = East;
        switch (location) {
            case RIGHT:
                angle = South;
            case LEFT:
                angle = North ;
        }

        if(!startPosition.isLeft()) {
            if(location == CameraSystem.DetectionLocation.MIDDLE) angle = West;
            xPoint = -xPoint;
        }

        opMode.telemetry.addData("xPoint", xPoint);
        opMode.telemetry.addData("yPoint", yPoint);
        opMode.telemetry.addData("angle", angle);
        opMode.telemetry.update();


        Trajectory splneToMiddle = drive.trajectoryBuilder(startLocation)
                .splineTo(new Vector2d(xPoint,yPoint), angle)
                .build();
        boolean ifrun = true;
        double moveInYAxis = 0;
        double moveInXAxis = 0;

        //Trajectory splineTofinalPos;






        if (opMode.isStopRequested()) return;


        //first deployment
        while(opMode.opModeIsActive()){
            if(ifrun) {
                drive.followTrajectory(splneToMiddle);
            }

                /*switch (location){
                    case RIGHT: {
                        if (!startPosition.isLeft() && startPosition.isFront()) {
                            moveInYAxis += SQUARE_SIZE*2;
                            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeRight(moveInYAxis).build());
                            drive.turn(angle);

                            //secondAngle = 2*(Math.PI/1.4);
                            //Trajectory splineTofinalPos = drive.trajectoryBuilder(drive.getPoseEstimate())
                            //        .splineTo(drive.getPoseEstimate().vec().plus(new Vector2d(20, SQUARE_SIZE))
                            //        ,0)
                            //        .build();
                            //ExecutorTrajectories executorTrajectories = new ExecutorTrajectories();
                            break;
                        }
                        angle = -1*angle;
                        drive.turn(angle);
                        break;
                    }
                    case MIDDLE: {
                        if (!startPosition.isFront()) {
                            moveInXAxis += SQUARE_SIZE;
                            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .forward(moveInXAxis).build());
                            drive.turn(2*angle);

                        }
                        break;
                    }
                    case LEFT:{
                        if (startPosition.isLeft() && startPosition.isFront()) {
                            moveInYAxis += SQUARE_SIZE*2;
                            drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .strafeLeft(moveInYAxis).build());
                            drive.turn(angle);
                            break;
                        }
                        angle = -1*angle;
                        drive.turn(angle);
                        break;
                    }
                }

                gWheelSystem.setPower(0.4);
                if (opMode.isStopRequested()) return;
                long time = System.currentTimeMillis();
                while(!opMode.isStopRequested()){
                    if(System.currentTimeMillis()-time > 1200)
                        break;
                }
                gWheelSystem.setPower(0);

                //start the second deployment
                if(!startPosition.isFront()){
                }
            }*/
            ifrun = false;
        }
    }
}