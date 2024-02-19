package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


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
import org.firstinspires.ftc.teamcode.OpModes.DriverControl.SequenceControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.ExecutorUtils.ExecutorTrajectories;

@Autonomous(name = "Basic Autunumous Red")
public class MoveToConusAutonumous extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException {
       //AutonumousGeneral(this, StartPosition.FRONTLEFT);
    }

    public static CameraSystem.DetectionLocation AutonumousGeneral(LinearOpMode opMode
            , StartPosition startPosition, SampleMecanumDrive drive){
        double angleRatio = 1;
        GWheelSystem gWheelSystem = new GWheelSystem(opMode);
        CameraSystem cameraSystem = new CameraSystem(opMode, !startPosition.isLeft());
        //Sequence drop = sequenceControl.DropAndRetreatSeq();
        double xPoint = 85, yPoint = -15;

        Trajectory splneToMiddle = drive.trajectoryBuilder(new Pose2d(0,0, 0))
                .splineTo(new Vector2d(xPoint,yPoint), 0)
                .build();

        boolean ifrun = true;
        double moveInYAxis = 0;
        double moveInXAxis = 0;

        Trajectory splineTofinalPos;

        opMode.waitForStart();


        CameraSystem.DetectionLocation location  = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Target", location);
        opMode.telemetry.addData("startPos", startPosition.name());
        opMode.telemetry.update();

        if (opMode.isStopRequested()) return location;
        double angle = Math.toRadians(-90);

        //first deployment
        drive.followTrajectory(splneToMiddle);

        // locate and put the first pixel //
        switch (location){
            case RIGHT:
                if (!startPosition.isLeft() && startPosition.isFront()) {
                    moveInYAxis += SQUARE_SIZE*2;
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .strafeRight(moveInYAxis).build());
                    drive.turn(angle);
                    break;
                }
                drive.turn(angle);
                break;

            case MIDDLE:
                if (!startPosition.isFront()) {
                    moveInXAxis += SQUARE_SIZE;
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .forward(moveInXAxis).build());
                    drive.turn(2*angle);
                }
                break;

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

        //start the second deployment//
        return location;
    }
}

 /*
package org.firstinspires.ftc.teamcode.OpModes.Autonomous;


import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

    static boolean IsFirstRun = true;
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


    public static void AutonumousGeneral(LinearOpMode opMode, StartPosition startPosition) {
        // inlasation //
        GWheelSystem gWheelSystem = new GWheelSystem(opMode);
        CameraSystem cameraSystem = new CameraSystem(opMode, !startPosition.isLeft());
        FourBarSystem fourBarSystem = new FourBarSystem(opMode);
        ElevatorSystem elevatorSystem = new ElevatorSystem(opMode);
        ClawSystem clawSystem = new ClawSystem(opMode);
        SampleMecanumDrive drive = new SampleMecanumDrive(opMode.hardwareMap);
        SequenceControl sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        Sequence getReadyToDrop = sequenceControl.GetReadyToDropSeq();
        Sequence drop = sequenceControl.DropAndRetreatSeq();

        //before waitForStart //
        Pose2d startLocation = new Pose2d();
        if (startPosition.isLeft()) startLocation = new Pose2d(0, 0, East);
        else startLocation = new Pose2d(0, 0, West);

        opMode.waitForStart();

        // start opMode - calculations //

        CameraSystem.DetectionLocation teamProptIden = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Target", teamProptIden);
        opMode.telemetry.addData("startPos", startPosition.name());


        double purpleDropXPoint = 82, purpleDropYPoint = -15, purpleDropAngle = East;
        switch (teamProptIden) {
            case RIGHT:
                purpleDropAngle = South;
            case LEFT:
                purpleDropAngle = North;
        }

        if (!startPosition.isLeft()) {
            if (teamProptIden == CameraSystem.DetectionLocation.MIDDLE) purpleDropAngle = West;
            purpleDropXPoint = -purpleDropXPoint;
        }

        Pose2d purpleDropLocation = new Pose2d(purpleDropXPoint, purpleDropYPoint, purpleDropAngle);


        opMode.telemetry.addData("purpleDropXPoint", purpleDropXPoint);
        opMode.telemetry.addData("purpleDropYPoint", purpleDropYPoint);
        opMode.telemetry.addData("angle", purpleDropAngle);
        opMode.telemetry.addData("startLocation:", startLocation);
        opMode.telemetry.update();

        Trajectory trajectoryPurpleDrop = drive.trajectoryBuilder(startLocation)
                .splineTo(new Vector2d(purpleDropXPoint, purpleDropYPoint), purpleDropAngle)
                .build();


        // seconed Drop

        double yellowDropXPoint = 75;
        if (!startPosition.isLeft()) yellowDropXPoint = -yellowDropXPoint;
        double tagXDistance = 15;
        switch (teamProptIden) {
            case LEFT:
                yellowDropXPoint += tagXDistance;
            case RIGHT:
                yellowDropXPoint -= tagXDistance;
        }

        double yellowDropYPoint = 100;
        if (!startPosition.isFront()) yellowDropYPoint += 2 * SQUARE_SIZE;

        Trajectory trajectoryYellowDrop = drive.trajectoryBuilder(purpleDropLocation)
                .splineTo(new Vector2d(purpleDropXPoint, purpleDropYPoint), North)
                .build();


        // run everything //

        if (opMode.isStopRequested()) return;

        drive.followTrajectory(trajectoryPurpleDrop);

        if (opMode.isStopRequested()) return;

        opMode.sleep(3000);
        gWheelSystem.toggle(true);
        opMode.sleep(3000);

        gWheelSystem.toggle(true);

        if (opMode.isStopRequested()) return;

        drive.followTrajectory(trajectoryYellowDrop);

    }
}

  */