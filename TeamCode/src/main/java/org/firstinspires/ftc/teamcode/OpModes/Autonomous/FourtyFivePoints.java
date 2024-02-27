package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static org.EverglowLibrary.Systems.Executor.sleep;
import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Systems.CameraSystem;
import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FourtyFivePoints {

    ///////////////////////////////
    ///////////////////////////////
    // Start Position functions  //
    ///////////////////////////////
    ///////////////////////////////

    public enum StartPosition{
        FRONT_LEFT(new Vector2d(0.25, 3.75)), FRONT_RIGHT(new Vector2d(5.75, 3.75))
        , BACK_LEFT(new Vector2d(0.25, 1.75)), BACK_RIGHT(new Vector2d(5.75, 1.75));

        public final Vector2d startPositionVector;
        StartPosition(Vector2d startPosition) {
            this.startPositionVector = startPosition;
        }

        public boolean isRight() {
            if(this == FRONT_RIGHT || this == BACK_RIGHT)
                return true;
            else
                return false;
        }

        public boolean isBack() {
            if(this == BACK_LEFT || this == BACK_RIGHT)
                return true;
            else
                return false;
        }
    }


    /**
     * Does the robot start on the right(Red Alliance)
     *
     * @return 0 - start on left, 1 - starts on right
     */
    public boolean isRight() { return startPosition.isRight(); }


    /**
     * Does the robot start in the back of the filed
     *
     * @return 0 - start in Front, 1 - starts in the Back
     */
    public boolean isBack() { return startPosition.isBack(); }





    /////////////////////////////////////////////
    /////////////////////////////////////////////
    // Useful Functions That Manipulate Pose2d //
    /////////////////////////////////////////////
    /////////////////////////////////////////////

    /**
     * Creates a new Pose2d but with X and Y values multiplaied by Tile Length
     *
     * @param x The X value of the robot Position according to filed Axis - and in Tile units
     * @param y The Y value of the robot Position according to filed Axis - and in Tile units
     * @param Heading The angle of the Robot in Relation to the filed - 0 is North
     *
     * @return new Pose2d in CM Units
     */
    public Pose2d PoseInTiles(double x, double y, double Heading) {
        return new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading);
    }


    /**
     * Mirrors any Pose2d from the left side to the right side
     * Use to turn any Pose that is written for a Left side Autonomous to work for Right side Autonomous
     *
     * @param pose the Location of the Robot on the Filed
     * @return new Pose2d to use for a Right-Start Autonomous
     */
    public Pose2d mirrorToRight(Pose2d pose) {
        return new Pose2d(pose.getX(), 6 * TILE_LENGTH - pose.getY(), -pose.getHeading());
    }


    /**
     * Makes any Pose2d be Correct for Both Left(Blue Alliance) and Right(red Alliance) Autonomous
     * by Mirroring to the right - if and only if needed - according to the startPosition.
     *
     * @param pose the Location of the Robot on the Filed
     * @return new Pose2d to use
     */
    public Pose2d tryRight(Pose2d pose) {
        if(isRight() && pose.getY() > 3 * TILE_LENGTH) {
            pose = mirrorToRight(pose);
        }
        return pose;
    }


    /**
     * Mirrors any Pose2d using the trust as the mirror line, for nerow use to copy from a Front Autonomous to a Back Autonomous
     *
     * @param pose the Location of the Robot on the Filed, written for a Front Autonomous
     * @return new Pose2d Mirrored to the Trust
     */
    public Pose2d mirrorToTrust(Pose2d pose) {
        final double middleOfTrust = 2.5 * TILE_LENGTH;
        if(pose.getHeading() >= 0) return new Pose2d(2 * middleOfTrust - pose.getX(), pose.getY(), PI - pose.getHeading());
        return new Pose2d(2 * middleOfTrust - pose.getX(), pose.getY(), -PI - pose.getHeading());
    }

    public Trajectory trajToPose(Pose2d poseStart, Pose2d poseFinish) {
        return drive.trajectoryBuilder(poseStart)
                .splineTo(new Vector2d(poseFinish.getX(), poseFinish.getY()), poseFinish.getHeading())
                .build();
    }


    public class ThreePose {
        public Pose2d poseMiddle, poseLeft, poseRight;
        ThreePose(Pose2d poseMiddle, Pose2d poseLeft, Pose2d poseRight) {
            this.poseMiddle = poseMiddle;
            this.poseLeft = poseLeft;
            this.poseRight = poseRight;
        }
        ThreePose() {
            this.poseMiddle = new Pose2d();
            this.poseLeft = new Pose2d();
            this.poseRight = new Pose2d();
        }
    }


    public class ThreeTrajectories {
        public ThreePose startLocations = new ThreePose(), endLocations = new ThreePose();
        public Trajectory trajMiddle, trajLeft, trajRight;


        public void setStartPose(Pose2d startLocation) {
            startLocations = new ThreePose(startLocation, startLocation, startLocation);
        }
        public void setEndPose(Pose2d endLocation) {
            endLocations = new ThreePose(endLocation, endLocation, endLocation);
        }

        public void createTrajectories() {
            trajMiddle = trajToPose(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToPose(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToPose(startLocations.poseRight, endLocations.poseRight);
        }


        public void driveCorrectTrajectory() {
            switch (propPlace) {
                case MIDDLE:
                    drive.followTrajectory(trajMiddle);
                case LEFT:
                    drive.followTrajectory(trajLeft);
                case RIGHT:
                    drive.followTrajectory(trajRight);
            }
        }
    }





    /////////////
    /////////////
    // Systems //
    /////////////
    /////////////

    LinearOpMode opMode;
    SequenceControl sequenceControl;
    SampleMecanumDrive drive;
    ElevatorSystem elevatorSystem;
    ClawSystem clawSystem;
    FourBarSystem fourBarSystem;
    GWheelSystem gWheelSystem;





    ///////////////
    ///////////////
    // Constants //
    ///////////////
    ///////////////

    static double distanceOfPropFromRobot = 67; //in cm
    static double distanceBetweenTags=15.0; //in cm
    static double distanceBuffer=0;

    static final double North = 0;
    static final double East = -PI/2;
    static final double West = PI/2;
    static final double South = PI;

    static final double TILE_LENGTH = 60.5;






    ///////////////////////////
    ///////////////////////////
    // Autonomous Parameters //
    ///////////////////////////
    ///////////////////////////

    Sequence getReadyToDrop, returnSystemsToStart;
    StartPosition startPosition = StartPosition.FRONT_LEFT;
    CameraSystem.DetectionLocation propPlace = CameraSystem.DetectionLocation.RIGHT;

    Trajectory trajLeftYellow, trajRightYellow, trajMiddleYellow, splineToPurple;
    Trajectory trajParkMiddle, trajParkLeft, trajParkRight;

    //ThreePose yellowDropLocations = new ThreePose();

    ThreeTrajectories threeYellowDropTrajectories = new ThreeTrajectories();
    ThreeTrajectories threeParkTrajectories = new ThreeTrajectories();






    ////////////////////
    ////////////////////
    // Main Functions //
    ////////////////////
    ////////////////////

    /**
     * Set the Start Position (Witch Alliance and Front or Back), a Must Run For any Autonomous using this class.
     *
     * @param startPosition FRONT_LEFT or FRONT_RIGHT or BACK_LEFT or BACK_RIGHT
     *
     * @return new Pose2d in CM Units
     */
    public void chooseStartPosition(StartPosition startPosition) {
        this.startPosition = startPosition;
    }


    public void firstCall(LinearOpMode opMode, StartPosition startPosition) throws InterruptedException{
        // Initalize Systems //
        this.opMode = opMode;
        this.startPosition = startPosition;
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        CameraSystem cameraSystem = new CameraSystem(opMode, isRight());


        /*elevatorSystem = new ElevatorSystem(this);
        clawSystem = new ClawSystem(this);
        fourBarSystem = new FourBarSystem(this);
        gWheelSystem = new GWheelSystem(this);

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);

        // Create all Sequences //

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();*/


        // Trajectory Calculations //
        Pose2d startLocation = PoseInTiles(3.636, 5.667, East);
        if(isBack()) {
            startLocation.minus(PoseInTiles(2, 0, 0));
        }
        if(isRight()) {
            startLocation.minus(PoseInTiles(0.27, 0, 0));
        }
        drive.setPoseEstimate(tryRight(startLocation));



        Pose2d middleDropLocation = PoseInTiles(3.5, 4.5, East);
        splineToPurple = trajToPose(tryRight(startLocation), tryRight(middleDropLocation));

        threeYellowDropTrajectories.setStartPose(tryRight(middleDropLocation));
        threeYellowDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(5, 4.5, South));
        threeYellowDropTrajectories.endLocations.poseLeft = threeYellowDropTrajectories.endLocations.poseMiddle.plus(new Pose2d(0, -distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight = threeYellowDropTrajectories.endLocations.poseMiddle.plus(new Pose2d(0, distanceBetweenTags, 0));
        threeYellowDropTrajectories.createTrajectories();



        Pose2d parkLocation = PoseInTiles(5.55, 5.6, South);
        threeParkTrajectories.setEndPose(parkLocation);
        threeParkTrajectories.startLocations = threeYellowDropTrajectories.endLocations;
        threeParkTrajectories.createTrajectories();



        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        propPlace = cameraSystem.DetectAndFindPropLocation();

        runAfterInput();
    }
    
    
    public void runAfterInput() {

        // set the correct start location

        drive.followTrajectory(splineToPurple);

        // GWheel - Drop Purple
        /*switch (propPlace) {
            case LEFT:
                if(isRight()) {
                    drive.turn(PI / 2);
                }
                else {
                    drive.turn(-PI / 2);
                }

            case RIGHT:
                if(isRight()) {
                    drive.turn(-PI / 2);
                }
                else {
                    drive.turn(PI / 2);
                }
        }

        gWheelSystem.setPower(0.4);
        if (opMode.isStopRequested()) return;
        long time = System.currentTimeMillis();
        while(opMode.isStopRequested()){
            if(System.currentTimeMillis()-time > 1000)
                break;
        }
        gWheelSystem.setPower(0); // G Wheel

        switch (propPlace) {
            case LEFT:
                if(isRight()) {
                    drive.turn(-PI / 2);
                }
                else {
                    drive.turn(PI / 2);
                }

            case RIGHT:
                if(isRight()) {
                    drive.turn(PI / 2);
                }
                else {
                    drive.turn(-PI / 2);
                }
        }*/



        opMode.telemetry.addData("start ", threeYellowDropTrajectories.startLocations.poseMiddle);
        opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);


        opMode.telemetry.update();
        sleep(3000);

        //threeYellowDropTrajectories.driveCorrectTrajectory();

        /*getReadyToDrop.startSequence();

        // Yellow Drop Location
        switch(propPlace) {
            case MIDDLE:
                drive.followTrajectory(trajMiddleYellow);

            case LEFT:
                drive.followTrajectory(trajLeftYellow);

            case RIGHT:
                drive.followTrajectory(trajRightYellow);

        }

        clawSystem.toggle();

        returnSystemsToStart.startSequence();


        switch(propPlace) {
            case MIDDLE:
                drive.followTrajectory(trajParkMiddle);

            case LEFT:
                drive.followTrajectory(trajParkLeft);

            case RIGHT:
                drive.followTrajectory(trajParkRight);

        }*/


        // spline to drop Purple

        // Drop Purple
    }

}

