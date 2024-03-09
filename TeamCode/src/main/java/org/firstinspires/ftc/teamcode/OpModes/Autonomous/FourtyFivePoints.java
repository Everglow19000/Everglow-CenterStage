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
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
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

    public Pose2d LocationInTiles() {
        Pose2d location = drive.getPoseEstimate();
        return new Pose2d(location.getX() / TILE_LENGTH, location.getY() / TILE_LENGTH, location.getY());
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
                .splineTo(poseFinish.vec(), poseFinish.getHeading())
                .build();
    }
    public Trajectory trajToConstantHeading(Pose2d startPose, Pose2d endPose){
        return drive.trajectoryBuilder(startPose)
                .splineToConstantHeading(endPose.vec(),endPose.getHeading())
                .build();
    }
    public Trajectory trajToLinearHeading(Pose2d startPose, Pose2d endPose){
        return drive.trajectoryBuilder(startPose)
                .splineToLinearHeading(endPose,endPose.getHeading())
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

        public void createLinerHeadingTrajectories() {
            trajMiddle = trajToLinearHeading(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToLinearHeading(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToLinearHeading(startLocations.poseRight, endLocations.poseRight);
        }

        public void createConstHeadingTrajectories() {
            trajMiddle = trajToConstantHeading(startLocations.poseMiddle, endLocations.poseMiddle);
            trajLeft = trajToConstantHeading(startLocations.poseLeft, endLocations.poseLeft);
            trajRight = trajToConstantHeading(startLocations.poseRight, endLocations.poseRight);
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

    CameraSystem cameraSystem;

    SequenceRunner sequenceRunner;





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

    Trajectory splineToPurple;


    //ThreePose yellowDropLocations = new ThreePose();

    ThreeTrajectories threeYellowDropTrajectories = new ThreeTrajectories();
    ThreeTrajectories threeParkTrajectories = new ThreeTrajectories();
    ThreeTrajectories threeMiddleForBackTrajectories = new ThreeTrajectories();






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


    private  void Sleep(double seconeds) {
        double time = 1000000 * seconeds;
        sleep((int)time);
    }



    public void firstCall(LinearOpMode opMode, StartPosition startPosition) throws InterruptedException{
        // Initalize Systems //
        this.opMode = opMode;
        this.startPosition = startPosition;
        drive = new SampleMecanumDrive(opMode.hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cameraSystem = new CameraSystem(opMode, isRight(), isBack());


        elevatorSystem = new ElevatorSystem(opMode);
        clawSystem = new ClawSystem(opMode);
        fourBarSystem = new FourBarSystem(opMode);
        gWheelSystem = new GWheelSystem(opMode);

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        sequenceRunner = new SequenceRunner();


        // Create all Sequences //

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();


        // Trajectory Calculations //
        Pose2d startLocation = PoseInTiles(3.61, 5.667, East);
        if(isBack()) {
            startLocation.minus(PoseInTiles(2, 0, 0));
        }
        if(isRight()) {
            startLocation.minus(PoseInTiles(0.27, 0, 0));
        }
        drive.setPoseEstimate(tryRight(startLocation));


        Pose2d middleDropLocation = PoseInTiles(3.5, 4.5, East);
        if(isBack()) {
            middleDropLocation.minus(PoseInTiles(2, 0, 0));
        }
        splineToPurple = trajToLinearHeading(tryRight(startLocation), tryRight(middleDropLocation));

        if(isBack()) {
            threeMiddleForBackTrajectories.setStartPose(tryRight(PoseInTiles(middleDropLocation.getX(), middleDropLocation.getY(), South)));

            Pose2d waitPosition = PoseInTiles(3.5, 3.5, South);
            threeMiddleForBackTrajectories.setEndPose(waitPosition);
            threeMiddleForBackTrajectories.createConstHeadingTrajectories();

            threeYellowDropTrajectories.setStartPose((tryRight(waitPosition)));


        }

        else{
            threeYellowDropTrajectories.setStartPose(tryRight(PoseInTiles(middleDropLocation.getX(), middleDropLocation.getY(), South)));
        }








        threeYellowDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(5, 4.5, South));
        threeYellowDropTrajectories.endLocations.poseLeft =
                threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                        new Pose2d(0, -distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight
                = threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                new Pose2d(0, distanceBetweenTags, 0));
        threeYellowDropTrajectories.createTrajectories();



        Pose2d parkLocation = PoseInTiles(5.55, 5.6, South);
        if(isBack()) {
            parkLocation.minus(PoseInTiles(0, 2, 0));
        }
        threeParkTrajectories.setEndPose(tryRight(parkLocation));
        threeParkTrajectories.startLocations = threeYellowDropTrajectories.endLocations;
        threeParkTrajectories.createTrajectories();

        //

        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        runAfterInput();

    }
    
    
    public void runAfterInput() {


        propPlace = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Detection ", propPlace);
        opMode.telemetry.update();
        //Sleep(2);
        // set the correct start location

        drive.followTrajectory(splineToPurple);

        opMode.telemetry.addData("Location ", LocationInTiles());
        opMode.telemetry.addData("start ", threeYellowDropTrajectories.startLocations.poseMiddle);
        opMode.telemetry.update();

        Sleep(7);

        //drive.turnAsync(South);

/*       // GWheel - Drop Purple
        switch (propPlace) {
            case LEFT:
                drive.turn(PI / 2); //todo: if the turn is oposside (-Pi/2) then change it like it
                break;

            case RIGHT:
                drive.turn(-PI / 2); //todo: if the turn is oposside Pi/2 then change it like it
                break;
        }

        gWheelSystem.setPower(0.4);

        long time = System.currentTimeMillis();
        while(!opMode.isStopRequested()){
            if(System.currentTimeMillis()-time > 1000)
                break;
        }

        gWheelSystem.setPower(0); // G Wheel

        if(opMode.isStopRequested())
            return;

//        switch (propPlace) {
//            case LEFT:
//                if(isRight()) {
//                    drive.turn(-PI / 2);
//                }
//                else {
//                    drive.turn(PI / 2);
//                }
//                break;
//
//            case RIGHT:
//                if(isRight()) {
//                    drive.turn(PI / 2);
//                }
//                else {
//                    drive.turn(-PI / 2);
//                }
//                break;
//        }*/




        opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);

        opMode.telemetry.update();

        Sleep(3);


        threeYellowDropTrajectories.driveCorrectTrajectory();

        opMode.telemetry.addData("Location ", LocationInTiles());
        opMode.telemetry.addData("end ", threeYellowDropTrajectories.endLocations.poseMiddle);
        opMode.telemetry.update();
        Sleep(10);

    }

}


