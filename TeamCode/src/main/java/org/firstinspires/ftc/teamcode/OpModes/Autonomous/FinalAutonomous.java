package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.EverglowLibrary.Systems.CameraSystem;

import static java.lang.Math.abs;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.Systems.GWheelSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;
import org.EverglowLibrary.ThreadHandleLib.SequenceControl;
import org.EverglowLibrary.ThreadHandleLib.SequenceRunner;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class FinalAutonomous {

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


    public double trueAngle(double angel) {
        if(angel > PI /2) {
            angel -= PI;
        }
        else if(angel < PI /2) {
            angel += PI;
        }
        return angel;
    }

    private void turnTo(double angel) {
        double currentAngle = LocationInTiles().getHeading();
        double dev = trueAngle(angel - currentAngle);
        drive.turn(dev);
    }


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
    public static Pose2d PoseInTiles(double x, double y, double Heading) {
        return new Pose2d(x * TILE_LENGTH, y * TILE_LENGTH, Heading);
    }

    public Pose2d LocationInTiles() {
        Pose2d location = drive.getPoseEstimate();
        return new Pose2d(location.getX() / TILE_LENGTH, location.getY() / TILE_LENGTH, trueAngle(Math.toRadians(location.getY())));
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

    public Pose2d mirrorToFront(Pose2d pose) {
        return new Pose2d(6 * TILE_LENGTH - pose.getX(),  pose.getY(), pose.getHeading());
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

    public Pose2d tryFront(Pose2d pose){
        if(!isBack() && pose.getX() > 3 * TILE_LENGTH) {
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

    SequenceRunner sequenceRunner = new SequenceRunner();

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

    Sequence getReadyToDrop, returnSystemsToStart, returnFromPurple;
    StartPosition startPosition = StartPosition.FRONT_LEFT;
    CameraSystem.DetectionLocation propPlace = CameraSystem.DetectionLocation.RIGHT;

    Trajectory splineToPurple, trajMiddleForBack;

    Trajectory tempPark;
    ThreeTrajectories threePurpleDropTrajectories;

    ThreeTrajectories threeYellowDropTrajectories;
    ThreeTrajectories threeParkTrajectories;
    ThreeTrajectories threeMiddleForBackTrajectories;

    Sequence dropPurpleSeq;

    Sequence dropYellow;

    Pose2d StartLocation;



    ////////////////////
    ////////////////////
    // Main Functions //
    ////////////////////
    ////////////////////

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

        fourBarSystem.restart();

        sequenceControl = new SequenceControl(clawSystem, fourBarSystem, elevatorSystem);
        sequenceRunner = new SequenceRunner();
        threeParkTrajectories = new ThreeTrajectories(drive);
        threePurpleDropTrajectories = new ThreeTrajectories(drive);
        threeYellowDropTrajectories = new ThreeTrajectories(drive);
        threeMiddleForBackTrajectories = new ThreeTrajectories(drive);

        // Create all Sequences //
        dropPurpleSeq  = new Sequence(false,
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.LOW, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
        );

        dropYellow = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)
        );

        getReadyToDrop  = sequenceControl.GetReadyToDropSeq();
        returnSystemsToStart = sequenceControl.DropAndRetreatSeq();
        returnFromPurple = sequenceControl.DropAndRetreatSeq();


        // Trajectory Calculations //

        StartLocation = PoseInTiles(3.71, 5.6, East);

        if(isBack()) {
            opMode.telemetry.addLine("move the pose...");
            StartLocation = StartLocation.minus(PoseInTiles(2.42, 0, 0));
        }

        StartLocation = tryRight(StartLocation);
        drive.setPoseEstimate(StartLocation);

        // Complex drive to Purple //

        threePurpleDropTrajectories.createThreePoseStart(new ThreePose(StartLocation));

        ThreePose parkLocation;
        double correctPark;
        double rightBuffer = 0;
        if(isBack()) {
            ThreePose purpleDropLocation;
            if(isRight()){
                purpleDropLocation = new ThreePose(
                        tryRight(PoseInTiles(1.1, 4.01, South)), //middle
                        tryRight(PoseInTiles(1.55, 4.3, North)), //left
                        tryRight(PoseInTiles(1.383, 4.3, South)) //right
                );
            }
            else{
                purpleDropLocation = new ThreePose(
                        tryRight(PoseInTiles(1.1, 4.01, South)), //middle
                        tryRight(PoseInTiles(1.383, 4.3, South)), //left
                        tryRight(PoseInTiles(1.55, 4.3, North)) //right
                );
            }
            rightBuffer = 0.35;
            Pose2d purpleHalfWayLocation = tryRight(PoseInTiles(0.5,5,South));

            threePurpleDropTrajectories.trajbuilderLeft
                    .lineToSplineHeading(purpleHalfWayLocation);

            threePurpleDropTrajectories.trajbuilderLeft
                    .splineToConstantHeading(purpleDropLocation.poseLeft.vec(),purpleDropLocation.poseLeft.getHeading());

            threePurpleDropTrajectories.trajbuilderRight
                    .splineToLinearHeading(purpleDropLocation.poseRight,purpleDropLocation.poseRight.getHeading());

            threePurpleDropTrajectories.trajbuilderMiddle
                    .lineToSplineHeading(purpleHalfWayLocation)
                    .splineToConstantHeading(purpleDropLocation.poseMiddle.vec(),purpleDropLocation.poseMiddle.getHeading());

            //purpleDropLocation.poseLeft,purpleDropLocation.poseLeft.getHeading());
            //threePurpleDropTrajectories.addLineToSplineHeading(new ThreePose(purpleHalfWayLocation));
            //threePurpleDropTrajectories.addConstHeadingTraj(purpleDropLocation);

            ThreePose between = new ThreePose(
                    tryRight(purpleDropLocation.poseMiddle.plus(
                            PoseInTiles(0,3.9-purpleDropLocation.poseMiddle.getY()/TILE_LENGTH,0))),
                    tryRight(purpleDropLocation.poseLeft.plus(
                            PoseInTiles(0,3.9-purpleDropLocation.poseLeft.getY()/TILE_LENGTH,0))),
                    tryRight(purpleDropLocation.poseRight.plus(
                            PoseInTiles(-0.1,3.9-purpleDropLocation.poseRight.getY()/TILE_LENGTH,0)))
            );

            threeMiddleForBackTrajectories.createThreePoseStart(purpleDropLocation);
            threeMiddleForBackTrajectories.addLineToSplineHeading(between);

            ThreePose waitLoc = new ThreePose(tryRight(PoseInTiles(4.5, 3.5, South)));
            waitLoc.poseRight = waitLoc.poseRight.plus(tryRight(PoseInTiles(0,rightBuffer,0)));

            threeMiddleForBackTrajectories.addConstHeadingTraj(waitLoc);

            threeMiddleForBackTrajectories.endLocations = waitLoc;
            correctPark = 3.5;
            threeYellowDropTrajectories.startLocations = threeMiddleForBackTrajectories.endLocations;
            threeMiddleForBackTrajectories.buildTrajs();
            threePurpleDropTrajectories.buildTrajs();
        }
        else{
//            threePurpleDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(4.2, 4, East));
//            threePurpleDropTrajectories.endLocations.poseLeft = tryRight(PoseInTiles(4.35, 3.5, East));
//            threePurpleDropTrajectories.endLocations.poseRight =tryRight( PoseInTiles(0.39, 3.5, East));
//            threePurpleDropTrajectories.createConstHeadingTrajectories();
            tryRight()
            correctPark = 5.5;
            threeYellowDropTrajectories.startLocations = threePurpleDropTrajectories.endLocations;
        }


        // yellow Trajectories //
        threeYellowDropTrajectories.endLocations.poseMiddle = tryRight(PoseInTiles(5, 4.5, South));
        threeYellowDropTrajectories.endLocations.poseLeft =
                threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                        new Pose2d(0, distanceBetweenTags, 0));
        threeYellowDropTrajectories.endLocations.poseRight
                = threeYellowDropTrajectories.endLocations.poseMiddle.plus(
                new Pose2d(0, -distanceBetweenTags + (rightBuffer * TILE_LENGTH), 0));

        threeYellowDropTrajectories.createConstHeadingTrajectories();

        // park //
        ThreePose dropYellowLocs = threeYellowDropTrajectories.endLocations;
        ThreePose between = new ThreePose(
                tryRight(dropYellowLocs.poseMiddle.plus(
                        PoseInTiles(-0.2,correctPark-dropYellowLocs.poseMiddle.getY()/TILE_LENGTH,0))),
                tryRight(dropYellowLocs.poseLeft.plus(
                        PoseInTiles(-0.2,correctPark-dropYellowLocs.poseLeft.getY()/TILE_LENGTH,0))),
                tryRight(dropYellowLocs.poseRight.plus(
                        PoseInTiles(-0.2,correctPark+rightBuffer-dropYellowLocs.poseRight.getY()/TILE_LENGTH,0)))
        );

        threeParkTrajectories.createThreePoseStart(threeYellowDropTrajectories.endLocations);
        threeParkTrajectories.addConstHeadingTraj(between);

        parkLocation = new ThreePose(
                between.poseMiddle.plus(new Pose2d(TILE_LENGTH/1.6, 0, 0)),
                between.poseLeft.plus(new Pose2d(TILE_LENGTH/1.6,0,0)),
                between.poseRight.plus(new Pose2d(TILE_LENGTH/1.6,0,0)));

        threeParkTrajectories.addConstHeadingTraj(parkLocation);
        threeParkTrajectories.buildTrajs();

        clawSystem.MoveOneClaw(true);

        opMode.telemetry.addLine("Ready!!!!! ");
        opMode.telemetry.update();

        opMode.waitForStart();

        if(opMode.isStopRequested())
            return;

        runAfterInput();

    }


    public void runAfterInput() {

        fourBarSystem.setMotorPower(0.85);
        propPlace = cameraSystem.DetectAndFindPropLocation();
        opMode.telemetry.addData("Detection ", propPlace);
        opMode.telemetry.update();

        //drive.followTrajectory(splineToPurple); //tod
        dropPurpleSeq.startSequence();

        threePurpleDropTrajectories.driveCorrectTrajectory(propPlace);

        // Drop Purple

        while (!opMode.isStopRequested()){
            if(dropPurpleSeq.isDone()){
                break;
            }
        }

        clawSystem.ChangePos(true);
        opMode.sleep(1000);
        // Sequance drop
        returnFromPurple.startSequence();

        while (!opMode.isStopRequested()){
            if(returnFromPurple.isDone()){
                break;
            }
        }

    }

    public void SecondCall(){
        Trajectory backWaitForMove;
        Trajectory betweenWait = null;
        /*
        if(isBack()) {
            if (propPlace != CameraSystem.DetectionLocation.MIDDLE) {
                if (isRight()) {
                    backWaitForMove = drive.trajectoryBuilder(PoseInTiles(1.5,2.5,0))
                            .lineToConstantHeading(PoseInTiles(4.5, 2.5, 0).vec())
                            .build();
                    betweenWait = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(1.5, 2.5, PI))
                            .build();
                } else {
                    backWaitForMove = drive.trajectoryBuilder(PoseInTiles(1.5, 3.5,PI))
                            .lineToConstantHeading(PoseInTiles(4.5, 3.5, 0).vec())
                            .build();
                    betweenWait = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(1.5, 3.5, PI))
                            .build();
                }
            }
            else {
                if(isRight()){
                    backWaitForMove = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(0.5, 4.5, PI))
                            .splineTo(PoseInTiles(2.5, 2.5, 0).vec(), -PI)
                            .splineToConstantHeading(PoseInTiles(4.5, 2.5, 0).vec(), 0)
                            .build();
                }
                else {
                    backWaitForMove = drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineToSplineHeading(PoseInTiles(0.5, 4.5, PI))
                            .splineTo(PoseInTiles(0.5, 3.5, 0).vec(), PI)
                            .splineToConstantHeading(PoseInTiles(4.5, 3.5, 0).vec(), 0)
                            .build();
                }
            }

         */
        //drive.followTrajectory(backWaitForMove);
        threeMiddleForBackTrajectories.driveCorrectTrajectory(propPlace);
//            if(isBack() && propPlace == CameraSystem.DetectionLocation.RIGHT){
//                //drive.turn(PI);
//                drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
//                        .lineToSplineHeading(tryRight(drive.getPoseEstimate().plus(new Pose2d(0,20,PI))))
//                        .build());
//            }
        //opMode.sleep(10000);

        dropYellow.startSequence();
        threeYellowDropTrajectories.driveCorrectTrajectory(propPlace);

        while (!opMode.isStopRequested()){
            if(dropYellow.isDone())
                break;
        }

        clawSystem.toggle();
        opMode.sleep(300);
        drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                .lineToConstantHeading(drive.getPoseEstimate().plus(new Pose2d(0,-30,0)).vec()).build());
        returnSystemsToStart.startSequence();
        threeParkTrajectories.driveCorrectTrajectory(propPlace);

        while (!opMode.isStopRequested()){
            if(returnSystemsToStart.isDone())
                break;
        }
        //drop
        //gWheelSystem.setPower(1);

        //opMode.sleep(2500);

        //gWheelSystem.setPower(0);
        }
    }

