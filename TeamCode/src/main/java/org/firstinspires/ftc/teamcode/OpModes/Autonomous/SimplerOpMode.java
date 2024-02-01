package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.CameraSystem;


@TeleOp(name = "SimplerOpMode")
public class SimplerOpMode extends LinearOpMode {
    static double squareSize = 60.5; //in cm
    static double distanceOfPropFromRobot = 67; //in cm
    static double distanceBetweenTags=15.0; //in cm
    static double distanceBuffer=0;

    public enum PropPlace {
        LEFT(new Vector2d(0, 0)), MIDDLE(new Vector2d(0, 0)), RIGHT(new Vector2d(0, 0));
        public final Vector2d propPlaceVector;

        PropPlace(Vector2d propPlace) {
            this.propPlaceVector = propPlace;
        }
    }

    public enum StartPosition{
        FRONTLEFT(new Vector2d(0.25, 3.75)), FRONTRIGHT(new Vector2d(5.75, 3.75))
        , BACKLEFT(new Vector2d(0.25, 1.75)), BACKRIGHT(new Vector2d(5.75, 1.75));

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

    public static void runAfterInput(SimplerOpMode.StartPosition startPosition, CameraSystem.DetectionLocation propPlace) {
        Pose2d startLocation;
        final double RightLainMiddleX = 1.5 * squareSize;
        final double LeftLainMiddleX = 4.5 * squareSize;

        final double RightLainFirstDropX = 1.75 * squareSize;
        final double LeftLainFirstDropX = 4.25 * squareSize;

        final double RightStartPosition = 0.5 *squareSize;
        final double LeftStartPosition = 5.5 *squareSize;



        final double BackLainMiddleY = 1.5 * squareSize;

        final double trustY = 2.5 * squareSize;

        final double FrontLainMiddleY = 3.5 * squareSize;

        final double FinalDropY = 5 * squareSize;



        final double North = 0;
        final double East = -PI/2;
        final double West = PI/2;
        final double South = PI;

        //final Pose2d LeftLainSecondDrop = new Pose2d(LeftLainMiddleX, FinalDropY, South);
        //final Pose2d RightLainSecondDrop = new Pose2d(RightLainMiddleX, FinalDropY, South);


        final double forwardDeviationDistance = 0.9 * squareSize;

        Pose2d FirstDropMiddlePoint, FirstDropLocation, SecondDropMiddlePoint, SecondDropLocation;

        if(startPosition.isLeft()) SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, South);
        else SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, South);


        if(propPlace == CameraSystem.DetectionLocation.LEFT) SecondDropLocation.plus(new Pose2d(distanceBetweenTags, 0, 0));
        else if(propPlace == CameraSystem.DetectionLocation.RIGHT) SecondDropLocation.plus(new Pose2d(-distanceBetweenTags, 0, 0));


        switch (startPosition) {
            case BACKLEFT:
                startLocation = new Pose2d(LeftStartPosition, BackLainMiddleY, East);
                FirstDropMiddlePoint = new Pose2d(LeftLainMiddleX, BackLainMiddleY, East);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, trustY, South);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, BackLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose2d(LeftLainMiddleX, BackLainMiddleY, East);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, East);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case FRONTLEFT:
                startLocation = new Pose2d(LeftStartPosition, FrontLainMiddleY, East);
                FirstDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
                        SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case BACKRIGHT:
                startLocation = new Pose2d(RightStartPosition, BackLainMiddleY, West);
                FirstDropMiddlePoint = new Pose2d(RightLainMiddleX, BackLainMiddleY, West);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose2d(RightLainFirstDropX, trustY, South);
                        SecondDropMiddlePoint = new Pose2d(1*squareSize, trustY, South);
                        SecondDropLocation = new Pose2d(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose2d(RightLainFirstDropX, BackLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose2d(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose2d(RightLainMiddleX, BackLainMiddleY, West);
                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, trustY, West);
                        SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case FRONTRIGHT:
                startLocation = new Pose2d(RightStartPosition, FrontLainMiddleY, West);
                FirstDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose2d(RightLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(1*squareSize, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose2d(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose2d(RightLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose2d(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
                        SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
        }

    }



    @Override
    public void runOpMode() throws InterruptedException{
        CameraSystem cameraSystem = new CameraSystem(this);

        SimplerOpMode.StartPosition startPosition;//todo: add functionality
        CameraSystem.DetectionLocation propPlace = cameraSystem.DetectAndFindPropLocation(); // camera
//
//        Pose2d startLocation;
//        final double RightLainMiddleX = 1.5 * squareSize;
//        final double LeftLainMiddleX = 4.5 * squareSize;
//
//        final double RightLainFirstDropX = 1.75 * squareSize;
//        final double LeftLainFirstDropX = 4.25 * squareSize;
//
//        final double RightStartPosition = 0.5 *squareSize;
//        final double LeftStartPosition = 5.5 *squareSize;
//
//
//
//        final double BackLainMiddleY = 1.5 * squareSize;
//
//        final double trustY = 2.5 * squareSize;
//
//        final double FrontLainMiddleY = 3.5 * squareSize;
//
//        final double FinalDropY = 5 * squareSize;
//
//
//
//        final double North = 0;
//        final double East = -PI/2;
//        final double West = PI/2;
//        final double South = PI;
//
//        //final Pose2d LeftLainSecondDrop = new Pose2d(LeftLainMiddleX, FinalDropY, South);
//        //final Pose2d RightLainSecondDrop = new Pose2d(RightLainMiddleX, FinalDropY, South);
//
//
//        final double forwardDeviationDistance = 0.9 * squareSize;
//
//        Pose2d FirstDropMiddlePoint, FirstDropLocation, SecondDropMiddlePoint, SecondDropLocation;
//
//        if(startPosition.isLeft()) SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, South);
//        else SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, South);
//
//
//        if(propPlace == CameraSystem.DetectionLocation.LEFT) SecondDropLocation.plus(new Pose2d(distanceBetweenTags, 0, 0));
//        else if(propPlace == CameraSystem.DetectionLocation.RIGHT) SecondDropLocation.plus(new Pose2d(-distanceBetweenTags, 0, 0));
//
//
//        switch (startPosition) {
//            case BACKLEFT:
//                startLocation = new Pose2d(LeftStartPosition, BackLainMiddleY, East);
//                FirstDropMiddlePoint = new Pose2d(LeftLainMiddleX, BackLainMiddleY, East);
//                switch (propPlace) {
//                    case LEFT:
//                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, trustY, South);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, South);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
//                        break;
//                    case RIGHT:
//                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, BackLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, South);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
//                        break;
//                    default:
//                        FirstDropLocation= new Pose2d(LeftLainMiddleX, BackLainMiddleY, East);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, trustY, East);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, North);
//                        break;
//                }
//                break;
//            case FRONTLEFT:
//                startLocation = new Pose2d(LeftStartPosition, FrontLainMiddleY, East);
//                FirstDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
//                switch (propPlace) {
//                    case LEFT:
//                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, FrontLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, South);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
//                        break;
//                    case RIGHT:
//                        FirstDropLocation= new Pose2d(LeftLainFirstDropX, FrontLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, South);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
//                        break;
//                    default:
//                        FirstDropLocation= new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
//                        SecondDropMiddlePoint = new Pose2d(LeftLainMiddleX, FrontLainMiddleY, East);
//                        SecondDropLocation = new Pose2d(LeftLainMiddleX, FinalDropY, North);
//                        break;
//                }
//                break;
//            case BACKRIGHT:
//                startLocation = new Pose2d(RightStartPosition, BackLainMiddleY, West);
//                FirstDropMiddlePoint = new Pose2d(RightLainMiddleX, BackLainMiddleY, West);
//                switch (propPlace) {
//                    case LEFT:
//                        FirstDropLocation= new Pose2d(RightLainFirstDropX, trustY, South);
//                        SecondDropMiddlePoint = new Pose2d(1*squareSize, trustY, South);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
//                        break;
//                    case RIGHT:
//                        FirstDropLocation= new Pose2d(RightLainFirstDropX, BackLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, trustY, South);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
//                        break;
//                    default:
//                        FirstDropLocation= new Pose2d(RightLainMiddleX, BackLainMiddleY, West);
//                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, trustY, West);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, North);
//                        break;
//                }
//                break;
//            case FRONTRIGHT:
//                startLocation = new Pose2d(RightStartPosition, FrontLainMiddleY, West);
//                FirstDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
//                switch (propPlace) {
//                    case LEFT:
//                        FirstDropLocation= new Pose2d(RightLainFirstDropX, FrontLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(1*squareSize, FrontLainMiddleY, South);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
//                        break;
//                    case RIGHT:
//                        FirstDropLocation= new Pose2d(RightLainFirstDropX, FrontLainMiddleY, South);
//                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, South);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
//                        break;
//                    default:
//                        FirstDropLocation= new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
//                        SecondDropMiddlePoint = new Pose2d(RightLainMiddleX, FrontLainMiddleY, West);
//                        SecondDropLocation = new Pose2d(RightLainMiddleX, FinalDropY, North);
//                        break;
//                }
//                break;
//        }
//
//
//
//
//
//
//
//        drive(startLocation, FirstDropMiddlePoint, FirstDropLocation);
//        // להפעיל גג"ז
//        drive(FirstDropLocation, SecondDropMiddlePoint, SecondDropLocation);
//        // פריקה שנייה

    }
}
