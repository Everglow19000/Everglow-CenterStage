package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "SimplerOpMode")
public class SimplerOpMode extends LinearOpMode {

    static double squareSize = 60.5; //in cm
    static double distanceOfPropFromRobot = 67; //in cm
    static double distanceBetweenTags=15.0; //in cm
    static double distanceBuffer=0;
    public enum PropPlace {
        LEFT(new Vector2d(0, 0)), MIDDLE(new Vector2d(0, 0)), RIGHT(new Vector2d(0, 0))
        public final Vector2d propPlaceVector;

        PropPlace(Vector2d propPlace) {
            this.propPlaceVector = propPlace;
        }
        }

    public enum StartPosition{
        FRONTLEFT(new Vector2d(0, 0)), FRONTRIGHT(new Vector2d(0, 0)), BACKLEFT(new Vector2d(0, 0)), BACKRIGHT(new Vector2d(0, 0))

        public final Vector2d startPositionVector;
        StartPosition(Vector2d startPosition) {
            this.startPositionVector = startPosition;
        }

        public boolean isLeft(AutonomousFirst.StartPosition startPosition) {
            if(startPosition == FRONTLEFT || startPosition == BACKLEFT)
                return true;
            return false;
        }

        public boolean isFront(AutonomousFirst.StartPosition startPosition) {
            if(startPosition == FRONTLEFT || startPosition == FRONTRIGHT)
                return true;
            return false;
        }
    }



    public void runOpMode(AutonomousFirst.StartPosition startPosition) {
        PropPlace propPlace; // camera

        Pose startLocation;
        final double RightLainMiddleX = 1.5 * squareSize;
        final double RightLainFirstDropX = 1.75 * squareSize;
        final double RightStartPosition = 0.5 *squareSize;

        final double LeftLainMiddleX = 4.5 * squareSize;
        final double LeftLainFirstDropX = 4.25 * squareSize;
        final double LeftStartPosition = 5.5 *squareSize;

        final double BackLainMiddleY = 1.5 * squareSize;

        final double trustY = 2.5 * squareSize;

        final double FrontLainMiddleY = 3.5 * squareSize;

        final double FinalDropY = 5 * squareSize;

        final double North = 0;
        final double East = -PI/2;
        final double West = PI/2;
        final double South = PI;



        Pose LeftLainSeconedDropX = new Pose(LeftLainMiddle, 5 * squareSize, 0);
        if(propPlace == PropPlace.LEFT) LeftLainSeconedDrop.x += distanceBetweenTags;
        else if(propPlace == PropPlace.RIGHT) LeftLainSeconedDrop.x -= distanceBetweenTags;

        final double forwardDeviationDistance = 0.9 * squareSize;

        Pose FirstDropMiddlePoint, FirstDropLocation, SecondDropMiddlePoint, SecondDropLocation;


        switch (startPosition) {
            case BACKLEFT:
                startLocation = new Pose(LeftStartPosition, BackLainMiddleY, East);
                FirstDropMiddlePoint = new Pose(LeftLainMiddleX, BackLainMiddleY, East);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose(LeftLainFirstDropX, trustY, South);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose(LeftLainFirstDropX, BackLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose(LeftLainMiddleX, BackLainMiddleY, East);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, trustY, East);
                        SecondDropLocation = new Pose(LeftLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case FRONTLEFT:
                startLocation = new Pose(LeftStartPosition, FrontLainMiddleY, East);
                FirstDropMiddlePoint = new Pose(LeftLainMiddleX, FrontLainMiddleY, East);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose(LeftLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose(LeftLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose(LeftLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose(LeftLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose(LeftLainMiddleX, FrontLainMiddleY, East);
                        SecondDropMiddlePoint = new Pose(LeftLainMiddleX, FrontLainMiddleY, East);
                        SecondDropLocation = new Pose(LeftLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case BACKRIGHT:
                startLocation = new Pose(RightStartPosition, BackLainMiddleY, West);
                FirstDropMiddlePoint = new Pose(RightLainMiddleX, BackLainMiddleY, West);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose(RightLainFirstDropX, trustY, South);
                        SecondDropMiddlePoint = new Pose(1*squareSize, trustY, South);
                        SecondDropLocation = new Pose(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose(RightLainFirstDropX, BackLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(RightLainMiddleX, trustY, South);
                        SecondDropLocation = new Pose(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose(RightLainMiddleX, BackLainMiddleY, West);
                        SecondDropMiddlePoint = new Pose(RightLainMiddleX, trustY, West);
                        SecondDropLocation = new Pose(RightLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
            case FRONTRIGHT:
                startLocation = new Pose(RightStartPosition, FrontLainMiddleY, West);
                FirstDropMiddlePoint = new Pose(RightLainMiddleX, FrontLainMiddleY, West);
                switch (propPlace) {
                    case LEFT:
                        FirstDropLocation= new Pose(RightLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(1*squareSize, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose(RightLainMiddleX+distanceBetweenTags, FinalDropY, North);
                        break;
                    case RIGHT:
                        FirstDropLocation= new Pose(RightLainFirstDropX, FrontLainMiddleY, South);
                        SecondDropMiddlePoint = new Pose(RightLainMiddleX, FrontLainMiddleY, South);
                        SecondDropLocation = new Pose(RightLainMiddleX-distanceBetweenTags, FinalDropY, North);
                        break;
                    default:
                        FirstDropLocation= new Pose(RightLainMiddleX, FrontLainMiddleY, West);
                        SecondDropMiddlePoint = new Pose(RightLainMiddleX, FrontLainMiddleY, West);
                        SecondDropLocation = new Pose(RightLainMiddleX, FinalDropY, North);
                        break;
                }
                break;
        }




        drive(startLocation, FirstDropMiddlePoint, FirstDropLocation);
        // להפעיל גג"ז
        drive(FirstDropLocation, SecondDropMiddlePoint, SecondDropLocation);
        // פריקה שנייה

    }
}