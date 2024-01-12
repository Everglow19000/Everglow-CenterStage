package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Utils.PointD;
import com.acmerobotics.roadrunner.geometry.Pose2d;

enum PropPlace{
    LEFT, MIDDLE,RIGHT
}

@TeleOp(name = "AutoMovementRight")
public class AutoMovementOpMode extends LinearOpMode {
    DcMotor leftBack, leftFront, rightBack, rightFront;
    private boolean isStartRight;
    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack =hardwareMap.get(DcMotor.class, "rightBack");
                absoluteAutonomus(true,true, PropPlace.RIGHT, this);
            }

            public static void absoluteAutonomus(boolean isStartRight, boolean isStartBack
                    ,PropPlace propPlace, LinearOpMode linearOpMode)
            {
                //propPlace = 0 :prop in left stripe
                //propPlace = 1 :prop in middle stripe
                //propPlace = 2 :prop in right stripe
                Pose2d firstPoint;
                linearOpMode.waitForStart();

                double squareSize = 60.5; //in cm
                //double distanceOfPropFromRobot = 67; //in cm
                double distanceBetweenTags=17; //in cm
                Pose2d startingPoint = new Pose2d(1,2); //todo: change later
                Pose2d movement;

                Pose2d startPose2d = new Pose2d(0.5* squareSize, 1.5 * squareSize, 0);

                switch (propPlace) {
                    case LEFT:
                        firstPoint = new Pose2d(1.75 * squareSize, 1.5 * squareSize,0.5 * Math.PI);
                        distanceBetweenTags= distanceBetweenTags;
                        break;
                    case MIDDLE:
                        firstPoint = new Pose2d(1.5 * squareSize, 1.5 * squareSize, 0);
                        distanceBetweenTags = 0;
                        break;
                    default:
                        firstPoint = new Pose2d(1.75 * squareSize, 1.5 * squareSize, -0.5 * Math.PI);
                        distanceBetweenTags = -distanceBetweenTags;
                        break;
                }


                Pose2d secondPoint = new Pose2d(3.5 * squareSize, 1.5 * squareSize, 0);


                Pose2d finalPoint = new Pose2d(3.5 * squareSize + distanceBetweenTags, 5 * squareSize,-0.5*Math.PI);



                //todo: function of putting the pixel in the place

                //Todo: moving to the bord to put the thing on it using the roadRunner
                if(linearOpMode.opModeIsActive()){
                    movement = firstPoint.minus(startingPoint);
                    //call move(movement)
                    movement = secondPoint.minus(finalPoint);
                    //call move(movement)
                    movement = finalPoint.minus(secondPoint);
                    //call move(movement)
                }
            }
        }