package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Utils.PointD;
import org.EverglowLibrary.Utils.Pose;

enum PropPlace{
    LEFT, MIDDLE,RIGHT
}

@TeleOp(name = "AutoMovementRight")
public class AutoMovementOpMode extends LinearOpMode {
    DcMotor leftBack, leftFront, rightBack, rightFront;
    private boolean isStartRight;
    @Override
    public void runOpMode() throws InterruptedException {
        //leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        //leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        //rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        absoluteAutonomus(true,true, PropPlace.RIGHT, this);
    }

    public static void absoluteAutonomus(boolean isStartRight, boolean isStartBack
            ,PropPlace propPlace, LinearOpMode linearOpMode)
    {
        //propPlace = 0 :prop in left stripe
        //propPlace = 1 :prop in middle stripe
        //propPlace = 2 :prop in right stripe
        Pose firstPoint;
        linearOpMode.waitForStart();

        double squareSize = 60.5; //in cm
        double distanceOfPropFromRobot = 67; //in cm
        double distanceBetweenTags=17; //in cm
        double distanceBuffer=0;
        Pose movement;

        /*
        PointD right = new PointD(30,30)
                , left = new PointD(30,30)
                , middle =  new PointD(30,30);
        double objectDis = 30;

        if(getDistance1() < objectDis){
            move(right.add(new PointD(10,0)));
        }
        else if(getDistance2() < objectDis){
            move(left.add(new PointD(-10,0)));
        }
        else{
            move(middle.add(new PointD(0,-10)));
        }
        */
        Pose startPose = new Pose(0.5* squareSize, 1.5 * squareSize, 0);

        switch (propPlace) {
            case LEFT:
                firstPoint = new Pose(1.75 * squareSize, 1.5 * squareSize, 0.5 * Math.PI);
                distanceBuffer=distanceBetweenTags;
                break;
            case MIDDLE:
                firstPoint = new Pose(1.5 * squareSize, 1.5 * squareSize, 0);
                distanceBuffer=0;
                break;
            default:
                firstPoint = new Pose(1.75 * squareSize, 1.5 * squareSize, -0.5 * Math.PI);
                distanceBuffer=-distanceBetweenTags;
                break;
        }


        Pose secondPoint = new Pose(3.5 * squareSize, 1.5 * squareSize, 0);


        Pose finalPoint = new Pose(3.5 * squareSize + distanceBuffer, 5 * squareSize,-0.5*Math.PI);



        //todo: function of putting the pixel in the place

        //Todo: moving to the bord to put the thing on it using the roadRunner
        if(linearOpMode.opModeIsActive()){
            movement = firstPoint.subtract(startPose);
            //call move(movement)
            movement = secondPoint.subtract(finalPoint);
            //call move(movement)
            movement = finalPoint.subtract(secondPoint);
            //call move(movement)
        }
    }

    public static void move(Pose place){

    }

    public static double getDistance1(){ //מרחק מחיישן מספר אחד
        return 0;
    }

    public static double getDistance2(){ //מרחק מחיישן מספר אחד
        return 0;
    }


}
