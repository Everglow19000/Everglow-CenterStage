package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.EverglowLibrary.Utils.PointD;
import org.EverglowLibrary.Utils.Pose;

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
        absoluteAutonomus(true,true, this);
    }

    public static void absoluteAutonomus(boolean isStartRight, boolean isStartBack, LinearOpMode linearOpMode)
    {

        linearOpMode.waitForStart();

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

        //todo: function of puting the pixel in the place

        //Todo: moving to the bord to put the thing on it using the roadRunner
        if(linearOpMode.opModeIsActive()){
            //move to the board
        }
    }

    public static void move(PointD place){

    }

    public static double getDistance1(){ //מרחק מחיישן מספר אחד
        return 0;
    }

    public static double getDistance2(){ //מרחק מחיישן מספר אחד
        return 0;
    }


}
