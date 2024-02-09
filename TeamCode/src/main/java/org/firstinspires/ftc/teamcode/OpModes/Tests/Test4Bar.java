package org.firstinspires.ftc.teamcode.OpModes.Tests;



import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.FourBarSystem;
import org.EverglowLibrary.ThreadHandleLib.Sequence;

import java.nio.channels.AsynchronousCloseException;

@TeleOp(name = "Test4Bar", group = "test")
public class Test4Bar extends LinearOpMode {
    FourBarSystem fourBarSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        //ElevatorSystem elevatorSystem = new ElevatorSystem(this);
        //ClawSystem clawSystem = new ClawSystem(this);
        /*Sequence sequence = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

         */
        waitForStart();
        double positionServo = FourBarSystem.ServoAngel.PICKUP.state
                , positionMotor = FourBarSystem.Level.REST.state;
        double power4 = 0;


        boolean arm1 = false;
        boolean arm2 = false;
        boolean servo1 = false;
        boolean servo2 = false;

        while(opModeIsActive()) {
            if(!servo1 && gamepad1.circle){
                positionServo += 0.01;
            }
            servo1 = gamepad1.circle;

            if(!servo2 && gamepad1.cross){
                positionServo -= 0.01;
            }
            servo2 = gamepad1.cross;

            if(!arm1 && gamepad1.square){
                positionMotor += 1;
            }
            arm1 = gamepad1.square;
            if(!arm2 && gamepad1.triangle){
                positionMotor -= 1;
            }
            arm2 = gamepad1.triangle;

            if(gamepad1.dpad_down){
                fourBarSystem.setServoPosition(positionServo);
                fourBarSystem.set4BarPosition((int)positionMotor);
            }

            telemetry.addData("motor:", positionMotor);
            telemetry.addData("servo:", positionServo);
            telemetry.update();
        }
    }
}