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
        double positionServo = fourBarSystem.getCurrentServoPosition()
                , positionMotor = fourBarSystem.getCurrentMotorPosition();
        fourBarSystem.setMotorPower(0.8);

        while(opModeIsActive()) {
            positionServo += gamepad1.right_stick_y / 1000;
            positionMotor += gamepad1.left_stick_y / 20;
            fourBarSystem.set4BarPosition((int)positionMotor);
            fourBarSystem.setServoPosition(positionServo);

            if(gamepad1.triangle){
                positionMotor = FourBarSystem.Level.DROP.state;

            }

            if(gamepad1.circle){
                positionMotor = FourBarSystem.Level.PICKUP.state;

            }


            telemetry.addData("motor:", positionMotor);
            telemetry.addData("servo:", positionServo);
            telemetry.update();
            //fourBarSystem.updateP(0.35);
        }
    }
}