package org.firstinspires.ftc.teamcode.OpModes.Tests;



import static java.lang.Math.max;
import static java.lang.Math.min;

import android.util.Pair;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.FourBarSystem;

@TeleOp(name = "Test4Bar", group = "test")
public class Test4Bar extends LinearOpMode {
    FourBarSystem fourBarSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        waitForStart();
        int positionServo = 0, positionMotor = FourBarSystem.Level.PICKUP.state;
        double power4 = 0;
        while(opModeIsActive()) {
            positionMotor = fourBarSystem.getCurrentMotorPosition();

            if(gamepad1.dpad_up){
                fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.DROP);
                sleep(1000);
                fourBarSystem.set4BarPosition(FourBarSystem.Level.DROP.state);
            }
            if(gamepad1.cross){
                fourBarSystem.setMotorPower(0.1);
            }
            else if(gamepad1.triangle){
                fourBarSystem.setMotorPower(-0.1);
            }
            else
                fourBarSystem.set4BarPosition(positionMotor);
        }
    }
}