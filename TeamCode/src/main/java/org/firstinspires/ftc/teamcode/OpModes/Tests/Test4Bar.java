package org.firstinspires.ftc.teamcode.OpModes.Tests;



import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.FourBarSystem;

@TeleOp(name = "Test4Bar")
public class Test4Bar extends LinearOpMode {
    FourBarSystem fourBarSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        waitForStart();
        double positionServo = 0, positionMotor = FourBarSystem.Level.PICKUP.state;
        double power4 = 0;
        while(opModeIsActive()) {
            fourBarSystem.set4BarPosition((int)positionMotor);
            positionServo += gamepad1.right_stick_y / 1000;
            positionMotor += gamepad1.left_stick_y / 100;
            //position = min(1, position);
            //position = max(0, position);
            fourBarSystem.setServoPosition(positionServo);

            //if(gamepad1.a) {fourBarSystem.toggle4Bar();}

            //power4 += gamepad1.left_stick_y / 100;

            //fourBarSystem.setMotorPower(power4);
            fourBarSystem.setServoPosition(positionServo);

            telemetry.addData("MotorPosition", fourBarSystem.getCurrentMotorPosition());
            //telemetry.addData("MotorPower", power4);
            //fourBarSystem.updateP();
            telemetry.update();

        }
        fourBarSystem.setMotorPower(gamepad1.left_stick_y * 0.2);
    }
}