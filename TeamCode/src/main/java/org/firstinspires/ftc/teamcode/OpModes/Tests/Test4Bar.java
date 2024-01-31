package org.firstinspires.ftc.teamcode.OpModes.Tests;



import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Systems.FourBarSystem;

@TeleOp(name = "Test4Bar")
public class Test4Bar extends LinearOpMode {
    FourBarSystem fourBarSystem;
    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        waitForStart();
        double position = 0;
        while(opModeIsActive()) {
            fourBarSystem.setMotorPower(gamepad1.left_stick_y * 0.2);
            position += gamepad1.right_stick_y / 1000;
            //position = min(1, position);
            //position = max(0, position);
            fourBarSystem.setServoPosition(position);

            telemetry.addData("MotorPosition", fourBarSystem.getCurrentMotorPosition());
            telemetry.addData("ServoPosition", fourBarSystem.getCurrentServoPosition());
            telemetry.update();

        }
        fourBarSystem.setMotorPower(gamepad1.left_stick_y * 0.2);
    }
}