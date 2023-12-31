package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Motors Test")
public class TestMotors extends LinearOpMode {

    //make more attributes & functions
    @Override
    public void runOpMode(){
        DcMotor motor = hardwareMap.get(DcMotor.class, "back_left");
        boolean isRun = true;
        waitForStart();
        while (opModeIsActive()){
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if(gamepad1.x)
                isRun = !isRun;
            if(gamepad1.left_stick_y == 0 && isRun) {
                motor.setPower(0);
            }
            else{
                motor.setPower(gamepad1.left_stick_y/1.5);
            }
        }
    }
}