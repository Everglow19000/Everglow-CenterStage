package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.EverglowLibrary.Systems.ElevatorSystem;


@TeleOp(name = "TestEel", group = "test")
public class TestEel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        //ElevatorSystem elevators = new ElevatorSystem(this);
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "SlideL");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "SlideR");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection( DcMotorSimple.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();
        boolean toggle = false;
        boolean toggle2 = false;
        double p = 0;

        while (opModeIsActive()) {
            if(!toggle && gamepad1.circle){
                p += 0.01;
            }
            toggle = gamepad1.circle;

            if(!toggle2 && gamepad1.cross){
                p -= 0.01;
            }
            toggle2 = gamepad1.cross;

            if(gamepad1.triangle){
                p = 0;
            }

            if(gamepad1.dpad_down) {
                left.setPower(p);
                right.setPower(p);
            }
            else
            {
                left.setPower(0);
                right.setPower(0);
            }
            telemetry.addData("power", p);
            telemetry.update();
        }

    }


}