package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name = "TestEel")
public class TestEel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Elevators elevators = new Elevators(this);
        waitForStart();
        boolean is_sync = false;
        double buff = 0.5;
        while (opModeIsActive()) {
            if(!is_sync)
                elevators.setPower(gamepad1.left_stick_y*buff, gamepad1.right_stick_y*buff);
            else
                elevators.setPower(gamepad1.left_stick_y*buff, gamepad1.left_stick_y*buff);

            if(gamepad1.cross)
                is_sync = !is_sync;
        }

        elevators.setPower(0);

    }


}