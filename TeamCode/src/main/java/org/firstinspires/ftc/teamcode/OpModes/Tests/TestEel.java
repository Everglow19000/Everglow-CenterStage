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
        double buff = 0.2;
        double left;
        double right;
        while (opModeIsActive()) {
            left = gamepad1.left_stick_y*buff;
            right = gamepad1.right_stick_y*buff;

            if(!is_sync)
                elevators.setPower(left, right);
            else
                elevators.setPower(left);

            if(gamepad1.cross)
                is_sync = true;
            if(gamepad1.circle)
                is_sync = false;
            if(gamepad1.square)
                elevators.goTo(Elevators.Level.UP);
        }

        elevators.setPower(0);

    }


}