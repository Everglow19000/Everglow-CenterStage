package org.firstinspires.ftc.teamcode.OpModes.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.EverglowLibrary.Systems.ElevatorSystem;


@TeleOp(name = "TestEel", group = "test")
public class TestEel extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ElevatorSystem elevators = new ElevatorSystem(this);
        waitForStart();
        double buff = 0.2;
        double left;
        while (opModeIsActive()) {
            left = gamepad1.left_stick_y*buff;

            elevators.setPower(left);

            elevators.printPos();
        }

        elevators.setPower(0);

    }


}