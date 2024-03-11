package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static java.lang.Math.abs;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivingSystem;


@TeleOp(name = "TestDrivingSystem", group = "test")
public class TestDrivingSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        drivingSystem.setLocationInTiles(4, 5, 0);

        waitForStart();

        boolean ajust = true, axis = true, control = false, slow = false;

        while(opModeIsActive()) {
            if(gamepad1.cross) ajust = !ajust;
            if(gamepad1.triangle) axis = !axis;
            if(gamepad1.square) control = !control;

            if(gamepad1.circle) slow = !slow;

            if(ajust) telemetry.addLine("Ajusted powers is On");
            if(axis) telemetry.addLine("Axis powers is On");
            if(control) telemetry.addLine("Controlled powers is On");

            Pose2d location = drivingSystem.locationInTiles();
            telemetry.addData("X ", location.getX());
            telemetry.addData("Y ", location.getY());
            telemetry.addData("Heading ", location.getHeading());


            double Px = -gamepad1.left_stick_y,
                    Py = -gamepad1.left_stick_x,
                    Pangle = -gamepad1.right_stick_x;

            if(Py != 0 && Px != 0){
                if(abs(Py / Px) < 0.268) { // 15 degrees
                    Py = 0;
                }
                else if(abs(Px / Py) < 0.1763) { // 10 degrees
                    Px = 0;
                }
            }

            if(slow) { // little movements
                Px /= 3;
                Py /= 3;
                Pangle /= 5;
            }

            Pose2d powers = new Pose2d(Px, Py, Pangle);

            drivingSystem.allDrives(powers, ajust, axis, control);
            telemetry.update();

        }

    }
}