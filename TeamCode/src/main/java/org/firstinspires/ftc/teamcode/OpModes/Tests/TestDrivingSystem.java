package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.DrivingSystem;


@TeleOp(name = "TestDrivingSystem", group = "test")
public class TestDrivingSystem extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DrivingSystem drivingSystem = new DrivingSystem(this);
        drivingSystem.setLocationInTiles(4, 4, 0);

        waitForStart();

        boolean ajust = true, axis = true, control = false;

        while(opModeIsActive()) {
            if(gamepad1.circle) ajust = !ajust;
            if(gamepad1.triangle) axis = !axis;
            if(gamepad1.square) control = !control;

            if(ajust) telemetry.addLine("Ajusted powers is On");
            if(axis) telemetry.addLine("Axis powers is On");
            if(control) telemetry.addLine("Controlled powers is On");

            Pose2d location = drivingSystem.getPoseEstimate();
            telemetry.addData("X ", location.getX());
            telemetry.addData("Y ", location.getY());
            telemetry.addData("Heading ", location.getHeading());

            double Px = -gamepad1.left_stick_y,
                    Py = -gamepad1.left_stick_x,
                    Pangle = -gamepad1.right_stick_x;
            if(gamepad1.left_stick_button) {
                Px /= 3;
                Py /= 3;
            }
            if(gamepad1.right_stick_button) {
                Pangle /= 5;
            }
            Pose2d powers = new Pose2d(Px, Py, Pangle);

            drivingSystem.allDrives(powers, ajust, axis, control);
            telemetry.update();

        }

    }
}