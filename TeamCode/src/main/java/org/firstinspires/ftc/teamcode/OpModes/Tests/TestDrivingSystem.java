package org.firstinspires.ftc.teamcode.OpModes.Tests;

import static org.apache.commons.math3.dfp.DfpMath.pow;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DrivingSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "TestDrivingSystem", group = "test")
public class TestDrivingSystem extends LinearOpMode {

    public double linearInputToExponential(double power){
        double base = 6;
        return (Math.pow(base, Math.abs(power)) - 1) / (base - 1) * Math.signum(power);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        //SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        //drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        DrivingSystem drivingSystem = new DrivingSystem(this);
        drivingSystem.setLocationInTiles(4, 4, 0);

        waitForStart();

        boolean ajust = true, axis = true, control = false, slow = false;
        Pose2d location;
        Pose2d powers;
        while(opModeIsActive()) {
            if(gamepad1.circle) ajust = !ajust;
            if(gamepad1.triangle) axis = !axis;
            if(gamepad1.square) control = !control;
            if(gamepad1.cross) slow = !slow;

            if(ajust) telemetry.addLine("Ajusted powers is On");
            if(axis) telemetry.addLine("Axis powers is On");
            if(control) telemetry.addLine("Controlled powers is On");

            location = drivingSystem.getPoseEstimate();
            telemetry.addData("X ", location.getX());
            telemetry.addData("Y ", location.getY());
            telemetry.addData("Heading ", location.getHeading());

            drivingSystem.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                     -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x));

            double Px = -gamepad1.left_stick_y,
                    Py = -gamepad1.left_stick_x,
                    Pangle = -gamepad1.right_stick_x;

            Px = linearInputToExponential(Px);
            Py = linearInputToExponential(Py);
            Pangle = linearInputToExponential(Pangle);

            if(slow) {
                Px /= 3;
                Py /= 3;
                Pangle /= 5;
            }

            powers = new Pose2d(Px, Py, Pangle);

            drivingSystem.allDrives(powers, ajust, axis, control);
            telemetry.update();
        }

    }
}