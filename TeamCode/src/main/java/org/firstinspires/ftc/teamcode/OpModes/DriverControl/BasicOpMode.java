package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Systems.ClawSystem;
import org.firstinspires.ftc.teamcode.Systems.DrivingSystem;
import org.firstinspires.ftc.teamcode.Systems.ElevatorSystem;
import org.firstinspires.ftc.teamcode.Systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.Systems.GWheelSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "BasicOpMode")
public class BasicOpMode extends LinearOpMode {

    SampleMecanumDrive drive;
    ElevatorSystem elevatorSystem;
    ClawSystem clawSystem;
    FourBarSystem fourBarSystem;
    GWheelSystem gWheelSystem;


    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorSystem = new ElevatorSystem(this);
        clawSystem = new ClawSystem(this);
        fourBarSystem = new FourBarSystem(this);
        gWheelSystem = new GWheelSystem(this);

        waitForStart();
        while (opModeIsActive()) {


            if(gamepad1.right_trigger > 0.01) { gWheelSystem.toggle(true); }
            else if(gamepad1.left_trigger > 0.01) { gWheelSystem.toggle(false); }

            if(gamepad1.right_bumper){ elevatorSystem.toggle(); }

            if(gamepad1.left_bumper) { clawSystem.toggle(); }

            if(gamepad1.dpad_down) { fourBarSystem.set4BarPosition(FourBarSystem.Level.START); }
            if(gamepad1.dpad_left) { fourBarSystem.set4BarPosition(FourBarSystem.Level.PICKUP); }
            if(gamepad1.dpad_up) { fourBarSystem.set4BarPosition(FourBarSystem.Level.DROP); }
            if(gamepad1.dpad_down) { fourBarSystem.set4BarPosition(FourBarSystem.Level.REST); }


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
        }


    }
}