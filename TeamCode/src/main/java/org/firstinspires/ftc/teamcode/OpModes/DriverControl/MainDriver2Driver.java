package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Systems.FourBarSystem;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive", name = "MainDriver2Driver")
public class MainDriver2Driver extends LinearOpMode {
    FourBarSystem fourBarSystem;
    Servo FlipServo, ClawR, ClawL;
    DcMotorEx SlideL, SlideR, FourBar, GagazMot;
    boolean SlideUp = false;
    boolean ClawClosed = false;
    boolean ClawExtended = false;
    boolean DpadUp_toggle = false;
    boolean DpadDown_toggle = false;
    boolean circle_toggle = false;
    boolean right_bumper_toggle = false;
    boolean square_toggle = false;
    boolean isFourBarRun = false;


    @Override
    public void runOpMode() throws InterruptedException {
        fourBarSystem = new FourBarSystem(this);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // flip servo
        FlipServo = hardwareMap.get(Servo.class, "FlipServo");
        FlipServo.setPosition(0.1691);

        // claw
        ClawR = hardwareMap.get(Servo.class, "ClawR");
        ClawL = hardwareMap.get(Servo.class, "ClawL");
        ClawR.setPosition(1);
        ClawL.setPosition(0);

        // slide
        SlideL = hardwareMap.get(DcMotorEx.class, "SlideL");
        SlideR = hardwareMap.get(DcMotorEx.class, "SlideR");
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setDirection( DcMotorSimple.Direction.REVERSE);
        SlideR.setTargetPosition(0);
        SlideL.setTargetPosition(0);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // four bar
        /*
        FourBar = hardwareMap.get(DcMotorEx.class, "4Bar");
        FourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FourBar.setDirection( DcMotorSimple.Direction.REVERSE);
        //final double powerFourBar = 0.3;
        FourBar.setPower(0.3);
        FourBar.setTargetPosition(-10);
        FourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FourBar.setTargetPositionTolerance(10);
         */

        // galal azikonim
        GagazMot = hardwareMap.get(DcMotorEx.class, "GagazMot");

        waitForStart();
        double power = 0.4;
        while (!isStopRequested()) {
            if(isFourBarRun){
                power = Math.max(power-0.2, 0.1);
                fourBarSystem.setMotorPower(power);
            }
            if(gamepad2.circle && !circle_toggle){
                if(!SlideUp){ //get elevator up
                    SlideR.setPower(0.6);
                    SlideL.setPower(0.6);
                    SlideR.setTargetPosition(720);
                    SlideL.setTargetPosition(720);
                    SlideUp = !SlideUp;
                } else { //get elevator down
                    SlideR.setPower(0.4);
                    SlideL.setPower(0.4);
                    SlideR.setTargetPosition(35);
                    SlideL.setTargetPosition(35);
                    SlideUp = !SlideUp;
                }
            }
            circle_toggle = gamepad1.circle; //control when the elevator will work,
            // lock circle until the movement has done

            if(gamepad2.right_bumper && !right_bumper_toggle){
                if(ClawClosed){
                    ClawL.setPosition(0.7);
                    ClawR.setPosition(0.4);
                    ClawClosed = !ClawClosed;
                } else {
                    ClawL.setPosition(0);
                    ClawR.setPosition(1);
                    ClawClosed = !ClawClosed;
                }
            }
            right_bumper_toggle = gamepad1.right_bumper;

            if(gamepad1.square && !square_toggle){ //
                if(ClawExtended){
                    power = 0.3;
                    fourBarSystem.setMotorPower(power);
                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.PICKUP);
                    fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.PICKUP);
                    ClawExtended = !ClawExtended;
                    isFourBarRun = true;
                } else {
                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.DROP);
                    fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.DROP);
                    ClawExtended = !ClawExtended;
                    isFourBarRun = false;
                }
            }
            square_toggle = gamepad1.square;

            if(!DpadUp_toggle && (gamepad1.dpad_up || gamepad2.dpad_up)) { //galgal azikonim
                if (GagazMot.getPower() == 0) {
                    GagazMot.setPower(-1);
                } else {
                    GagazMot.setPower(0);
                }
            }
            DpadUp_toggle = (gamepad1.dpad_up || gamepad2.dpad_up);

            if(!DpadDown_toggle && (gamepad1.dpad_down || gamepad2.dpad_down)) { //galgal azikonim, revers
                if (GagazMot.getPower() == 0) {
                    GagazMot.setPower(1);
                }else {
                    GagazMot.setPower(0);
                }
            }
            DpadDown_toggle = (gamepad1.dpad_down || gamepad2.dpad_down);

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );

            drive.update();

            //Pose2d poseEstimate = drive.getPoseEstimate();
            //telemetry.addData("x", gamepad1.a);
            //telemetry.update();
        }
    }
}

