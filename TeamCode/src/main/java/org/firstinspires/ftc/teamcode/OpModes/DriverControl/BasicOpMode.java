package org.firstinspires.ftc.teamcode.OpModes.DriverControl;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Systems.ClawSystem;
//import org.firstinspires.ftc.teamcode.Systems.DrivingSystem;
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

    boolean isPressed = false;
    DcMotorEx SlideL, SlideR;
    boolean circle_toggle = false;
    boolean SlideUp = false;
    boolean FourBarUp = false;

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        elevatorSystem = new ElevatorSystem(this);
        clawSystem = new ClawSystem(this);
        fourBarSystem = new FourBarSystem(this);
        gWheelSystem = new GWheelSystem(this);


        SlideL = hardwareMap.get(DcMotorEx.class, "SlideL");
        SlideR = hardwareMap.get(DcMotorEx.class, "SlideR");
        SlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlideL.setDirection( DcMotorSimple.Direction.REVERSE);
        SlideR.setTargetPosition(0);
        SlideL.setTargetPosition(0);
        SlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        while (opModeIsActive()) {


            if(gamepad1.dpad_up) { gWheelSystem.toggle(true); }
            else if(gamepad1.dpad_down) { gWheelSystem.toggle(false); }

            if(gamepad1.circle && !circle_toggle){
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

            if(gamepad1.right_bumper) { clawSystem.toggle(); }

            //if(gamepad1.dpad_down) { fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.START); }
            if(gamepad1.square && !isPressed) {
                if(FourBarUp) {
                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.PICKUP);
                    fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.PICKUP);
                    FourBarUp = false;
                }
                else {
                    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.DROP);
                    fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.DROP);
                    FourBarUp = true;
                }
            }

            isPressed = gamepad1.square;

            //if(gamepad1.dpad_left) {
            //    fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.DROP);
            //    fourBarSystem.setServoPosition(FourBarSystem.ServoAngel.DROP);
            //}
            //if(gamepad1.dpad_right) { fourBarSystem.set4BarPositionByLevel(FourBarSystem.Level.REST); }


            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
            fourBarSystem.updateP();
        }


    }
}