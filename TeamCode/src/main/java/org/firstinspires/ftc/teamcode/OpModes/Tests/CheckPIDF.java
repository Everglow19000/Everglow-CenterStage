package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "PIDF")
public class CheckPIDF extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx FourBar = hardwareMap.get(DcMotorEx.class, "4Bar");
        FourBar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FourBar.setDirection(DcMotorSimple.Direction.REVERSE);
        //final double powerFourBar = 0.3;
        FourBar.setPower(0.3);
        FourBar.setTargetPosition(-10);
        FourBar.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FourBar.setTargetPositionTolerance(10);

        boolean IsPressed = false;
        double PID_coefficients = 0;
        final double deltaPID = 0.1;

        boolean square_toggle = false;
        boolean isFourBarUp = false;
        waitForStart();

        while (opModeIsActive()){

            if(!IsPressed && gamepad1.x){
                PID_coefficients += deltaPID;
                FourBar.setPositionPIDFCoefficients(PID_coefficients);
            }
            IsPressed = gamepad1.x;

            if(gamepad1.square && !square_toggle){
                if(!isFourBarUp){
                    FourBar.setPower(0.3);
                    FourBar.setTargetPosition(270);
                    isFourBarUp = !isFourBarUp;
                } else {
                    FourBar.setPower(0.15);
                    FourBar.setTargetPosition(-10);
                    sleep(500);
                    isFourBarUp = !isFourBarUp;
                }
            }
            square_toggle = gamepad1.square;

            telemetry.addData("PID", PID_coefficients);
            telemetry.update();
        }
    }
}
