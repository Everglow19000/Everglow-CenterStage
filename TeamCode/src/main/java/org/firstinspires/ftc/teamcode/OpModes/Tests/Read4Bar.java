package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Read4Bar", group = "test")
public class Read4Bar extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        DcMotorEx fourBarMotor;
        Servo clawAngelServo;
        fourBarMotor = this.hardwareMap.get(DcMotorEx.class, "4Bar");
        clawAngelServo = this.hardwareMap.get(Servo.class, "FlipServo");
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.dpad_up)
            {

            }
            telemetry.addData("fourBarMotor", fourBarMotor.getCurrentPosition());
            telemetry.addData("clawAngelServo", clawAngelServo.getPosition());
            telemetry.update();
        }

    }

}
