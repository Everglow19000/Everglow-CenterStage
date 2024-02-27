package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servoTest", group = "test")
public class TestServo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "FlipServo");
        boolean zero = false;
        boolean one = false;

        waitForStart();

        while (opModeIsActive()){
            if(gamepad1.cross && !zero){
                servo.setPosition(0);
            }
            zero = gamepad1.cross;

            if(gamepad1.circle && !one){
                servo.setPosition(1);
            }
            one = gamepad1.circle;

        }
    }
}
