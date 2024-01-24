package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

//more imports

@TeleOp(name = "MoveDaServo")
public class MoveDaServo extends LinearOpMode {

    //make more attributes & functions
    Servo DaServo;
    BNO055IMU imu;
    @Override
    public void runOpMode(){
        DaServo = hardwareMap.get(Servo.class, "Servo");

        waitForStart();
        DaServo.setPosition(0);
        while (opModeIsActive()) {
            DaServo.setPosition(DaServo.getPosition()+ gamepad1.right_stick_x);

            telemetry.addData("Servo Position", DaServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
        while (!isStopRequested()) {}
    }
}