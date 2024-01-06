package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//more imports

@TeleOp(name = "Modle Op-Mode")
public class OpMode_Modle extends LinearOpMode {

    //make more attributes & functions
    final double TicksToCm = 0.0586;
    DcMotor FR,FL,BR,BL;
    BNO055IMU imu;
    @Override
    public void runOpMode(){
        FL = hardwareMap.get(DcMotorEx.class, "leftFront");
        BL = hardwareMap.get(DcMotorEx.class, "leftRear");
        BR = hardwareMap.get(DcMotorEx.class, "rightRear");
        FR = hardwareMap.get(DcMotorEx.class, "rightFront");

        waitForStart();

        FR.setPower(0.4);
        sleep(1000);
        FL.setPower(0.4);
        sleep(1000);
        BR.setPower(0.4);
        sleep(1000);
        BL.setPower(0.4);
        sleep(1000);
    }
    void TestMotor(double dis, DcMotor motor){
        double speed = 0.5;
        double power;
        while (motor.getCurrentPosition()*TicksToCm < dis ) {
            power = ((dis - motor.getCurrentPosition()*TicksToCm)/dis)*speed;
            motor.setPower(power);
        }
    }
}
