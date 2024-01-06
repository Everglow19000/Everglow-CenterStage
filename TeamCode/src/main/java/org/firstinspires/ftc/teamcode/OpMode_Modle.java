package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//more imports

@TeleOp(name = "Modle Op-Mode")
public class OpMode_Modle extends LinearOpMode {

    //make more attributes & functions
    final double TicksToCm = 0.0586;
    DcMotor FR,FL,BR,BL;BNO055IMU imu;
    @Override
    public void runOpMode(){
        TestMotor(50, FR);
        sleep(2000);
        TestMotor(50, FL);
        sleep(2000);
        TestMotor(50, BR);
        sleep(2000);
        TestMotor(50, BL);
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
