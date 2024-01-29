package org.firstinspires.ftc.teamcode.OpModes.Tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

//more imports

@TeleOp(name = "Modle Op-Mode")
public class OpMode_Modle extends LinearOpMode {

    //make more attributes & functions
    final double TicksToCm = 0.0586;
    Encoder LE,RE,FE;
    BNO055IMU imu;
    @Override
    public void runOpMode(){
        LE = new Encoder(hardwareMap.get(DcMotorEx.class, "leftRear"));
        RE = new Encoder(hardwareMap.get(DcMotorEx.class, "GagazMot"));
        FE = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        RE.setDirection(Encoder.Direction.REVERSE);

        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("LE", LE.getCurrentPosition());
            telemetry.addData("RE", RE.getCurrentPosition());
            telemetry.addData("FE", FE.getCurrentPosition());
            telemetry.update();
        }
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
