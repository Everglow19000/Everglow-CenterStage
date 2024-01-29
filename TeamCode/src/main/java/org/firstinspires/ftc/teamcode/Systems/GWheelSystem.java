package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class GWheelSystem {
    private DcMotorEx motor;
    double currentPower = 0;
    OpMode opMode;
    public GWheelSystem(OpMode opMode){
        this.opMode = opMode;
        motor = opMode.hardwareMap.get(DcMotorEx .class, "GagazMot");
    }

    public void setPower(double Power) {
        motor.setPower(Power);
    }

    public void toggle(boolean in) {
        if(currentPower != 0) currentPower = 0;
        else if(in) currentPower = -1;
        else currentPower = 1;

        setPower(currentPower);
    }

}