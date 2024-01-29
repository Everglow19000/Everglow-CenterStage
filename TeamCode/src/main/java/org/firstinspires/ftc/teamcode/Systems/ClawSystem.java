package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class ClawSystem {
    private boolean open = false;
    final private double leftOpen = 0.7, leftClosed = 0, rightOpen = 0.4, rightClosed = 1;
    private OpMode opMode;

    Servo ClawR, ClawL;
    public ClawSystem(OpMode opMode){
        this.opMode = opMode;
        ClawR = opMode.hardwareMap.get(Servo .class, "ClawR");
        ClawL = opMode.hardwareMap.get(Servo.class, "ClawL");
    }

    public void toggle() {
        if(open) {
            ClawR.setPosition(rightOpen);
            ClawL.setPosition(leftOpen);
        }
        else {
            ClawR.setPosition(rightClosed);
            ClawL.setPosition(leftClosed);
        }
        open = !open;
    }
}