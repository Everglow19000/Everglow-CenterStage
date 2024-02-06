package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSystem implements ExecutorableSystem{
    private boolean open = false;
    final private double leftOpen = 0.7, leftClosed = 0, rightOpen = 0.4, rightClosed = 1;
    private final OpMode opMode;

    Servo ClawR, ClawL;
    public ClawSystem(OpMode opMode){
        this.opMode = opMode;
        ClawR = opMode.hardwareMap.get(Servo .class, "ClawR");
        ClawL = opMode.hardwareMap.get(Servo.class, "ClawL");
    }

    public void toggle() {
        if (open) {
            ClawR.setPosition(rightOpen);
            ClawL.setPosition(leftOpen);
        } else {
            ClawR.setPosition(rightClosed);
            ClawL.setPosition(leftClosed);
        }
        open = !open;
    }

    @Override
    public Executor getExecutor() {
        return new ClayExecutor();
    }

    public class ClayExecutor extends Executor{

        @Override
        public void run() {
            toggle();
        }

        @Override
        public boolean isFinished() {
            final double epsilon = 0.05;
            if(open)
                return ClawR.getPosition() - epsilon <= rightOpen && ClawL.getPosition() + epsilon >= leftOpen;
            else
                return ClawR.getPosition() + epsilon >= rightClosed && ClawL.getPosition() - epsilon <= leftClosed;
        }
    }
}