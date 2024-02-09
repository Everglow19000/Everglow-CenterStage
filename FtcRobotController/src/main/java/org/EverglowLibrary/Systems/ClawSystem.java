package org.EverglowLibrary.Systems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class ClawSystem{
    private boolean open = false;
    final private double leftClosed = 0.7, leftOpen = 0, rightClosed = 0.4, rightOpen = 1;
    private final LinearOpMode opMode;

    Servo ClawR, ClawL;
    public ClawSystem(LinearOpMode opMode){
        this.opMode = opMode;
        ClawR = opMode.hardwareMap.get(Servo .class, "ClawR");
        ClawL = opMode.hardwareMap.get(Servo.class, "ClawL");
        toggle();
    }

    public void ChangePos(boolean toClose){
        if(toClose){
            ClawR.setPosition(rightOpen); //open
            ClawL.setPosition(leftOpen);
        }
        else{
            ClawR.setPosition(rightClosed); //closed
            ClawL.setPosition(leftClosed);
        }
        open = !open;
    }

    public void toggle() {
        ChangePos(!open);
    }

    public Executor getExecutor(boolean toOpen){
        return new ClawExecutor(toOpen);
    }

    public class ClawExecutor extends Executor{
        private final boolean toClose;

        public ClawExecutor(boolean toClose){
            this.toClose = toClose;
        }

        @Override
        public void run() {
            ChangePos(toClose);
        }

        @Override
        public boolean isFinished() {
            /*
            final double epsilon = 0.15;
            opMode.telemetry.addData("right:", ClawR.getPosition());
            opMode.telemetry.addData("Left:", ClawL.getPosition());
            opMode.telemetry.addData("open:", open);
            opMode.telemetry.update();
            if(open)
                return ClawR.getPosition() + epsilon >= rightOpen && ClawL.getPosition() - epsilon <= leftOpen;
            else
                return ClawR.getPosition() - epsilon <= rightClosed && ClawL.getPosition() + epsilon >= leftClosed;
        }
             */
            opMode.sleep(800);
            return true;
        }

        @Override
        public void stop() {

        }
    }
}