package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSystem {

    public enum Level {
        START(0), PICKUP(-10), DROP(270), REST(150);

        public final int state;

        Level(int state) {
            this.state = state;
        }
    }

    public enum ServoAngel {
        START(0), PICKUP(0), DROP(0.5), Pass(0);

        public final double state;

        ServoAngel(double state) {
            this.state = state;
        }
    }

    Level currentLevel = Level.START;
    ServoAngel currentServoAngel = ServoAngel.START;

    public DcMotorEx fourBarMotor;
    OpMode opMode;
    public Servo clawAngelServo;

    public FourBarSystem(OpMode opMode){
        this.opMode = opMode;
        fourBarMotor = opMode.hardwareMap.get(DcMotorEx .class, "4Bar");
        clawAngelServo = opMode.hardwareMap.get(Servo.class, "FlipServo");
    }

    public void set4BarPosition(Level targetLevel) {
        currentLevel = targetLevel;
        fourBarMotor.setTargetPosition(currentLevel.state);
    }

    public void setServoPosition(ServoAngel targetServoAngel) {
        currentServoAngel = targetServoAngel;
        clawAngelServo.setPosition(currentServoAngel.state);
    }

    public void toggle4Bar() {
        if(currentLevel == Level.START || currentLevel == Level.PICKUP || currentLevel == Level.REST) {
            setServoPosition(ServoAngel.DROP);
        }
        else{
            setServoPosition(ServoAngel.PICKUP);
        }
    }

    public void toggleAngleServo() {
        if(currentLevel == Level.START || currentLevel == Level.PICKUP) {
            setServoPosition(ServoAngel.DROP);
        }
        else{
            setServoPosition(ServoAngel.PICKUP);
        }
    }


}