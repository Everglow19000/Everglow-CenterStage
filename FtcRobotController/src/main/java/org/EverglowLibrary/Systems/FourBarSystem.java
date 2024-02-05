package org.EverglowLibrary.Systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSystem {

    public enum Level {
        START(-16), PICKUP(-16), DROP(240), REST(150);
        //start: -45, -17, pickup: 210,235
        public final int state;

        Level(int state) {
            this.state = state;
        }
    }

    public enum ServoAngel {
        PICKUP(0.1), DROP(0.4);

        public final double state;

        ServoAngel(double state) {
            this.state = state;
        }
    }

    Level currentLevel = Level.PICKUP;
    ServoAngel currentServoAngel = ServoAngel.PICKUP;

    DcMotorEx fourBarMotor;
    OpMode opMode;
    Servo clawAngelServo;

    double fourBarTarget = 0;

    public FourBarSystem(OpMode opMode){
        this.opMode = opMode;
        fourBarMotor = opMode.hardwareMap.get(DcMotorEx .class, "4Bar");
        clawAngelServo = opMode.hardwareMap.get(Servo.class, "FlipServo");
        clawAngelServo.setPosition(ServoAngel.PICKUP.state);
        fourBarMotor.setDirection( DcMotorSimple.Direction.REVERSE);
        fourBarMotor.setTargetPosition(Level.PICKUP.state);
        fourBarMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarMotor.setPower(0.5);
    }

    public int getCurrentMotorPosition() {
        return fourBarMotor.getCurrentPosition();
    }

    public double getCurrentServoPosition() {
        return clawAngelServo.getPosition();
    }

    public void setMotorPower(double power) {
        fourBarMotor.setPower(power);
    }

    public void setServoPosition(double position) {
        clawAngelServo.setPosition(position);
    }

    public void set4BarPosition(int target) {
        fourBarTarget = target;
        fourBarMotor.setTargetPosition(target);
    }
    public void set4BarPositionByLevel(Level targetLevel) {
        currentLevel = targetLevel;
        set4BarPosition(currentLevel.state);
    }

    public void setServoPosition(ServoAngel targetServoAngel) {
        currentServoAngel = targetServoAngel;
        clawAngelServo.setPosition(currentServoAngel.state);
    }

    public void toggle4Bar() {
        if(currentLevel == Level.START || currentLevel == Level.PICKUP || currentLevel == Level.REST) {
            set4BarPositionByLevel(Level.DROP);
        }
        else{
            set4BarPositionByLevel(Level.PICKUP);
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

    double last = 0;
    double change = 0;

    public void updateP() {
        final double modifair = 0.02, mod2 = 0.15;
        final double restGravityPosition = 34;
        change = getCurrentMotorPosition() - last;
        last = getCurrentMotorPosition();
        double deviation = fourBarTarget - getCurrentMotorPosition();
        double motorPower = -deviation / 100;

        double AngleGravity = (getCurrentMotorPosition() - restGravityPosition) / 270 * PI;
        double gravityPower =  - modifair * Math.cos(AngleGravity);
        motorPower += gravityPower - mod2 * change;

        opMode.telemetry.addData("Target", fourBarTarget);
        opMode.telemetry.addData("deviation", deviation);
        opMode.telemetry.addData("motorPower", motorPower);



        //if(deviation > 100) motorPower += modifair;
        //if(deviation < 20) motorPower -= modifair;
        setMotorPower(motorPower);
    }


}