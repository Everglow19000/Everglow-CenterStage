package org.EverglowLibrary.Systems;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class FourBarSystem{

    public Executor getExecutor(Level level,ServoAngel servoAngel){
        return new FourBarExecutor(servoAngel,level);
    }

    public enum Level {
        START(-12), PICKUP(-12), DROP(344), REST(267), LOW(250);
        //start: -10, pickup: 210,235
        public final int state;

        Level(int state) {
            this.state = state;
        }
    }

    public enum ServoAngel {
        PICKUP(0.445), DROP(0.14), REST(0.66), LOW(0.3);

        public final double state;

        ServoAngel(double state) {
            this.state = state;
        }
    }

    Level currentLevel = Level.PICKUP;
    ServoAngel currentServoAngel = ServoAngel.PICKUP;

    DcMotorEx fourBarMotor;
    LinearOpMode opMode;
    Servo clawAngelServo;

    double fourBarTarget = 0;

    public FourBarSystem(LinearOpMode opMode){
        this.opMode = opMode;
        fourBarMotor = opMode.hardwareMap.get(DcMotorEx .class, "4Bar");
        clawAngelServo = opMode.hardwareMap.get(Servo.class, "FlipServo");
        clawAngelServo.setPosition(ServoAngel.PICKUP.state);

        //fourBarMotor.setDirection( DcMotorSimple.Direction.REVERSE);
        fourBarMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourBarMotor.setTargetPosition(Level.START.state);
        fourBarMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        fourBarMotor.setPower(0.5);
        //clawAngelServo.resetDeviceConfigurationForOpMode();
    }

    public boolean isFinish(Level level){
        int epsilon4Bar = 12;
        return (fourBarMotor.getCurrentPosition() >= level.state - epsilon4Bar) &&
                (fourBarMotor.getCurrentPosition() <= level.state + epsilon4Bar);
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

    public void setServoPositionByLevel(ServoAngel targetServoAngel) {
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
            setServoPositionByLevel(ServoAngel.DROP);
        }
        else{
            setServoPositionByLevel(ServoAngel.PICKUP);
        }
    }

    double last = 0;
    double change = 0;

    public void updateP(double modifairG) {
        final double mod2 = 0.15;
        final double restGravityPosition = 34;
        change = getCurrentMotorPosition() - last;
        last = getCurrentMotorPosition();
        double deviation = fourBarTarget - getCurrentMotorPosition();
        double motorPower = -deviation / 70;

        double AngleGravity = (getCurrentMotorPosition() - restGravityPosition) / 270 * PI;
        double gravityPower =  modifairG * Math.sin(AngleGravity);
        motorPower += gravityPower - mod2 * change;

        //opMode.telemetry.addData("Target", fourBarTarget);
        //opMode.telemetry.addData("deviation", deviation);
        //opMode.telemetry.addData("motorPower", motorPower);
        //if(deviation > 100) motorPower += modifair;
        //if(deviation < 20) motorPower -= modifair;
        setMotorPower(motorPower);
    }


    public class FourBarExecutor extends Executor{

        private final ServoAngel m_ServoAngle;
        private final Level m_Level;

        public FourBarExecutor(ServoAngel servoAngel, Level level) {
            m_ServoAngle = servoAngel;
            m_Level = level;
        }

        @Override
        public void run() {
            if(m_Level == Level.PICKUP){
                //fourBarMotor.setPower(0.65);
                set4BarPositionByLevel(m_Level);
                opMode.sleep(800);
                setServoPositionByLevel(m_ServoAngle);
            }
            else {
                //fourBarMotor.setPower(0.6);
                setServoPositionByLevel(m_ServoAngle);
                set4BarPositionByLevel(m_Level);
            }
        }


        @Override
        public boolean isFinished() {
            return isFinish(m_Level);
        }

        @Override
        public void stop() {
            fourBarMotor.setPower(0);
        }


    }
}