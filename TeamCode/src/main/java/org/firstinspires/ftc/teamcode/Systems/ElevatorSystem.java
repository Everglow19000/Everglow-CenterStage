package org.firstinspires.ftc.teamcode.Systems;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ElevatorSystem {
    public enum Level {
        DOWN(0), UP(720);

        public final int state;

        Level(int state) {
            this.state = state;
        }
    }



    Level ElevatorCurrentLevel = Level.DOWN;


    /**
     * The motors which control the left and right sides of the elevator.
     */
    private final DcMotor left, right;


    public ElevatorSystem(OpMode opMode) {
        left = opMode.hardwareMap.get(DcMotor.class, "SlideL");
        right = opMode.hardwareMap.get(DcMotor.class, "SlideR");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setDirection(DcMotor.Direction.REVERSE);
        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//		left.setTargetPosition(0);
//		right.setTargetPosition(0);
//
//		left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//		right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//		left.setPower(0.7);
//		right.setPower(0.7);
        setPower(0);
    }

    /**
     * Moves the elevator to the specified state.
     *
     * @param level The level to move the elevator to.
     */
    public void goTo(ElevatorSystem.Level level) {
        left.setTargetPosition(level.state);
        right.setTargetPosition(level.state);
    }

    public void setPower(double power){
        final double scale = 0.7;
        left.setPower(power * scale);
        right.setPower(power * scale);
    }

    public void setPower(double powerL, double powerR){
        final double scale = 0.7;
        left.setPower(powerL * scale);
        right.setPower(powerR * scale);
    }

    public void toggle(){
        final double downPower = -0.4;
        final double upPower = 0.6;
        if(this.ElevatorCurrentLevel == Level.DOWN) {
            ElevatorCurrentLevel = Level.UP;
            left.setPower(upPower);
            right.setPower(upPower);
        }
        else{
            ElevatorCurrentLevel = Level.DOWN;
            left.setPower(downPower);
            right.setPower(downPower);
        }

        left.setTargetPosition(ElevatorCurrentLevel.state);
        right.setTargetPosition(ElevatorCurrentLevel.state);
    }
}
