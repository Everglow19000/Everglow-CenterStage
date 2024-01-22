package org.firstinspires.ftc.teamcode.OpModes.Tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

public class Elevators {
    /**
     * Enum encapsulating all the positions the elevator should reach.
     */
    public enum Level {
        PICKUP(0), UP(300);

        public final int state;

        Level(int state) {
            this.state = state;
        }
    }

    /**
     * The motor which controls the left side of the elevator.
     */
    private final DcMotor left;
    /**
     * The motor which controls the right side of the elevator.
     */
    private final DcMotor right;

    public Elevators(OpMode opMode) {
        left = opMode.hardwareMap.get(DcMotor.class, "left_elevator");
        right = opMode.hardwareMap.get(DcMotor.class, "right_elevator");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
    public void goTo(Elevators.Level level) {
        left.setTargetPosition(level.state);
        right.setTargetPosition(level.state);
    }

    public void setPower(double power){
        final double scale = 0.7;
        left.setPower(power * scale);
        right.setPower(power * scale);
    }

    public void setPower(double powerL, double powerR){
        double scaleR = 1;
        double scaleL = 1;
        double eps = 0.1;
        if(left.getCurrentPosition() > 0 && right.getCurrentPosition() > 0){
            scaleR = Math.max(left.getCurrentPosition()/right.getCurrentPosition(), 1);
            scaleL  = Math.max(right.getCurrentPosition()/left.getCurrentPosition(), 1);
        }
        left.setPower(powerL * scaleL);
        right.setPower(powerR * scaleR);
    }
}
