package org.firstinspires.ftc.teamcode.OpModes.Tests;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.opencv.core.Mat;

public class Elevators {
    /**
     * Enum encapsulating all the positions the elevator should reach.
     */
    public enum Level {
        PICKUP(0), UP(764);

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
    private OpMode opMode;

    public Elevators(OpMode opMode) {
        this.opMode = opMode;
        left = opMode.hardwareMap.get(DcMotor.class, "SlideL");
        right = opMode.hardwareMap.get(DcMotor.class, "SlideR");

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//		left.setTargetPosition(0);
//		right.setTargetPosition(0);
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
        this.opMode.telemetry.addData("level", level.state);
        opMode.telemetry.update();
        left.setTargetPosition(-1*level.state);
        right.setTargetPosition(level.state);
    }

    public void setPower(double power){
        final double scale = 0.7;
        right.setPower(power * scale);
        left.setPower(power * scale);
    }


    public void setPower(double powerL, double powerR){
        double scaleR = 0.7;
        double scaleL = 0.7;
        /*
        double leftBC = (double) Math.abs(left.getCurrentPosition());
        double rightBC = (double)Math.abs(right.getCurrentPosition());
        if(-left.getCurrentPosition() > 0 && right.getCurrentPosition() > 0){
            scaleR = Math.min(leftBC/rightBC, 1);
            scaleL  = Math.min(rightBC/leftBC, 1);
        }

        if(-left.getCurrentPosition() > 0 && right.getCurrentPosition() > 0) {

            this.opMode.telemetry.addData("scaleR", scaleR);
            this.opMode.telemetry.addData("scaleL", scaleL);
            this.opMode.telemetry.addData("powerL:", powerL);
            this.opMode.telemetry.addData("powerR:", powerL);
            this.opMode.telemetry.addData("set power L:", powerL * scaleL);
            this.opMode.telemetry.addData("set power R:", powerR * scaleR);
            opMode.telemetry.update();
        }
         */
        right.setPower(powerR * scaleR);
        left.setPower(powerL * scaleL);

    }
}
