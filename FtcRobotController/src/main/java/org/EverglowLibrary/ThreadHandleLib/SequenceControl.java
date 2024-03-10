package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;


public class SequenceControl {

//    private boolean seq1_toggle = false;
//    private boolean seq2_toggle = false;
//    private boolean seq3_toggle = false;
//    private boolean seq4_toggle = false;
//    private boolean upAndDown_toggle = false;

    private final Sequence getReadyToDropSeq;
    private final Sequence setUpAndUnderBlockSeq;
    private final Sequence dropAndRetreatSeq;
    private final Sequence getUpAndReadyToDrop;

    public SequenceControl(ClawSystem clawSystem, FourBarSystem fourBarSystem
            , ElevatorSystem elevatorSystem){
        getReadyToDropSeq = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        setUpAndUnderBlockSeq = new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        dropAndRetreatSeq = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP, FourBarSystem.ServoAngel.PICKUP)
                        , elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        getUpAndReadyToDrop = new Sequence(true, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));
    }

    public Sequence GetReadyToDropSeq(){
        return getReadyToDropSeq;
    }

    public Sequence SetUpAndUnderBlockSeq(){
        return setUpAndUnderBlockSeq;
    }

    public Sequence DropAndRetreatSeq(){
        return dropAndRetreatSeq;
    }

    public Sequence GetUpAndReadyToDrop(){
        return getUpAndReadyToDrop;
    }
}
