package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.ClawSystem;
import org.EverglowLibrary.Systems.ElevatorSystem;
import org.EverglowLibrary.Systems.Executor;
import org.EverglowLibrary.Systems.FourBarSystem;


public class SequenceControl {

    private final Sequence getReadyToDropSeq;
    private final SequenceInSequence setUpAndUnderBlockSeq;
    private final Sequence dropAndRetreatSeq;
    private final Sequence getUpAndReadyToDrop;

    public SequenceControl(ClawSystem clawSystem, FourBarSystem fourBarSystem
            , ElevatorSystem elevatorSystem){
        getReadyToDropSeq = new Sequence(false, clawSystem.getExecutor(false)
                ,elevatorSystem.getExecutor(ElevatorSystem.Level.UP)
                , fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP));

        setUpAndUnderBlockSeq = new SequenceInSequence(false,
                new Sequence(false, clawSystem.getExecutor(false),
                elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.DROP)),
                new Sequence(true,fourBarSystem.getExecutor(FourBarSystem.Level.REST, FourBarSystem.ServoAngel.REST),
                elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN)));

        dropAndRetreatSeq = new Sequence(false, fourBarSystem.getExecutor(FourBarSystem.Level.PICKUP
                        , FourBarSystem.ServoAngel.PICKUP)
                        , elevatorSystem.getExecutor(ElevatorSystem.Level.DOWN));

        getUpAndReadyToDrop = new Sequence(false, elevatorSystem.getExecutor(ElevatorSystem.Level.UP),
                fourBarSystem.getExecutor(FourBarSystem.Level.DROP, FourBarSystem.ServoAngel.DROP,1));
    }

    public Sequence GetReadyToDropSeq(){
        return getReadyToDropSeq;
    }

    public SequenceInSequence SetUpAndUnderBlockSeq(){
        return setUpAndUnderBlockSeq;
    }

    public Sequence DropAndRetreatSeq(){
        return dropAndRetreatSeq;
    }

    public Sequence GetUpAndReadyToDrop(){
        return getUpAndReadyToDrop;
    }
}
