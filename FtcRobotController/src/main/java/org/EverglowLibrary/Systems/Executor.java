package org.EverglowLibrary.Systems;

import org.EverglowLibrary.utils.ExecutorSleep;

public abstract class Executor implements ISequenceable{
    //can add something.. like toggle

    public static Executor sleep(long time){
        return new ExecutorSleep(time);
    }
}
