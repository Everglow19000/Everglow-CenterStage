package org.EverglowLibrary.ExecuteMotor;

public class WarpedExecute implements Runnable{
    State goTo;
    RobotExecute theExecute;

    public WarpedExecute(RobotExecute execute, State state){
        goTo = state;
        theExecute = execute;
    }

    @Override
    public void run(){
        theExecute.Execute(goTo);
    }
}
