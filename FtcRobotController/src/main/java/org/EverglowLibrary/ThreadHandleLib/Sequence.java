package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.Executor;

import java.nio.channels.AsynchronousCloseException;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;

public class Sequence {

    private Queue<org.EverglowLibrary.Systems.Executor> m_Runs = new LinkedList<>();
    private Thread m_Thread = new Thread();
    private boolean m_IsRunAsync;
    private ExecutorService m_Service;
    private Runnable m_ToRun;
    private Queue<Future> m_Futures = new LinkedList<>();;


    public Sequence(boolean isRunAsync, Executor... AllRuns) {
        this(isRunAsync, Arrays.asList(AllRuns));
    }

    public Sequence(boolean isRunAsync, List<Executor> AllRuns) {
        m_Runs.addAll(AllRuns);
        m_IsRunAsync = isRunAsync;
        if(isRunAsync)
            setExecutorAsync();
        else
            setExecutorSync();
    }

    private void setExecutorSync() {
        m_ToRun = () -> {
            Queue<Executor> temp = new LinkedList<>();
            Executor toRun;
            while (m_Runs.size() != 0) {
                toRun = m_Runs.remove();
                temp.add(toRun);
                toRun.run();
                while (!toRun.isFinished()){

                }
            }

            while (temp.size() != 0){
                m_Runs.add(temp.remove());
            }
        };
    }


    private void setExecutorAsync() {
        if(m_Runs.size() > 0)
            m_Service = Executors.newFixedThreadPool(m_Runs.size());
    }

    public void startSequence() throws AsynchronousCloseException {
        if (m_IsRunAsync) {
            try {
                while (m_Runs.size() != 0) {
                    //run async the run method in order, save the Future class in Queue of the runs
                    m_Futures.add(m_Service.submit(m_Runs.remove()));
                }
            } catch (Exception e) {
                throw new AsynchronousCloseException();
            }
        } else{
            if(!m_Thread.isAlive()) {
                m_Thread = new Thread(m_ToRun);
                m_Thread.start();
            }
        }
    }

    public boolean isAllDone() {
        if (m_IsRunAsync) {
            for (Future future :
                    m_Futures) {
                if (!future.isDone())
                    return false;
            }
            return true;
        } else
            return isSyncThreadDone();
    }

    public boolean isSyncThreadDone() {
        if (m_Thread != null)
            return !m_Thread.isAlive();
        return true;
    }

    public Queue<Future> getFutures() {
        return m_Futures;
    }

    public Thread getThread() {
        return m_Thread;
    }

    public boolean isSequenceSync() {
        return m_IsRunAsync;
    }

    public void setRuns(Queue<Executor> runs) {
        //only if there is nothing in there or the size is zero
        if (m_Runs != null)
            if (m_Runs.size() == 0)
                m_Runs = runs;
            else{
                m_Runs = runs;
            }

        if(m_IsRunAsync)
            setExecutorAsync();
    }

    public boolean addRun(Executor run) {
        //only add run the thread isn't started yet
        if (isAllDone()) {
            m_Runs.add(run);
            if(m_IsRunAsync)
                setExecutorAsync();
            return true;
        }
        return false;
    }

    public void interruptSequence(){
        if(m_IsRunAsync)
            m_Service.shutdown();
        else
            m_Thread.interrupt();
    }

    //todo: set an method that alert when sequence done
}
