package org.EverglowLibrary.ThreadHandleLib;

import java.nio.channels.AsynchronousCloseException;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.Queue;

public class SequenceInSequence{
    private Queue<Sequence> sequenceQueue = new LinkedList<>();
    private Thread m_Thread;

    public SequenceInSequence(Sequence... sequences){
        sequenceQueue.addAll(Arrays.asList(sequences));
        m_Thread = new Thread(() -> {
            Sequence[] sequences1 = sequenceQueue.toArray(new Sequence[sequenceQueue.size()]);
            for (int i = 0; i < sequences1.length; i++) {
                try {
                    sequences1[i].startSequence();

                    while (!sequences1[i].isDone()) {

                    }
                } catch (AsynchronousCloseException e) {
                    e.printStackTrace();
                }
            }
        });
    }

    public void add(Sequence sequence){
        sequenceQueue.add(sequence);
    }

    public void RunAll() {
        if(!m_Thread.isAlive()){
            m_Thread.start();
        }
    }

    public void stopAll(){
        for (Sequence seq:
             sequenceQueue) {
            seq.interruptSequence();
        }
    }
}
