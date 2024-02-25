package org.EverglowLibrary.ThreadHandleLib;

import org.EverglowLibrary.Systems.Executor;

public class SequenceRunner {
    private boolean m_IsFinished = true;
    private Sequence m_RunningSequence;
    private Sequence[] m_AllSequences;
    private int SequenceIndex = 0;
    private Executor[] m_Runs;
    private int CurrentRun = 0;
    private boolean isInterapted = false;
    private boolean isSync;

    public SequenceRunner(Sequence sequence){
        TrySetSequence(sequence);
    }

    public SequenceRunner(){

    }

    public SequenceRunner(Sequence... sequences){
        TrySetSequence(sequences);
    }

    public SequenceRunner(SequenceInSequence sequence){
        TrySetSequence(sequence.GetAllSequences());
    }

    public void Update(){
        if(m_Runs == null)
            return;

        if(isInterapted) {
            m_Runs[CurrentRun].stop();
            Reset();
            return;
        }

        if(m_Runs[CurrentRun].isFinished() || !isSync){
            CurrentRun++;
            if(CurrentRun < m_Runs.length){
                m_Runs[CurrentRun].run();
            }
            else{
                SequenceIndex++;
                m_IsFinished = true;
                m_Runs = null;

                if(SequenceIndex < m_AllSequences.length)
                {
                    m_RunningSequence = m_AllSequences[SequenceIndex];
                    RunSequence();
                }
                else
                {
                    Reset();
                }
            }
        }
    }
    public void RunSequence(){
        if(m_IsFinished && m_RunningSequence != null)
        {
            m_IsFinished = false;
            m_Runs = m_RunningSequence.GetRuns();
            isSync = m_RunningSequence.isSequenceSync();
            CurrentRun = 0;
            m_Runs[CurrentRun].run();
        }
    }

    public void RunSequence(Sequence sequence){
        if(m_IsFinished) {
            TrySetSequence(sequence);
            RunSequence();
        }
    }

    public void RunSequence(Sequence[] sequences){
        if(m_IsFinished) {
            TrySetSequence(sequences);
            RunSequence();
        }
    }

    public void RunSequence(SequenceInSequence sequences){
        if(m_IsFinished) {
            TrySetSequence(sequences.GetAllSequences());
            RunSequence();
        }
    }

    public void TrySetSequence(Sequence sequence){
        TrySetSequence(new Sequence[] {sequence});
    }

    public void TrySetSequence(Sequence[] sequences){
        if(m_IsFinished){
            m_AllSequences = sequences;
            SequenceIndex = 0;
            m_RunningSequence = sequences[SequenceIndex];
            isSync = m_RunningSequence.isSequenceSync();
        }
    }

    private void Reset(){
        m_Runs = null;
        SequenceIndex = 0;
        m_AllSequences = null;
        CurrentRun = 0;
    }
    public boolean IsSequenceDone(){
        return m_IsFinished;
    }

    public void Interapt(){
        isInterapted = true;
        Update();
    }

}
