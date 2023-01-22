package com.github.mittyrobotics;

public class StateMachine {
    private State current = State.NONE;
    private State lastState;

    public static StateMachine instance;

    public static StateMachine getInstance() {
        if(instance == null) {
            instance = new StateMachine();
        }
        return instance;
    }

    public State getCurrentState() {
        return current;
    }

    public State getLastState() {
        return lastState;
    }

    public void setStateCube() {
        current = State.CUBE;
        lastState = State.CUBE;
    }

    public void setStateCone() {
        current = State.CONE;
        lastState = State.CONE;
    }

    public void setStateNone() {
        current = State.NONE;
    }

    public enum State {
        CONE,
        CUBE,
        NONE
    }

}
