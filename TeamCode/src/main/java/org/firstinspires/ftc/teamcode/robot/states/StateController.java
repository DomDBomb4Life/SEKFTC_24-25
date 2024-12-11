package org.firstinspires.ftc.teamcode.robot.states;

public class StateController {
    private State currentState;
    private State idleState;

    public StateController(State idleState) {
        this.idleState = idleState;
        setState(idleState);
    }

    public void setState(State newState) {
        if (currentState != null) {
            currentState.onExit();
        }
        currentState = newState;
        if (currentState != null) {
            currentState.onEnter();
        }
    }

    public void update() {
        if (currentState != null) {
            currentState.onUpdate();
            if (currentState.isCompleted()) {
                // After completion, return to idle by default
                setState(idleState);
            }
        }
    }

    public void handleUserInput(UserInput input) {
        if (currentState != null) {
            currentState.onUserInput(input);
        }
    }

    public String getCurrentStateName() {
        return currentState == null ? "None" : currentState.getClass().getSimpleName();
    }

    public String getCurrentSubstate() {
        return currentState == null ? "None" : currentState.getCurrentStep();
    }
}