package org.firstinspires.ftc.teamcode.robot.states;

public interface State {
    void onEnter();
    void onExit();
    void onUpdate();
    void onUserInput(UserInput input); // e.g., RIGHT_TRIGGER, PRIMARY_BUTTON, etc.
    boolean isCompleted();
    String getCurrentStep();
}
