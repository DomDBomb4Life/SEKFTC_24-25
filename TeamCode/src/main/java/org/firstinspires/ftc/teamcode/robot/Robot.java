// File: Robot.java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.states.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {

    // Subsystems
    public final ViperLift viperLift;
    public final Arm arm;
    public final Claw claw;
    public final Wrist wrist;

    // State instances
    private BaseState currentState;
    public IdleState idleState;
    public HomeState homeState;
    public ScoringBasketState scoringBasketState;
    public ScoringSpecimenState scoringSpecimenState;
    public PickupState pickupState;
    public ObservationState observationState;
    public LevelOneAscentState levelOneAscentState;

    // Constructor
    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public Robot(HardwareMap hardwareMap, boolean isAutonomous) {
        // Initialize subsystems
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap, claw);
        arm = new Arm(hardwareMap);
        viperLift = new ViperLift(hardwareMap);

        // Initialize states
        idleState = new IdleState(this, isAutonomous);
        homeState = new HomeState(this, isAutonomous);
        scoringBasketState = new ScoringBasketState(this, isAutonomous);
        scoringSpecimenState = new ScoringSpecimenState(this, isAutonomous);
        pickupState = new PickupState(this, isAutonomous);
        observationState = new ObservationState(this, isAutonomous);
        levelOneAscentState = new LevelOneAscentState(this, isAutonomous);

        // Set initial state
        setState(idleState);
    }

    // Update method called periodically
    public void update() {
        if (currentState != null) {
            currentState.update();
        }
    }

    // Method to set the current state
    public void setState(BaseState newState) {
        if (currentState != null) {
            currentState.deactivate();
        }
        currentState = newState;
        if (currentState != null) {
            currentState.activate();
        }
    }

    // Methods to handle button presses
    public void onHomeButtonPressed() {
        if (currentState != homeState){
        setState(homeState);
        } else{
            setState(idleState);
        }
    }

    public void onScoringButtonPressed() {
        setState(scoringBasketState);
    }

    public void onScoringSpecimenButtonPressed() {
        setState(scoringSpecimenState);
    }

    public void onObservationButtonPressed() {
        setState(observationState);
    }

    public void onLevelOneAscentButtonPressed() {
        setState(levelOneAscentState);
    }

    public void onPickupButtonPressed() {
        setState(pickupState);
    }

    public void onClawToggleButtonPressed() {
        if (claw.isClosed()) {
            claw.open();
        } else {
            claw.close();
        }
    }

    public void onRightTriggerPressed() {
        if (currentState != null) {
            currentState.onRightTriggerPressed();
        }
    }

    public String getCurrentStateName() {
        if (currentState != null) {
            return currentState.getClass().getSimpleName();
        }
        return "None";
    }

    public String getCurrentSubstate() {
        if (currentState != null) {
            return currentState.getCurrentStep();
        }
        return "None";
    }

    public boolean isCurrentStateCompleted() {
        if (currentState != null) {
            return currentState.isCompleted();
        }
        return false;
    }
}