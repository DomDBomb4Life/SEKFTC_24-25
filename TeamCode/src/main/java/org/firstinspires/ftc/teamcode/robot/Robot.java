package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.states.*;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Robot {

    public final ViperLift viperLift;
    public final Arm arm;
    public final Claw claw;
    public final Wrist wrist;

    // States
    private StateController stateController;
    public IdleState idleState;
    public HomeState homeState;
    public ScoringBasketState scoringBasketState;
    public ScoringSpecimenState scoringSpecimenState;
    public PickupState pickupState;
    public ObservationState observationState;
    public LevelOneAscentState levelOneAscentState;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, false);
    }

    public Robot(HardwareMap hardwareMap, boolean isAutonomous) {
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap, claw);
        arm = new Arm(hardwareMap);
        viperLift = new ViperLift(hardwareMap);

        idleState = new IdleState(this, isAutonomous);
        homeState = new HomeState(this, isAutonomous);
        scoringBasketState = new ScoringBasketState(this, isAutonomous);
        scoringSpecimenState = new ScoringSpecimenState(this, isAutonomous);
        pickupState = new PickupState(this, isAutonomous);
        observationState = new ObservationState(this, isAutonomous);
        levelOneAscentState = new LevelOneAscentState(this, isAutonomous);

        stateController = new StateController(idleState);
    }

    public void update() {
        stateController.update();
    }

    public void setState(State newState) {
        stateController.setState(newState);
    }

    public void onClawToggleButtonPressed() {
        if (claw.isClosed()) {
            claw.open();
        } else {
            claw.close();
        }
    }

    public void onRightTriggerPressed() {
        stateController.handleUserInput(UserInput.RIGHT_TRIGGER);
    }

    public void onHomeButtonPressed() {
        if (stateController.getCurrentStateName().equals(homeState.getClass().getSimpleName())) {
            // If already in HomeState, return to Idle
            stateController.setState(idleState);
        } else {
            stateController.setState(homeState);
        }
    }

    public void onScoringButtonPressed() {
        stateController.setState(scoringBasketState);
    }

    public void onScoringSpecimenButtonPressed() {
        stateController.setState(scoringSpecimenState);
    }

    public void onObservationButtonPressed() {
        stateController.setState(observationState);
    }

    public void onLevelOneAscentButtonPressed() {
        stateController.setState(levelOneAscentState);
    }

    public void onPickupButtonPressed() {
        stateController.setState(pickupState);
    }

    public String getCurrentStateName() {
        return stateController.getCurrentStateName();
    }

    public String getCurrentSubstate() {
        return stateController.getCurrentSubstate();
    }

    public boolean isCurrentStateCompleted() {
        // If the current state completed last loop, it would've returned to idle
        return stateController.getCurrentStateName().equals("IdleState");
    }
}