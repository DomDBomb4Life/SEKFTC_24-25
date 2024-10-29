// File: Robot.java
package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ViperLift;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.robot.states.HomeState;
import org.firstinspires.ftc.teamcode.robot.states.HangingState;
import org.firstinspires.ftc.teamcode.robot.states.ScoringBasketState;
import org.firstinspires.ftc.teamcode.robot.states.IdleState;
import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

/**
 * The Robot class encapsulates all the subsystems and manages state transitions.
 */
public class Robot {

    // Subsystems
    public final ViperLift viperLift;
    public final Arm arm;
    public final Claw claw;
    public final Wrist wrist;

    // State instances
    public final HomeState homeState;
    public final HangingState hangingState;
    public final ScoringBasketState scoringBasketState;
    public final IdleState idleState;

    // Drivetrain
    private final DriveTrainRR driveTrain;

    // Current state
    public enum State {
        IDLE,
        HOME,
        HANGING,
        SCORING
    }

    public State currentState = State.IDLE;

    // Constructor
    public Robot(HardwareMap hardwareMap, DriveTrainRR driveTrain) {
        // Initialize subsystems
        viperLift = new ViperLift(hardwareMap);
        arm = new Arm(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);

        // Initialize states
        homeState = new HomeState(viperLift, arm, wrist);
        hangingState = new HangingState(viperLift, arm);
        scoringBasketState = new ScoringBasketState(viperLift, arm, wrist, claw);
        idleState = new IdleState(viperLift, arm);

        // Activate initial state
        idleState.activate();

        // Initialize drivetrain
        this.driveTrain = driveTrain;
    }

    // Update method called periodically
    public void update() {
        // Update subsystems and states
        switch (currentState) {
            case HOME:
                homeState.update();
                break;

            case HANGING:
                hangingState.update();
                break;

            case SCORING:
                scoringBasketState.update();
                if (scoringBasketState.getCurrentStep() == ScoringBasketState.Step.COMPLETED) {
                    setState(State.IDLE);
                }
                break;

            case IDLE:
                idleState.update();
                break;

            default:
                break;
        }
    }

    // Method to set the current state
    public void setState(State newState) {
        if (currentState != newState) {
            currentState = newState;
            switch (newState) {
                case HOME:
                    homeState.activate();
                    break;

                case HANGING:
                    hangingState.start();
                    break;

                case SCORING:
                    scoringBasketState.start();
                    break;

                case IDLE:
                    idleState.activate();
                    break;

                default:
                    break;
            }
        }
    }
    // Autonomous actions
    public void performAutonomousActions() {
        // Placeholder for autonomous actions
        // This method must be expanded to include the sequence of actions during autonomous mode
    }

    // Methods to handle button presses
    public void onHomeButtonPressed() {
        setState(State.HOME);
    }

    public void onHangingButtonPressed() {
        setState(State.HANGING);
    }

    public void onScoringButtonPressed() {
        setState(State.SCORING);
    }

    public void onClawToggleButtonPressed() {
        if (claw.isClosed()) {
            claw.open();
        } else {
            claw.close();
        }
    }

    public void onSwitchVariationButtonPressed() {
        if (currentState == State.HOME) {
            homeState.switchVariation();
        }
    }

    // Methods to handle aimbot
    /*
    public void activateAimbot(Pose2d targetPose) {
        aimbotController.activate(targetPose);
    }

    public void deactivateAimbot() {
//        aimbotController.deactivate();
    }

     */
}