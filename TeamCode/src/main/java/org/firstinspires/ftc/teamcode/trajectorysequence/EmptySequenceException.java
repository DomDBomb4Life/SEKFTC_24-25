package org.firstinspires.ftc.teamcode.trajectorysequence;

/**
 * Exception thrown when attempting to build an empty trajectory sequence.
 */
public class EmptySequenceException extends RuntimeException {
    public EmptySequenceException() {
        super("Trajectory sequence is empty!");
    }
}