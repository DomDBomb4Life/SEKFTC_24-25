// File: TrajectoryActionFactoryImpl.java
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

public class TrajectoryActionFactoryImpl implements TrajectoryActionFactory {

    private final DriveTrainRR driveTrain;

    public TrajectoryActionFactoryImpl(DriveTrainRR driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public Action make(TimeTrajectory trajectory) {
        return new Action() {
            boolean isRunning = false;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                if (!isRunning) {
                    driveTrain.followTrajectoryAsync(trajectory);
                    isRunning = true;
                }
                driveTrain.update();
                return driveTrain.isBusy();
            }
        };
    }
}