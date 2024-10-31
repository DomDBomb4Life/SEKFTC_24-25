// File: TurnActionFactoryImpl.java
package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TurnActionFactory;

import org.firstinspires.ftc.teamcode.drive.DriveTrainRR;

public class TurnActionFactoryImpl implements TurnActionFactory {

    private final DriveTrainRR driveTrain;

    public TurnActionFactoryImpl(DriveTrainRR driveTrain) {
        this.driveTrain = driveTrain;
    }

    @Override
    public Action make(TimeTurn turn) {
        return new Action() {
            boolean isRunning = false;

            @Override
            public boolean run(com.acmerobotics.dashboard.telemetry.TelemetryPacket packet) {
                if (!isRunning) {
                    driveTrain.turnAsync(turn.angle);
                    isRunning = true;
                }
                driveTrain.update();
                return driveTrain.isBusy();
            }
        };
    }
}