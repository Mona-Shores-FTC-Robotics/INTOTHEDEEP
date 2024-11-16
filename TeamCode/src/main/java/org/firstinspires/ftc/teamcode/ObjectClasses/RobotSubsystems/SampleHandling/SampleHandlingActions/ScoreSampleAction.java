// File: ScoreSampleAction.java
package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class ScoreSampleAction implements Action {

    private final SequentialAction actionSequence;
    private boolean started = false;

    public ScoreSampleAction() {
        // Define the sequence of actions within the constructor
        actionSequence = new SequentialAction(
                new InstantAction(Robot.getInstance().getSampleLiftBucketSubsystem()::dumpSampleInBucket),
                new DriveForwardFromBasket(10)
        );
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        // Run the internal action sequence
        if (!started) {
            started = true;
        }

        boolean isRunning = actionSequence.run(telemetryPacket);

        if (!isRunning) {
            reset();  // Reset the sequence when it completes
        }

        return isRunning;
    }

    private void reset() {
        started = false;
    }
}