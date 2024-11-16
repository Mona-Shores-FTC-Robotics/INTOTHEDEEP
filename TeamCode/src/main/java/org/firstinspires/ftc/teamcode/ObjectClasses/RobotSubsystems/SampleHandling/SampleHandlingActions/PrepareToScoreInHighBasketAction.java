package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleHandlingActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class PrepareToScoreInHighBasketAction implements Action {

    private SequentialAction actionSequence;
    private boolean started = false;

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!started) {
            Robot robot = Robot.getInstance();
            actionSequence = new SequentialAction(
                    new InstantAction(robot.getSampleLiftBucketSubsystem()::moveLiftToHighBasket),
                    new InstantAction(robot.getSampleLiftBucketSubsystem()::moveDumperToPreScore),
                    new InstantAction(robot.getSampleLiftBucketSubsystem()::setBucketToScorePosition)
            );
            started = true;
        }

        // Run the action sequence and update telemetry
        boolean isRunning = actionSequence != null && actionSequence.run(telemetryPacket);

        if (!isRunning) {
            reset(); // Reset the state when the action completes
        }

        return !isRunning; // Return true when the action is complete
    }

    private void reset() {
        started = false;
        actionSequence = null;
    }
}