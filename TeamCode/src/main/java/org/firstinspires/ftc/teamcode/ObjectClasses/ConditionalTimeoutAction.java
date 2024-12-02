package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.Supplier;

public class ConditionalTimeoutAction implements Action {
    private final Action trueAction;
    private final Supplier<Boolean> condition;
    private final Action timeoutAction;
    private final SleepAction sleepAction = new SleepAction(.1); // Reuse instance
    private boolean hasRunAction = false;
    private boolean started = false;

    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private final double timeoutTime;

    public ConditionalTimeoutAction(Action trueAction, Action timeoutAction, Supplier<Boolean> condition, double timeoutTime) {
        this.trueAction = trueAction;
        this.condition = condition;
        this.timeoutAction = timeoutAction;
        this.timeoutTime = timeoutTime;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket t) {
        if (!started) {
            // Initialize the state when the action starts
            MatchConfig.telemetryPacket.addLine("Action started");
            started = true;
            hasRunAction = false;
            timeoutTimer.reset();
        }

        if (condition.get()) {
            // If the condition is met, run the trueAction
            MatchConfig.telemetryPacket.addLine("Condition met, running trueAction");
            if (trueAction.run(t)) {
                return true; // Continue running until trueAction completes
            } else {
                resetState();
                return false; // trueAction is complete, this action is done
            }
        } else if (timeoutTimer.milliseconds() > timeoutTime) {
            // If timeout occurs, run the timeoutAction
            MatchConfig.telemetryPacket.addLine("Timeout reached, running timeoutAction");
            if (timeoutAction.run(t)) {
                return true; // Continue running until timeoutAction completes
            } else {
                resetState();
                return false; // timeoutAction is complete, this action is done
            }
        } else {
            // Neither condition met nor timeout, perform a short delay
            MatchConfig.telemetryPacket.addLine("Condition not met, sleeping");
            sleepAction.run(t); // Use the persistent sleepAction instance
            return true; // Continue checking in subsequent calls
        }
    }

    private void resetState() {
        started = false;
        hasRunAction = false;
    }
}
