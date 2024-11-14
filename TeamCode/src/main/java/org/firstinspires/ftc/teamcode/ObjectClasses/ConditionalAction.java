package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SleepAction;

import java.util.function.Supplier;

public class ConditionalAction implements Action {
    private final Action trueAction;
    private final Supplier<Boolean> condition;
    private final Action sleepAction;  // Sleep action to introduce delay between checks
    private boolean hasRunTrueAction = false;

    public ConditionalAction(Action trueAction, Supplier<Boolean> condition, long sleepMillis) {
        this.trueAction = trueAction;
        this.condition = condition;
        this.sleepAction = new SleepAction(sleepMillis);  // Pause duration between checks
    }

    @Override
    public boolean run(@NonNull TelemetryPacket t) {
        // Check condition; if true, execute the action
        if (condition.get()) {
            if (!hasRunTrueAction) {
                hasRunTrueAction = trueAction.run(t);
            }
            return hasRunTrueAction;
        }
        // If condition is false, run sleepAction to introduce delay
        return sleepAction.run(t);  // Returns false, effectively pausing until next check
    }
}
