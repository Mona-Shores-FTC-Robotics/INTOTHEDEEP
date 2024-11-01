package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;
public class ConditionalAction implements Action {
    private final Action trueAction;
    private final Action falseAction;
    private final Supplier<Boolean> condition;

    public ConditionalAction(Action trueAction, Action falseAction, Supplier<Boolean> condition) {
        this.trueAction = trueAction;
        this.falseAction = falseAction;
        this.condition = condition;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket t) {
        // Choose and run the appropriate action based on the condition
        // Track the current action based on the condition
        Action activeAction = condition.get() ? trueAction : falseAction;
        return activeAction.run(t);  // Run the selected action and return its completion status
    }
}