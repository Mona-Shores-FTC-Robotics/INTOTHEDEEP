package org.firstinspires.ftc.teamcode.ObjectClasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import java.util.function.Supplier;
public class ConditionalAction implements Action {
    private final Action trueAction;
    private final Action falseAction;
    private final Supplier<Boolean> condition;
    private Action chosenAction; // intentionally null
    public ConditionalAction(Action trueAction, Action falseAction, Supplier<Boolean> condition) {
        this.trueAction = trueAction;
        this.falseAction = falseAction;
        this.condition = condition;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket t) {
        if (chosenAction == null) {
            // if we haven't decided on an action to run yet
            // (so on the first run of this action)
            if (condition.get()) { // use .get() to check the value of the condition by running the input function
                chosenAction = trueAction; // and then save the decision to the chosenAction variable
            } else {
                chosenAction = falseAction;
            }
        }
        // then, every loop, pass through the chosen action
        return chosenAction.run(t);
    }
    // ambiguous which one to preview, so preview both
    @Override
    public void preview(@NonNull Canvas canvas) {
        trueAction.preview(canvas);
        falseAction.preview(canvas);
    }
}