package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MoveLinearActuatorAction implements Action {
    // State tracking variables
    private boolean hasNotInit = true;
    private boolean timeout;
    private SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem;
    private final SampleLinearActuatorSubsystem.SampleActuatorStates targetState;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private final double timeoutTimeSeconds;  // Optional timeout for the action

    // Constructor with default timeout (from ACTUATOR_PARAMS)
    public MoveLinearActuatorAction(SampleLinearActuatorSubsystem.SampleActuatorStates inputState) {
        this(inputState, SampleLinearActuatorSubsystem.ACTUATOR_PARAMS.TIMEOUT_TIME_SECONDS);
    }

    // Constructor with custom timeout
    public MoveLinearActuatorAction(SampleLinearActuatorSubsystem.SampleActuatorStates inputState, double timeoutTimeSeconds) {
        targetState = inputState;
        this.timeoutTimeSeconds = timeoutTimeSeconds;
    }

    // Initialization method
    public void init() {
        // Reference the SampleLinearActuatorSubsystem instance
        sampleLinearActuatorSubsystem = Robot.getInstance().getSampleLinearActuatorSubsystem();

        // Set the target state and position in the subsystem
        sampleLinearActuatorSubsystem.setTargetState(targetState);

        // Reset the timeout timer and set timeout to false
        timeoutTimer.reset();
        timeout = false;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) {
            init();
            hasNotInit = false;  // Set to false after initialization
        }

        // Run-to-position motor action; no specific action needed in this loop
        return isFinished();
    }

    // Method to check if the action is finished
    public boolean isFinished() {
        // Check if the actuator has reached its target
        boolean finished = sampleLinearActuatorSubsystem.isActuatorAtTarget();

        // Check for timeout using the specified timeout time (can be default or custom)
        boolean timedOut = timeoutTimer.seconds() > timeoutTimeSeconds;

        // Return true if finished or timeout occurred
        return finished || timedOut;
    }

    // Method to handle end of the action
    public void end(TelemetryPacket p) {
        // Force a final state update to ensure currentState is in sync
        sampleLinearActuatorSubsystem.updateActuatorState();

        if (timeout) {
            p.addLine("Linear Actuator Move TIMEOUT");
            p.put("Timeout Timer", timeoutTimer.seconds());
            p.put("Target Position", sampleLinearActuatorSubsystem.getTargetTicks());
            p.put("Current Position at Timeout", sampleLinearActuatorSubsystem.getCurrentTicks());
        } else {
            // Report successful completion with state transition details
            p.addLine(String.format("Linear Actuator Move COMPLETE: State: %s -> %s",
                    sampleLinearActuatorSubsystem.getCurrentState(), targetState));
            p.put("Completion Time", timeoutTimer.seconds());
            p.put("Final Position", sampleLinearActuatorSubsystem.getCurrentTicks());
        }

        // Send the telemetry packet
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}