package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class ChangeSampleIntakePowerAction implements Action {

    // The subsystem the action runs on
    private final SampleIntakeSubsystem sampleIntakeSubsystem;

    // Target state for the intake motor
    private final SampleIntakeSubsystem.SampleIntakeStates targetState;

    private boolean hasNotInit = true;  // To track initialization

    // Constructor to initialize the action with the subsystem and target state
    public ChangeSampleIntakePowerAction(SampleIntakeSubsystem subsystem, SampleIntakeSubsystem.SampleIntakeStates inputState) {
        sampleIntakeSubsystem = subsystem;
        targetState = inputState;
    }

    // Initialization method
    public void init() {
        // Set the intake motor power based on the target state
        sampleIntakeSubsystem.setCurrentState(targetState);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) {
            init();  // Perform initialization if it hasn't been done
            hasNotInit = false;  // Set to false after initialization
        }
        // Run-to-position motor action; no specific action needed in this loop
        if (isFinished()) {
            // When finished, call end method and stop the action
            end(telemetryPacket);
            return false;
        } else {
            // Action continues running
            return true;
        } // This action only needs to run once
    }

    // Method to check if the action is finished
    public boolean isFinished() {
        return true;  // Action completes after changing the intake state
    }

    // Method to handle end of the action
    public void end(TelemetryPacket p) {
        hasNotInit=true;
        // Telemetry feedback to show state change completion
        p.addLine(String.format("Sample Intake Power set to %s (Power: %.2f)",
                targetState.toString(), targetState.power));

        // Send the telemetry packet to FtcDashboard
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
