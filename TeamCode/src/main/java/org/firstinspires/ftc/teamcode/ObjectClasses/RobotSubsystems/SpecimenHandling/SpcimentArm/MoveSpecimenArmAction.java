package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import android.annotation.SuppressLint;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MoveSpecimenArmAction implements Action {
    // State tracking variables
    private boolean hasNotInit = true;

    // Timeout indicator
    private boolean timeout;
    private SpecimenArmSubsystem specimenArmSubsystem;
    private final SpecimenArmSubsystem.SpecimenArmStates targetState;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private final double timeoutTimeSeconds;

    // Constructor with default timeout
    public MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates inputState) {
        this(inputState, SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.TIMEOUT_TIME_SECONDS);
    }

    // Constructor with custom timeout
    public MoveSpecimenArmAction(SpecimenArmSubsystem.SpecimenArmStates inputState, double timeoutTimeSeconds) {
        targetState = inputState;
        this.timeoutTimeSeconds = timeoutTimeSeconds;  // Use provided timeout
    }

    // Initialization method
    public void init() {
        specimenArmSubsystem = Robot.getInstance().getSpecimenArmSubsystem();

        // Reset the timeout timer and set timeout to false
        timeoutTimer.reset();
        timeout = false;
        specimenArmSubsystem.setTargetState(targetState);
    }

    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (hasNotInit) {
            init();
            hasNotInit = false; // Set to false after initialization
        }

        // Run-to-position motor action; no specific action needed in this loop
        if (isFinished()) {
            // When finished, call end method and stop the action
            end(telemetryPacket);
            return false;
        } else {
            // Action continues running
            return true;
        }
    }

    // Method to check if the action is finished
    public boolean isFinished() {
        // Check if the subsystem reports that the arm has finished its move
        boolean finished = specimenArmSubsystem.getCurrentState() == targetState;

        // Check for timeout using the specified timeout time (can be default or custom)
        boolean timedOut = timeoutTimer.seconds() > timeoutTimeSeconds;

        // If finished, update the state immediately
        if (finished) {
            specimenArmSubsystem.setCurrentState(targetState);
        }

        // Return true if either the target position is reached or the timeout occurs
        return finished || timedOut;
    }

    // Method to handle end of the action
    @SuppressLint("DefaultLocale")
    public void end(TelemetryPacket p) {
        hasNotInit=true;
        if (timeout) {
            p.addLine("Specimen Arm Move TIMEOUT");
            p.put("Timeout Timer", timeoutTimer.seconds());
            p.put("Target Angle", specimenArmSubsystem.getTargetAngleDegrees());
            p.put("Current Angle at Timeout", specimenArmSubsystem.getCurrentAngleDegrees());
        } else {
            // Report successful completion (state already set in isFinished)
            p.addLine(String.format("Specimen Arm Move COMPLETE: State: %s -> %s, Time: %.2f seconds",
                    specimenArmSubsystem.getCurrentState(), targetState, timeoutTimer.seconds()));
        }

        // Send the telemetry packet
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
