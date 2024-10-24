package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift;

import android.annotation.SuppressLint;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class MoveSampleLiftAction implements Action {
    // State tracking variables
    private boolean hasNotInit = true;
    // Timeout indicator
    private boolean timeout;
    private SampleLiftSubsystem sampleLiftSubsystem;
    private final SampleLiftSubsystem.SampleLiftStates targetState;
    private final ElapsedTime timeoutTimer = new ElapsedTime();
    private final double timeoutTimeSeconds;  // Now we have an optional timeout

    // Constructor with default timeout (from LIFT_PARAMS)
    public MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates inputState) {
        this(inputState, SampleLiftSubsystem.SAMPLE_LIFT_PARAMS.TIMEOUT_TIME_SECONDS);  // Default to the one in LIFT_PARAMS
    }

    // Constructor with custom timeout
    public MoveSampleLiftAction(SampleLiftSubsystem.SampleLiftStates inputState, double timeoutTimeSeconds) {
        targetState = inputState;
        this.timeoutTimeSeconds = timeoutTimeSeconds;  // Use provided timeout
    }

    // Initialization method
    public void init() {
        // Reference the SampleLiftSubsystem instance
        sampleLiftSubsystem = Robot.getInstance().getSampleLiftSubsystem();

        // Set the target state and target ticks in the subsystem
        sampleLiftSubsystem.setTargetState(targetState);

        // Reset the timeout timer and set timeout to false
        timeoutTimer.reset();
        timeout = false;

        // Set motor power and target position
        sampleLiftSubsystem.lift.setPower(SampleLiftSubsystem.SAMPLE_LIFT_PARAMS.LIFT_POWER);
        sampleLiftSubsystem.lift.setTargetPosition(sampleLiftSubsystem.getTargetTicks());

        // Set motor mode to RUN_TO_POSITION
        sampleLiftSubsystem.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        // Check if the subsystem reports that the lift has finished its move
        boolean finished = sampleLiftSubsystem.getCurrentState() == targetState;

        // Check for timeout using the specified timeout time (can be default or custom)
        boolean timedOut = timeoutTimer.seconds() > timeoutTimeSeconds;

        // If finished, update the state immediately
        if (finished) {
            sampleLiftSubsystem.setCurrentState(targetState);
        }

        // Return true if either the target position is reached or the timeout occurs
        return finished || timedOut;
    }

    // Method to handle end of the action
    @SuppressLint("DefaultLocale")
    public void end(TelemetryPacket p) {
        if (timeout) {
            p.addLine("SampleLift Move TIMEOUT");
            p.put("Timeout Timer", timeoutTimer.seconds());
            p.put("Target Position", sampleLiftSubsystem.getTargetTicks());
            p.put("Current Position at Timeout", sampleLiftSubsystem.getCurrentTicks());
        } else {
            // Report successful completion (state already set in isFinished)
            p.addLine(String.format("SampleLift Move COMPLETE: State: %s -> %s, Time: %.2f seconds",
                    sampleLiftSubsystem.getCurrentState(), targetState, timeoutTimer.seconds()));
        }

        // Send the telemetry packet
        FtcDashboard.getInstance().sendTelemetryPacket(p);
    }
}
