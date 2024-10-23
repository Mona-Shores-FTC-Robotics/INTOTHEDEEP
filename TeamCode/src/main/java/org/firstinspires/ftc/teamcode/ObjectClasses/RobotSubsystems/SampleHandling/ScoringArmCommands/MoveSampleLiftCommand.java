package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftSubsystem.LIFT_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftSubsystem;

public class MoveSampleLiftCommand extends CommandBase {

    private final SampleLiftSubsystem sampleLiftSubsystem;
    private final SampleLiftSubsystem.SampleLiftStates targetState;
    private final ElapsedTime timeoutTimer = new ElapsedTime();

    public MoveSampleLiftCommand(SampleLiftSubsystem subsystem, SampleLiftSubsystem.SampleLiftStates inputState) {
        sampleLiftSubsystem = subsystem;
        targetState = inputState;

        addRequirements(sampleLiftSubsystem);
    }

    @Override
    public void initialize() {
        // Set the target state and position
        sampleLiftSubsystem.setTargetState(targetState);
        int targetTicks = targetState.getLiftHeightTicks();

        // Enforce limits on target ticks through the subsystem
        sampleLiftSubsystem.setTargetTicks(targetTicks);

        // Set motor behavior for reaching the target
        sampleLiftSubsystem.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sampleLiftSubsystem.lift.setTargetPosition(sampleLiftSubsystem.getTargetTicks());

        // Use unified power for movement
        sampleLiftSubsystem.lift.setPower(LIFT_PARAMS.LIFT_POWER);

        // Set motor to RUN_TO_POSITION mode
        sampleLiftSubsystem.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Start timeout timer
        timeoutTimer.reset();
    }

    @Override
    public void execute() {
        // No need for execution logic since the motor is handling the movement automatically in RUN_TO_POSITION mode
    }

    @Override
    public boolean isFinished() {
        // Check if the lift is within the threshold of the target position
        boolean finished = Math.abs(sampleLiftSubsystem.getCurrentTicks() - sampleLiftSubsystem.getTargetTicks()) < LIFT_PARAMS.LIFT_HEIGHT_TICK_THRESHOLD;

        // Check for timeout
        boolean timedOut = timeoutTimer.seconds() > LIFT_PARAMS.TIMEOUT_TIME_SECONDS;

        // If finished, update the state immediately
        if (finished) {
            sampleLiftSubsystem.setCurrentState(targetState);
        }

        return finished || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        String statusMessage;

        if (interrupted) {
            statusMessage = String.format("LiftSlide Move INTERRUPTED from %s to %s", sampleLiftSubsystem.getCurrentState(), targetState);
        } else if (timeoutTimer.seconds() > LIFT_PARAMS.TIMEOUT_TIME_SECONDS) {
            statusMessage = String.format("LiftSlide Move TIMEOUT after %.2f seconds", timeoutTimer.seconds());
        } else {
            // The state is already updated in isFinished(), so here we just confirm it completed
            statusMessage = String.format("LiftSlide Move COMPLETE from %s to %s in %.2f seconds", sampleLiftSubsystem.getCurrentState(), targetState, timeoutTimer.seconds());
        }

        // Add the status message to telemetryPacket with a consistent key
        MatchConfig.telemetryPacket.put("LiftSlide Command Status", statusMessage);
    }
}
