package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.ScoringArmCommands;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftSubsystem.LIFT_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultSampleLiftCommand extends CommandBase {

    // Declare the local variable to hold the lift subsystem
    private final SampleLiftSubsystem sampleLiftSubsystem;

    // Declare the local variable to hold the lift stick input
    private final DoubleSupplier liftSupplier;

    public DefaultSampleLiftCommand(SampleLiftSubsystem subsystem, DoubleSupplier liftStick) {
        sampleLiftSubsystem = subsystem;
        liftSupplier = liftStick;

        // Add the subsystem to the requirements
        addRequirements(sampleLiftSubsystem);
    }

    @Override
    public void initialize() {
        // Set the initial target ticks to the current ticks so that movements are relative to where the lift is right now
        sampleLiftSubsystem.setTargetTicks(sampleLiftSubsystem.getCurrentTicks());
        sampleLiftSubsystem.lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // Ensure motor holds position
        sampleLiftSubsystem.lift.setTargetPosition(sampleLiftSubsystem.getTargetTicks());
        sampleLiftSubsystem.lift.setPower(LIFT_PARAMS.LIFT_POWER); // Use unified power for initial setup
        sampleLiftSubsystem.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Set to RUN_TO_POSITION once during initialization
    }

    @Override
    public void execute() {
        double liftInput = liftSupplier.getAsDouble();

        // If the joystick is outside the dead zone, adjust the target position based on the input
        if (Math.abs(liftInput) > LIFT_PARAMS.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
            // Calculate new target ticks based on joystick input
            int newTargetTicks = calculateNewTargetTicks(liftInput);
            sampleLiftSubsystem.setTargetTicks(newTargetTicks);

            // Update the target position for the motor
            sampleLiftSubsystem.lift.setTargetPosition(sampleLiftSubsystem.getTargetTicks());
        }
        // If in dead zone, maintain the current target
        else {
            sampleLiftSubsystem.setTargetTicks(sampleLiftSubsystem.getCurrentTicks());
            sampleLiftSubsystem.lift.setTargetPosition(sampleLiftSubsystem.getTargetTicks());

            // Use holding power for stability
            sampleLiftSubsystem.lift.setPower(LIFT_PARAMS.HOLDING_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        // This command should never finish, as it is the default command for manual control
        return false;
    }

    private int calculateNewTargetTicks(double liftInput) {
        // Update the targetTicks based on the liftSupplier's value
        int deltaTicks = (int) Math.round(liftInput * LIFT_PARAMS.SCALE_FACTOR_FOR_MANUAL_LIFT);

        // Limit maximum change per execution to ensure smooth movement
        deltaTicks = Range.clip(deltaTicks, -LIFT_PARAMS.MAX_DELTA_TICKS, LIFT_PARAMS.MAX_DELTA_TICKS);

        // Update the targetTicks based on the current target, not the current motor position
        int newTargetTicks = sampleLiftSubsystem.getTargetTicks() + deltaTicks;

        // Ensure the new target is within the range of MIN_TICKS to MAX_TICKS using Range.clip
        return Range.clip(newTargetTicks, LIFT_PARAMS.MIN_TARGET_TICKS, LIFT_PARAMS.MAX_TARGET_TICKS);
    }
}
