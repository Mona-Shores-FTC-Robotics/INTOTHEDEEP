package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultSampleLiftCommand extends CommandBase {

    private final SampleLiftBucketSubsystem sampleLiftBucketSubsystem;
    private final DoubleSupplier liftSupplier;

    public DefaultSampleLiftCommand(SampleLiftBucketSubsystem subsystem, DoubleSupplier liftStick) {
        sampleLiftBucketSubsystem = subsystem;
        liftSupplier = liftStick;

        // Add the subsystem to the requirements
        addRequirements(sampleLiftBucketSubsystem);
    }

    @Override
    public void execute() {
        double liftInput = liftSupplier.getAsDouble();

        if (Math.abs(liftInput) > SampleLiftBucketSubsystem.SAMPLE_LIFT_PARAMS.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
            // Adjust the target position based on manual control method in subsystem
            sampleLiftBucketSubsystem.setManualTargetState(liftInput); // this argument will be a value between 1 and -1
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // This command is the default and should not finish
    }
}
