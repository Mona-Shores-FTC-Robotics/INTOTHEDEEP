package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift.SampleLiftSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultSpecimenArmCommand extends CommandBase {

    private final SampleLiftSubsystem sampleLiftSubsystem;
    private final DoubleSupplier liftSupplier;

    public DefaultSpecimenArmCommand(SampleLiftSubsystem subsystem, DoubleSupplier liftStick) {
        sampleLiftSubsystem = subsystem;
        liftSupplier = liftStick;

        // Add the subsystem to the requirements
        addRequirements(sampleLiftSubsystem);
    }

    @Override
    public void execute() {
        double liftInput = liftSupplier.getAsDouble();

        if (Math.abs(liftInput) > SampleLiftSubsystem.SAMPLE_LIFT_PARAMS.LIFT_DEAD_ZONE_FOR_MANUAL_LIFT) {
            // Adjust the target position based on manual control method in subsystem
            sampleLiftSubsystem.setManualTargetState(liftInput); // this argument will be a value between 1 and -1
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // This command is the default and should not finish
    }
}
