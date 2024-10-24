package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.DoubleSupplier;

public class DefaultSampleLiftCommand extends CommandBase {

    private final SampleLiftSubsystem sampleLiftSubsystem;
    private final DoubleSupplier liftSupplier;

    public DefaultSampleLiftCommand(SampleLiftSubsystem subsystem, DoubleSupplier liftStick) {
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
