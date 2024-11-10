package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem.ACTUATOR_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultSampleLinearActuatorCommand extends CommandBase {

    private final SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem;
    private final DoubleSupplier actuatorSupplier;

    public DefaultSampleLinearActuatorCommand(SampleLinearActuatorSubsystem subsystem, DoubleSupplier actuatorInput) {
        this.sampleLinearActuatorSubsystem = subsystem;
        this.actuatorSupplier = actuatorInput;
        addRequirements(sampleLinearActuatorSubsystem);
    }

    @Override
    public void execute() {
        double actuatorInput = actuatorSupplier.getAsDouble();

        if (Math.abs(actuatorInput) > ACTUATOR_PARAMS.DEAD_ZONE_FOR_MANUAL_ACTUATION) {
            // Convert joystick input into movement, adjust target ticks based on manual input
            sampleLinearActuatorSubsystem.manualMove(-actuatorInput);
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // This is the default command, so it runs continuously
    }
}
