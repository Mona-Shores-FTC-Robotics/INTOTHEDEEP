package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator;

import static org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLinearActuator.SampleLinearActuatorSubsystem.ACTUATOR_PARAMS;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultSampleLinearActuatorCommand extends CommandBase {

    private final SampleLinearActuatorSubsystem sampleLinearActuatorSubsystem;
    private final SampleIntakeSubsystem sampleIntakeSubsystem;
    private final DoubleSupplier actuatorSupplier;

    public DefaultSampleLinearActuatorCommand(SampleLinearActuatorSubsystem subsystem,
                                              SampleIntakeSubsystem intakeSubsystem,
                                              DoubleSupplier actuatorInput) {
        this.sampleLinearActuatorSubsystem = subsystem;
        this.sampleIntakeSubsystem = intakeSubsystem;
        this.actuatorSupplier = actuatorInput;
        addRequirements(sampleLinearActuatorSubsystem, sampleIntakeSubsystem);
    }

    @Override
    public void execute() {
        double actuatorInput = actuatorSupplier.getAsDouble();
        if (Math.abs(actuatorInput) > ACTUATOR_PARAMS.DEAD_ZONE_FOR_MANUAL_ACTUATION) {

            sampleLinearActuatorSubsystem.setFlipperHover();
            Robot.getInstance().getSampleTiwsterSubsystem().setTwisterServoFaceOutwards();

            // Move actuator based on input
            sampleLinearActuatorSubsystem.manualMove(actuatorInput);
            sampleIntakeSubsystem.turnOnIntake();  // Turn on intake

        } else if (sampleLinearActuatorSubsystem.getCurrentState() == SampleLinearActuatorSubsystem.SampleActuatorStates.MANUAL) {
            sampleLinearActuatorSubsystem.stopActuator();
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // This is the default command, so it runs continuously
    }
}
