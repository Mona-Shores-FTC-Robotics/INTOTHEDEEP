package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpcimentArm;

import com.arcrobotics.ftclib.command.CommandBase;

import java.util.function.DoubleSupplier;

public class DefaultSpecimenArmCommand extends CommandBase {

    private final SpecimenArmSubsystem specimenArmSubsystem;
    private final DoubleSupplier armSupplier;

    public DefaultSpecimenArmCommand(SpecimenArmSubsystem subsystem, DoubleSupplier armStick) {
        specimenArmSubsystem = subsystem;
        armSupplier = armStick;

        // Add the subsystem to the requirements
        addRequirements(specimenArmSubsystem);
    }

    @Override
    public void execute() {
        double armInput = armSupplier.getAsDouble();
        if (Math.abs(armInput) > SpecimenArmSubsystem.SPECIMEN_ARM_PARAMS.DEAD_ZONE){
            // Adjust the target position based on manual control method in subsystem
            specimenArmSubsystem.setManualTargetState(-armInput); // this argument will be a value between 1 and -1
        }
    }

    @Override
    public boolean isFinished() {
        return false;  // This command is the default and should not finish
    }
}
