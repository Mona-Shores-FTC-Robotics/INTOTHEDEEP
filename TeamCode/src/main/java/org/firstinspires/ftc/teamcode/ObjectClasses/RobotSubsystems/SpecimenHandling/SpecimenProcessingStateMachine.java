package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting.LightingSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

public class SpecimenProcessingStateMachine {
    private final SpecimenIntakeSubsystem intakeSubsystem;
    private final SpecimenArmSubsystem armSubsystem;
    private final LightingSubsystem lightingSubsystem;
    public SpecimenProcessingStateMachine(SpecimenIntakeSubsystem intakeSubsystem,
                                          SpecimenArmSubsystem armSubsystem,
                                          LightingSubsystem lightingSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
        this.lightingSubsystem = lightingSubsystem;
    }

    // If we detect a good specimen, shut off the intake and move the arm to clockwise home staging for scoring position
    public void onSpecimenDetection() {
        intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
        armSubsystem.flipCWFast();
//        lightingSubsystem.setGoodSampleIndicator();
    }
}
