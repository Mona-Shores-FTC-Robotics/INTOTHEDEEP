package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

public class SpecimenButtonHandling {

    private final SpecimenIntakeSubsystem intakeSubsystem;
    private final SpecimenArmSubsystem armSubsystem;

    // Constructor
    public SpecimenButtonHandling(SpecimenIntakeSubsystem intakeSubsystem,
                                  SpecimenArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    public void onSpecimenHandleButtonPress() {
        switch (armSubsystem.getCurrentState()) {
            default:
            case ZERO_POWER_AT_CCW_ARM_HOME:
            case CCW_ARM_HOME:
                armSubsystem.setTargetAngle(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
                if (!intakeSubsystem.getSpecimenDetector().haveSpecimen()) {
                    intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
                }
                break;
            case SPECIMEN_PICKUP:
                //It should automatically move out of this state, but it shouldn't hurt to have this here in case our detection doesn't work
                armSubsystem.flipCWFast();
                intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            case CW_ARM_HOME:
            case ZERO_POWER_AT_CW_ARM_HOME:
                armSubsystem.flipCCWFast();
                break;
        }
    }
}
