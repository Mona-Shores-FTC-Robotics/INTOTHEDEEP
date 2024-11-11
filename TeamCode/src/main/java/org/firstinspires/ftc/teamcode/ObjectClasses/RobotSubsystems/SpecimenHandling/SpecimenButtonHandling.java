package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;


import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

//Right now these are just for Teleop... should they be?
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
            case CCW_ARM_HOME:
                armSubsystem.setTargetStateWithMotionProfile(SpecimenArmSubsystem.SpecimenArmStates.SPECIMEN_PICKUP);
                intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_ON);
                break;
            case SPECIMEN_PICKUP:
                //It should automatically move out of this state, but it shouldn't hurt to have this here in case our detection doesn't work
                armSubsystem.flipCWFastAction();
                intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
                break;
            case CW_ARM_HOME:
                armSubsystem.flipCCWFastAction();
                break;
            default:
            case ARM_MANUAL:
                armSubsystem.setCurrentState(armSubsystem.lastState);
                onSpecimenHandleButtonPress();
                break;
        }
    }

    private void setIntakeState(SpecimenIntakeSubsystem.SpecimenIntakeStates newState) {
        intakeSubsystem.setCurrentState(newState);
    }

}
