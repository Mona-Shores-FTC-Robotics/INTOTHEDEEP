package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetectionStateMachine;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

public class SpecimenDetectionStateMachine {
    private final SpecimenIntakeSubsystem intakeSubsystem;
    private final SpecimenArmSubsystem armSubsystem;

    public SpecimenDetectionStateMachine(SpecimenIntakeSubsystem intakeSubsystem,
                                         SpecimenArmSubsystem armSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.armSubsystem = armSubsystem;
    }

    // If we detect a good specimen, shut off the intake and move the arm to staging position
    public void onSpecimenDetection() {
        intakeSubsystem.setCurrentState(SpecimenIntakeSubsystem.SpecimenIntakeStates.INTAKE_OFF);
        armSubsystem.flipCWFastAction();
    }
}
