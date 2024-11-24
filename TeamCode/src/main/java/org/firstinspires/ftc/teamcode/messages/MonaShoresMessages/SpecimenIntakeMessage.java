package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenIntake.SpecimenIntakeSubsystem;

public final class SpecimenIntakeMessage {
    public long timestamp;
    SpecimenIntakeSubsystem.SpecimenIntakeStates specimenIntakeState;
    double power;

    public SpecimenIntakeMessage(SpecimenIntakeSubsystem.SpecimenIntakeStates specimenIntakeState, double power) {
        this.timestamp = System.nanoTime();
        this.specimenIntakeState = specimenIntakeState;
        this.power = power;
    }
}
