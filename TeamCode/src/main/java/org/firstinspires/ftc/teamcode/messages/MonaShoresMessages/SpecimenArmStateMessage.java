package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.SpecimenArmSubsystem;

public final class SpecimenArmStateMessage {
    public long timestamp;
    public double currentAngleDegrees;
    public double currentVelocity;
    public SpecimenArmSubsystem.SpecimenArmStates currentState;

    public SpecimenArmStateMessage(double currentAngleDegrees, double currentVelocity, SpecimenArmSubsystem.SpecimenArmStates currentState) {
        this.timestamp = System.nanoTime();
        this.currentAngleDegrees = currentAngleDegrees;
        this.currentVelocity = currentVelocity;
        this.currentState = currentState;

    }
}
