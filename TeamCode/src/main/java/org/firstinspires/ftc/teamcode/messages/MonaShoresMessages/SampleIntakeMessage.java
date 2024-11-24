package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;

public final class SampleIntakeMessage {
    public long timestamp;
    SampleIntakeSubsystem.SampleIntakeStates currentState;
    double power;

    public SampleIntakeMessage(  SampleIntakeSubsystem.SampleIntakeStates currentState, double power) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
        this.power = power;
    }
}
