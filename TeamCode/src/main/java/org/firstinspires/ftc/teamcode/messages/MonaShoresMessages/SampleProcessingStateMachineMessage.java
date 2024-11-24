package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleProcessingStateMachine;

public final class SampleProcessingStateMachineMessage {
    public long timestamp;
    SampleProcessingStateMachine.SampleDetectionStates  currentState;

    public SampleProcessingStateMachineMessage(SampleProcessingStateMachine.SampleDetectionStates currentState) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
    }
}
