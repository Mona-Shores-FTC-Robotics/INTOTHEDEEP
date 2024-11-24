package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;

public final class SampleLiftMessage {
    public long timestamp;

    public SampleLiftBucketSubsystem.SampleLiftStates currentState;

    public SampleLiftMessage(SampleLiftBucketSubsystem.SampleLiftStates  currentState) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
    }
}
