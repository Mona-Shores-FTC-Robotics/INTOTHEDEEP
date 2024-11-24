package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;

public final class SampleDumperMessage {
    public long timestamp;

    public SampleLiftBucketSubsystem.DumperStates currentState;

    public SampleDumperMessage(SampleLiftBucketSubsystem.DumperStates currentState) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
    }
}
