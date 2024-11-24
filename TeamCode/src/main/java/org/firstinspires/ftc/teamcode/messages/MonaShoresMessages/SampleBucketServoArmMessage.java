package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleLiftBucket.SampleLiftBucketSubsystem;

public final class SampleBucketServoArmMessage {
    public long timestamp;

    public SampleLiftBucketSubsystem.BucketStates currentState;

    public SampleBucketServoArmMessage(SampleLiftBucketSubsystem.BucketStates currentState) {
        this.timestamp = System.nanoTime();
        this.currentState = currentState;
    }
}
