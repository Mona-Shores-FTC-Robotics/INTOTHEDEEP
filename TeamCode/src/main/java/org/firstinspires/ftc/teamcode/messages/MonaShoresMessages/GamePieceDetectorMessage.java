package org.firstinspires.ftc.teamcode.messages.MonaShoresMessages;

import com.example.sharedconstants.FieldConstants;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector;

public final class GamePieceDetectorMessage {
    public long timestamp;
    public GamePieceDetector.DetectionState detectionState;
    public double consensusProximity;
    public FieldConstants.SampleColor consensusColor;

    public GamePieceDetectorMessage(GamePieceDetector.DetectionState detectionState, double consensusProximity, FieldConstants.SampleColor consensusColor) {
        this.timestamp = System.nanoTime();
        this.detectionState = detectionState;
        this.consensusProximity = consensusProximity;
        this.consensusColor = consensusColor;
    }
}
