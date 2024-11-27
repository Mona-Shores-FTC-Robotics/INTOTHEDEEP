package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems;

import static com.example.sharedconstants.FieldConstants.SampleColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedList;
import java.util.Queue;

public abstract class GamePieceDetector {

    public enum DetectionState {
        DETECTING,
        NOT_DETECTED,
        JUST_DETECTED,
        STILL_DETECTED,
        SENSOR_DISCONNECTED
    }

    protected final RevColorSensorV3 sensor;
    protected DetectionState detectionState = DetectionState.NOT_DETECTED;
    protected SampleColor consensusColor = SampleColor.UNKNOWN;
    protected double consensusProximity = -1; // Stores the latest stable proximity

    protected final Queue<Double> proximityHistory;
    protected final Queue<SampleColor> colorHistory;

    public GamePieceDetector(RevColorSensorV3 sensor, int historySize) {
        this.sensor = sensor;
        this.proximityHistory = new LinkedList<>();
        this.colorHistory = new LinkedList<>();
    }

    public DetectionState updateDetection() {
        boolean proximityStable = updateProximity();
        boolean colorStable = updateColor();

        // Detection is valid if proximity is stable and, if required, color is stable
        if (proximityStable) {
            if (detectionState == DetectionState.NOT_DETECTED) {
                detectionState = DetectionState.JUST_DETECTED;
            } else {
                detectionState = DetectionState.STILL_DETECTED;
            }
        } else {
            detectionState = DetectionState.NOT_DETECTED;
        }

        return detectionState;
    }


    private boolean updateProximity() {
        double proximity = sensor.getDistance(DistanceUnit.MM);

        if (proximityHistory.size() >= getProximityHistorySize()) {
            proximityHistory.poll();
        }
        proximityHistory.add(proximity);

        if (proximityHistory.size() < getProximityHistorySize()) {
            return false;
        }
        for (double p : proximityHistory) {
            if (p >= getProximityThreshold()) {
                return false;
            }
        }

        // Set consensusProximity if stable
        double sum = 0;
        for (double p : proximityHistory) {
            sum += p;
        }
        consensusProximity = sum / proximityHistory.size();
        return true;
    }

    protected boolean updateColor() {
        SampleColor color = getRawDetectedColor();

        if (colorHistory.size() >= getColorHistorySize()) {
            colorHistory.poll();
        }
        colorHistory.add(color);

        consensusColor = calculateConsensusColor();
        return consensusColor != SampleColor.UNKNOWN;
    }

    protected SampleColor calculateConsensusColor() {
        if (colorHistory.size() < getColorHistorySize()) {
            return SampleColor.UNKNOWN;
        }
        SampleColor firstColor = colorHistory.peek();
        for (SampleColor color : colorHistory) {
            if (color != firstColor) {
                return SampleColor.UNKNOWN;
            }
        }
        return firstColor;
    }

    protected abstract double getProximityThreshold();
    protected abstract int getProximityHistorySize();
    protected abstract int getColorHistorySize();
    protected abstract SampleColor getRawDetectedColor();  // Each subclass defines its specific color detection logic

    public void clearDetectionState() {
        proximityHistory.clear();
        colorHistory.clear();
        consensusColor = SampleColor.UNKNOWN;
        detectionState = DetectionState.NOT_DETECTED;
    }

    public DetectionState getDetectionState() {
        return detectionState;
    }

    public SampleColor getConsensusColor() {
        return consensusColor;
    }

    public double getConsensusProximity() {
        return consensusProximity;
    }

    public Queue<Double> getProximityHistory() {
        return proximityHistory;
    }

    protected boolean haveGamePiece() {
        return (detectionState== DetectionState.JUST_DETECTED || detectionState== DetectionState.STILL_DETECTED);
    }

}
