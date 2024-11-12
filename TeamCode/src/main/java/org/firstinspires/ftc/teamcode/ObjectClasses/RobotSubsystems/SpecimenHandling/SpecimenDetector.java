package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling;

import static com.example.sharedconstants.FieldConstants.SampleColor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector;

public class SpecimenDetector extends GamePieceDetector {
    private final double proximityThreshold;
    private final int historySize;

    public SpecimenDetector(RevColorSensorV3 sensor, double proximityThreshold, int historySize) {
        super(sensor, historySize);
        this.proximityThreshold = proximityThreshold;
        this.historySize = historySize;
    }

    @Override
    protected double getProximityThreshold() {
        return proximityThreshold;
    }

    @Override
    protected int getProximityHistorySize() {
        return historySize;
    }

    @Override
    protected int getColorHistorySize() {
        return 0; // SpecimenDetector does not use color history
    }

    @Override
    protected SampleColor getRawDetectedColor() {
        return SampleColor.UNKNOWN; // SpecimenDetector does not require color detection
    }

    public boolean haveSpecimen() {
        return haveGamePiece();
    }
}
