package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling;

import static com.example.sharedconstants.FieldConstants.SampleColor;
import android.graphics.Color;

import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.GamePieceDetector;

public class SampleDetector extends GamePieceDetector {
    private final double proximityThreshold;
    private final int colorHistorySize;

    public SampleDetector(RevColorSensorV3 sensor, double proximityThreshold, int colorHistorySize) {
        super(sensor, colorHistorySize);
        this.proximityThreshold = proximityThreshold;
        this.colorHistorySize = colorHistorySize;
    }

    @Override
    protected double getProximityThreshold() {
        return proximityThreshold;
    }

    @Override
    protected int getProximityHistorySize() {
        return colorHistorySize;
    }

    @Override
    protected int getColorHistorySize() {
        return colorHistorySize;
    }

    @Override
    public SampleColor getRawDetectedColor() {
        float[] hsvValues = new float[3];
        Color.RGBToHSV(sensor.red(), sensor.green(), sensor.blue(), hsvValues);

        int hue = Math.round(hsvValues[0]);
        if (hue >= 10 && hue <= 35) {
            return SampleColor.RED;
        } else if (hue >= 190 && hue <= 250) {
            return SampleColor.BLUE;
        } else if (hue >= 70 && hue <= 120) {
            return SampleColor.YELLOW;
        }
        return SampleColor.UNKNOWN;
    }

    public boolean isGoodSample() {
        return (consensusColor == SampleColor.YELLOW) ||
                (consensusColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED) ||
                (consensusColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE);
    }

    public boolean isBadSample() {
        return (consensusColor == SampleColor.RED && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) ||
                (consensusColor == SampleColor.BLUE && MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED);
    }

    public boolean haveSample() {
        return haveGamePiece();
    }


    public boolean isUnknown() {
        return consensusColor==SampleColor.UNKNOWN;
    }

    @Override
    public DetectionState updateDetection() {
        boolean proximityStable = updateProximity();
        boolean colorStable = updateColor();

        // Detection is valid if proximity is stable and, if required, color is stable
        if (proximityStable && colorStable) {
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


}