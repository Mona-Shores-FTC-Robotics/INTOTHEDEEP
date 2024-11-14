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
        // Retrieve the raw color from the sensor and convert it to SampleColor
        int red = sensor.red();
        int green = sensor.green();
        int blue = sensor.blue();

        // Example logic to determine color, replace with your own thresholds as needed
        if (red > blue && red > green) {
            return SampleColor.RED;
        } else if (blue > red && blue > green) {
            return SampleColor.BLUE;
        } else if (green > red && green > blue) {
            return SampleColor.YELLOW;
        } else {
            return SampleColor.UNKNOWN; // Default if no specific color is detected
        }
    }

    public boolean haveSpecimen() {
        return haveGamePiece();
    }
}
