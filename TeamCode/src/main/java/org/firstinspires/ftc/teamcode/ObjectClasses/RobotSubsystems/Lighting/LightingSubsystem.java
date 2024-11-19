package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting;

import static com.example.sharedconstants.FieldConstants.AllianceColor.BLUE;
import static com.example.sharedconstants.FieldConstants.AllianceColor.RED;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenDetector;

public class LightingSubsystem extends SubsystemBase{

    private final RevBlinkinLedDriver blinkin;
    private SpecimenDetector specimenDetector;
    private SampleDetector sampleDetector;

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final String blinkinName) {
        blinkin = hMap.get(RevBlinkinLedDriver.class, blinkinName);
    }

    // Initialize lighting system
    public void init() {
       sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
       specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
    }

    public void setGreenIndicatorColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setBadSampleWarningColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }

    public void setBadSpecimenWarningColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE);
    }

    public void setBadSampleProblemColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void setTooManyPiecesWarningColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }

    public void setTooManyPiecesProblemColor() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    public void setAllianceColor() {
        switch (MatchConfig.finalAllianceColor ) {
            case RED:
                setLightRed();
                break;
            case BLUE:
                setLightBlue();
                break;
        }
    }

    public void setLightToSampleColor() {
        FieldConstants.SampleColor sampleColor = sampleDetector.getConsensusColor();
        switch (sampleColor) {
            case RED:
                setLightRed();
                break;
            case BLUE:
                setLightBlue();
                break;
            case YELLOW:
                setLightYellow();
                break;
            default:
                setLightBlack();
                break;
        }
    }

    public void setLightToSpecimenColor() {
        FieldConstants.SampleColor sampleColor = specimenDetector.getConsensusColor();

        // Check if the sample color matches the alliance color
        if ((MatchConfig.finalAllianceColor == BLUE && sampleColor == FieldConstants.SampleColor.BLUE) ||
                (MatchConfig.finalAllianceColor == RED && sampleColor == FieldConstants.SampleColor.RED)) {
            // Set light to match the alliance color
            switch (sampleColor) {
                case RED:
                    setLightRed();
                    break;
                case BLUE:
                    setLightBlue();
                    break;
            }
        } else {
            // Flash white light for non-scorable pieces
            setBadSpecimenWarningColor();
        }
    }

    public void setLightRed() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void setLightBlue() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void setLightYellow() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
    public void setLightBlack() {
        setLight(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }


    public void setLight(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkin.setPattern(pattern);
    }

    public void updateLightBasedOnPreloadPresenceAndAllianceColorAndSideOfField(FieldConstants.AllianceColor finalAllianceColor, FieldConstants.SideOfField finalSideOfField) {
        // Determine preload status
        boolean preload = true;
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            preload = Robot.getInstance().getSpecimenIntakeSubsystem().checkForPreload();
        }

        // Determine light pattern
        RevBlinkinLedDriver.BlinkinPattern lightPattern;

        if (!preload) {
            // Blinking white when no preload is detected
            lightPattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        } else if (finalAllianceColor == BLUE) {
            // Blue alliance logic
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
            } else {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; // Problem if we see this
            }
        } else if (finalAllianceColor == RED) {
            // Red alliance logic
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
            } else {
                lightPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK; // Problem if we see this
            }
        } else {
            // Default to lights off if alliance color is unknown
            lightPattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW; // Problem if we see this
        }

        // Apply the light pattern
        setLight(lightPattern);
    }
}
