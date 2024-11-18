package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenDetector;

public class LightingSubsystem extends SubsystemBase{


    public static class LightingParams extends SubsystemBase {
        private final double WARNING_DURATION_SECONDS=3.0;
        private final double GREEN_INDICATOR_DURATION = 1.0; // 0.5 seconds

    }

    public static LightingSubsystem.LightingParams LIGHTING_PARAMS = new LightingSubsystem.LightingParams();

    private final RevBlinkinLedDriver blinkinBack;
    private final RevBlinkinLedDriver blinkinFront;
    private SpecimenDetector specimenDetector;
    private SampleDetector sampleDetector;

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final String frontLights, final String backLights) {
        blinkinFront = hMap.get(RevBlinkinLedDriver.class, frontLights);
        blinkinBack = hMap.get(RevBlinkinLedDriver.class, backLights);
    }

    // Initialize lighting system
    public void init() {
       sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
       specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
    }

    public void setGreenIndicatorColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    public void setBadSampleWarningColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
    }

    public void setBadSampleProblemColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    public void setTooManyPiecesWarningColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }

    public void setTooManyPiecesProblemColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    public void setAllianceColor() {
        switch (MatchConfig.finalAllianceColor ) {
            case RED:
                setBothLightsRed();
                break;
            case BLUE:
                setBothLightsBlue();
                break;
        }
    }

    public void setBothLightsSampleColor() {
        FieldConstants.SampleColor sampleColor = sampleDetector.getConsensusColor();
        switch (sampleColor) {
            case RED:
                setBothLightsRed();
                //todo set Score AutoDrive button to go to observation zone?
                break;
            case BLUE:
                setBothLightsBlue();
                //todo  set Score AutoDrive button to go to observation zone?
                break;
            case YELLOW:
                setBothLightsYellow();
                //todo  set Score AutoDrive button to go to basket?
                break;
            default:
                setBothLightsBlack();
                break;
        }
    }

    public void setBothLightsRed() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.RED);
    }
    public void setBothLightsBlue() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.BLUE);
    }
    public void setBothLightsYellow() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
    }
    public void setBothLightsBlack() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setLeftLight(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinBack.setPattern(pattern);
    }
    public void setRightLight(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinFront.setPattern(pattern);
    }
    public void setBothLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinBack.setPattern(pattern);
        blinkinFront.setPattern(pattern);
    }

    public void updateLightsBasedOnAllianceColorAndSide(FieldConstants.AllianceColor finalAllianceColor, FieldConstants.SideOfField finalSideOfField) {
        // Determine preload status
        boolean preload = true;
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            preload = Robot.getInstance().getSpecimenIntakeSubsystem().checkForPreload();
        }

        // Determine light patterns based on preload status
        RevBlinkinLedDriver.BlinkinPattern leftPattern;
        RevBlinkinLedDriver.BlinkinPattern rightPattern;

        if (finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                leftPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
                rightPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLACK : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                rightPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLUE : RevBlinkinLedDriver.BlinkinPattern.STROBE_BLUE;
                leftPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLACK : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY;
            } else {
                leftPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                rightPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
        } else if (finalAllianceColor == FieldConstants.AllianceColor.RED) {
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                leftPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                rightPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLACK : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                rightPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.RED : RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                leftPattern = preload ? RevBlinkinLedDriver.BlinkinPattern.BLACK : RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY;
            } else {
                leftPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                rightPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            }
        } else {
            // Default to both lights off
            leftPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
            rightPattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        }

        // Apply patterns to lights
        setLeftLight(leftPattern);
        setRightLight(rightPattern);
    }
}
