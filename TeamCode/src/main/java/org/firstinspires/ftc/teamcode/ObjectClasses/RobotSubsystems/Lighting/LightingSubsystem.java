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

    boolean haveSpecimen;
    boolean haveSample;

    boolean haveTooManyGamePieces=false;
    boolean stillHaveBadSample=false;

    private final ElapsedTime tooManyPiecesWarningTimer = new ElapsedTime();
    private final ElapsedTime badSampleWarningTimer = new ElapsedTime();
    // Add flags for warning and problem lights
    private boolean warningLightSet = false;
    private boolean problemLightSet = false;
    private boolean tooManyPiecesWarningSet = false;
    // Add green indicator timer
    private final ElapsedTime greenIndicatorTimer = new ElapsedTime();
    private boolean greenIndicatorActive = false;


    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final String frontLights, final String backLights) {
        blinkinFront = hMap.get(RevBlinkinLedDriver.class, frontLights);
        blinkinBack = hMap.get(RevBlinkinLedDriver.class, backLights);
    }

    // Initialize lighting system
    public void init() {
       sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
       specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
       haveTooManyGamePieces=false;
    }

    @Override
    public void periodic() {
        // Update specimen detection and lighting
        if (specimenDetector != null) {
            switch (specimenDetector.getDetectionState()) {
                case JUST_DETECTED:
                    haveSpecimen = true;
                    greenIndicatorActive = true;
                    greenIndicatorTimer.reset();
                    setGreenIndicatorColor(); // Temporarily set green on new detection
                    warningLightSet = false;
                    problemLightSet = false;
                    tooManyPiecesWarningSet = false;
                    break;
                case STILL_DETECTED:
                    haveSpecimen = true;
                    break;
                case NOT_DETECTED:
                    haveSpecimen = false;
                    break;
            }
        } else {
            haveSpecimen = false;
        }

        // Update sample detection and lighting
        if (sampleDetector != null) {
            switch (sampleDetector.getDetectionState()) {
                case JUST_DETECTED:
                    haveSample = true;
                    greenIndicatorActive = true;
                    greenIndicatorTimer.reset();
                    setGreenIndicatorColor(); // Temporarily set green on new detection
                    warningLightSet = false;
                    problemLightSet = false;
                    tooManyPiecesWarningSet = false;
                    if (sampleDetector.isGoodSample()) {
                        stillHaveBadSample = false;
                    } else if (sampleDetector.isBadSample()) {
                        stillHaveBadSample = true;
                        badSampleWarningTimer.reset();
                    }
                    break;
                case STILL_DETECTED:
                    haveSample = true;
                    break;
                case NOT_DETECTED:
                    haveSample = false;
                    break;
            }
        } else {
            haveSample = false;
        }

        // Handle green indicator duration
        if (greenIndicatorActive && greenIndicatorTimer.seconds() >= LIGHTING_PARAMS.GREEN_INDICATOR_DURATION) {
            greenIndicatorActive = false;
        }

        // Check for too many game pieces condition
        if (haveSpecimen && haveSample) {
            if (!tooManyPiecesWarningSet) {
                tooManyPiecesWarningSet = true;
                tooManyPiecesWarningTimer.reset();
                setTooManyPiecesWarningColor();
            } else if (tooManyPiecesWarningTimer.seconds() >= LIGHTING_PARAMS.WARNING_DURATION_SECONDS) {
                setTooManyPiecesProblemColor();
            }
        } else {
            tooManyPiecesWarningSet = false;
        }

        // Set the final colors based on detection and conditions, if green indicator is not active
        if (!greenIndicatorActive) {
            if (haveSpecimen) {
                setAllianceColor();
            } else if (haveSample) {
                if (sampleDetector.isGoodSample()) {
                    setBothLightsSampleColor();
                } else if (stillHaveBadSample && badSampleWarningTimer.seconds() >= LIGHTING_PARAMS.WARNING_DURATION_SECONDS) {
                    setBadSampleProblemColor();
                } else if (stillHaveBadSample) {
                    setBadSampleWarningColor();
                }
            } else {
                setBothLightsBlack();
            }
        }
    }

    private void setGreenIndicatorColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.GREEN);
    }

    private void setBadSampleWarningColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
    }

    private void setBadSampleProblemColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.WHITE);
    }

    private void setTooManyPiecesWarningColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GRAY);
    }

    private void setTooManyPiecesProblemColor() {
        setBothLights(RevBlinkinLedDriver.BlinkinPattern.GRAY);
    }

    private void setAllianceColor() {
        switch (MatchConfig.finalAllianceColor ) {
            case RED:
                setBothLightsRed();
                break;
            case BLUE:
                setBothLightsBlue();
                break;
        }
    }

    private void setBothLightsSampleColor() {
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

        // Optional telemetry for debugging
        Robot.getInstance().getActiveOpMode().telemetry.addData("Preload Detected", preload);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Alliance Color", finalAllianceColor);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Side of Field", finalSideOfField);
        Robot.getInstance().getActiveOpMode().telemetry.addData("Left Light Pattern", leftPattern.toString());
        Robot.getInstance().getActiveOpMode().telemetry.addData("Right Light Pattern", rightPattern.toString());
        Robot.getInstance().getActiveOpMode().telemetry.update();
    }
}
