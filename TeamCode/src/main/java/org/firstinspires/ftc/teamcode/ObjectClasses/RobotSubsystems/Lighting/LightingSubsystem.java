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
    }

    public static LightingSubsystem.LightingParams LIGHTING_PARAMS = new LightingSubsystem.LightingParams();

    private final RevBlinkinLedDriver blinkinLeft;
    private final RevBlinkinLedDriver blinkinRight;
    private SpecimenDetector specimenDetector;
    private SampleDetector sampleDetector;

    boolean haveSpecimen;
    boolean haveSample;

    boolean haveTooManyGamePieces=false;
    boolean stillHaveBadSample=false;

    private ElapsedTime tooManyPiecesWarningTimer = new ElapsedTime();
    private ElapsedTime badSampleWarningTimer = new ElapsedTime();

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final String leftLights, final String rightLights) {
        blinkinLeft = hMap.get(RevBlinkinLedDriver.class, leftLights);
        blinkinRight = hMap.get(RevBlinkinLedDriver.class, rightLights);
    }

    // Initialize lighting system
    public void init() {
       sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
       specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
       haveTooManyGamePieces=false;
    }

    @Override
    public void periodic() {
        if (specimenDetector != null) {
            haveSpecimen = specimenDetector.haveSpecimen();
        } else {
            haveSpecimen = false;
        }
        if (sampleDetector != null) {
            haveSample = sampleDetector.haveSample();
        } else {
            haveSample = false;
        }

        //Check for too many game pieces
        if (haveSpecimen && haveSample) {
            if (! haveTooManyGamePieces) {
                haveTooManyGamePieces = true;
                tooManyPiecesWarningTimer.reset();
            }
            if (tooManyPiecesWarningTimer.seconds() < LIGHTING_PARAMS.WARNING_DURATION_SECONDS) {
                setTooManyPiecesWarningColor();
            } else setTooManyPiecesProblemColor();
        } else {
            haveTooManyGamePieces = false;
            if (haveSpecimen) {
                //If we have a specimen assume its the right color
                setAllianceColor();
            } else if (haveSample) {
                if (sampleDetector.isGoodSample()) {
                    //If we have a good sample set the light to that color
                    setBothLightsSampleColor();
                } else if (sampleDetector.isBadSample()) {
                    //If we have a bad sample, set the warning/problem light pattern
                    if (! stillHaveBadSample) {
                        stillHaveBadSample = true;
                        badSampleWarningTimer.reset();
                    }
                    if (badSampleWarningTimer.seconds() < LIGHTING_PARAMS.WARNING_DURATION_SECONDS) {
                        setBadSampleWarningColor();
                    } else setBadSampleProblemColor();
                }
            } else {
                stillHaveBadSample = false;
                setBothLightsBlack();
            }
        }
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
        blinkinLeft.setPattern(pattern);
    }
    public void setRightLight(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinRight.setPattern(pattern);
    }
    public void setBothLights(RevBlinkinLedDriver.BlinkinPattern pattern) {
        blinkinLeft.setPattern(pattern);
        blinkinRight.setPattern(pattern);
    }

}
