package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleDetector;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenDetector;

public class LightingSubsystem extends SubsystemBase{

    public static LightingParams LIGHTING_PARAMS = new LightingParams();

    public static void configureParamsForRobotType(Robot.RobotType robotType) {
        switch (robotType) {
            case INTO_THE_DEEP_19429:
                LIGHTING_PARAMS.HAVE_GAME_PIECE_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;
                LIGHTING_PARAMS.BAD_SAMPLE_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.SHADE_OF_RED = RevBlinkinLedDriver.BlinkinPattern.RED;
                LIGHTING_PARAMS.SHADE_OF_BLUE = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                LIGHTING_PARAMS.SHADE_OF_YELLOW = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                LIGHTING_PARAMS.SHADE_OF_BLACK = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                LIGHTING_PARAMS.ALLIANCE_NET_BLUE = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                LIGHTING_PARAMS.ALLIANCE_NET_RED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_BLUE = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_RED = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                LIGHTING_PARAMS.UNKNOWN_COLOR = RevBlinkinLedDriver.BlinkinPattern.GRAY;
                break;

            case INTO_THE_DEEP_20245:
                LIGHTING_PARAMS.HAVE_GAME_PIECE_COLOR = RevBlinkinLedDriver.BlinkinPattern.LIME;
                LIGHTING_PARAMS.BAD_SAMPLE_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.SHADE_OF_RED = RevBlinkinLedDriver.BlinkinPattern.RED;
                LIGHTING_PARAMS.SHADE_OF_BLUE = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                LIGHTING_PARAMS.SHADE_OF_YELLOW = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                LIGHTING_PARAMS.SHADE_OF_BLACK = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                LIGHTING_PARAMS.ALLIANCE_NET_BLUE = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                LIGHTING_PARAMS.ALLIANCE_NET_RED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_BLUE = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_RED = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                LIGHTING_PARAMS.UNKNOWN_COLOR = RevBlinkinLedDriver.BlinkinPattern.GRAY;
                break;


            default:
                throw new IllegalArgumentException("Unknown robot type: " + robotType);
        }
    }

    public static class LightingParams {
        public RevBlinkinLedDriver.BlinkinPattern PROBLEM_COLOR = RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        public RevBlinkinLedDriver.BlinkinPattern HAVE_GAME_PIECE_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;
        public RevBlinkinLedDriver.BlinkinPattern BAD_SAMPLE_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        public RevBlinkinLedDriver.BlinkinPattern BAD_SPECIMEN_WARNING_COLOR = RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        public RevBlinkinLedDriver.BlinkinPattern SHADE_OF_RED = RevBlinkinLedDriver.BlinkinPattern.RED;
        public RevBlinkinLedDriver.BlinkinPattern SHADE_OF_BLUE = RevBlinkinLedDriver.BlinkinPattern.BLUE;
        public RevBlinkinLedDriver.BlinkinPattern SHADE_OF_YELLOW = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        public RevBlinkinLedDriver.BlinkinPattern SHADE_OF_BLACK = RevBlinkinLedDriver.BlinkinPattern.BLACK;
        public RevBlinkinLedDriver.BlinkinPattern ALLIANCE_NET_BLUE = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
        public RevBlinkinLedDriver.BlinkinPattern ALLIANCE_NET_RED = RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        public RevBlinkinLedDriver.BlinkinPattern ALLIANCE_OBSERVATION_BLUE = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        public RevBlinkinLedDriver.BlinkinPattern ALLIANCE_OBSERVATION_RED = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        public RevBlinkinLedDriver.BlinkinPattern UNKNOWN_COLOR = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    }


    private final RevBlinkinLedDriver blinkin;
    private SpecimenDetector specimenDetector;
    private SampleDetector sampleDetector;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final Robot.RobotType robotType, final String blinkinName) {
        configureParamsForRobotType(robotType);
        blinkin = hMap.get(RevBlinkinLedDriver.class, blinkinName);
        currentPattern = null; // Start with no pattern set
    }

    public void init() {
        sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
        specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
    }

    public void setLight(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (currentPattern != pattern) {
            currentPattern = pattern;
            blinkin.setPattern(pattern);
        }
    }

    public void setGoodSampleIndicator() {
        setLight(LIGHTING_PARAMS.HAVE_GAME_PIECE_COLOR);
    }

    public void setLightBlack() {
        setLight(LIGHTING_PARAMS.SHADE_OF_BLACK);
    }


    public void setBadSampleWarningColor() {
        setLight(LIGHTING_PARAMS.BAD_SAMPLE_WARNING_COLOR);
    }


    public void setProblemColor() {
        setLight(LIGHTING_PARAMS.PROBLEM_COLOR);

    }

    public void setBadSpecimenWarningColor() {
        setLight(LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR);
    }

    public void setAllianceColor() {
        switch (MatchConfig.finalAllianceColor) {
            case RED:
                setLight(LIGHTING_PARAMS.SHADE_OF_RED);
                break;
            case BLUE:
                setLight(LIGHTING_PARAMS.SHADE_OF_BLUE);
                break;
        }
    }

    public void setLightToSampleColor() {
        FieldConstants.SampleColor sampleColor = sampleDetector.getConsensusColor();
        switch (sampleColor) {
            case RED:
                setLight(LIGHTING_PARAMS.SHADE_OF_RED);
                break;
            case BLUE:
                setLight(LIGHTING_PARAMS.SHADE_OF_BLUE);
                break;
            case YELLOW:
                setLight(LIGHTING_PARAMS.SHADE_OF_YELLOW);
                break;
            default:
                setLight(LIGHTING_PARAMS.SHADE_OF_BLACK);
                break;
        }
    }

    public void setLightToSpecimenColor() {
        FieldConstants.SampleColor sampleColor = specimenDetector.getConsensusColor();
        if ((MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE && sampleColor == FieldConstants.SampleColor.BLUE) ||
                (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.RED && sampleColor == FieldConstants.SampleColor.RED)) {
            switch (sampleColor) {
                case RED:
                    setLight(LIGHTING_PARAMS.SHADE_OF_RED);
                    break;
                case BLUE:
                    setLight(LIGHTING_PARAMS.SHADE_OF_BLUE);
                    break;
            }
        } else {
            setBadSpecimenWarningColor();
        }
    }



    public void updateLightBasedOnPreloadPresenceAndAllianceColorAndSideOfField(FieldConstants.AllianceColor finalAllianceColor, FieldConstants.SideOfField finalSideOfField) {
        boolean preload = true;
        if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.SPECIMEN_INTAKE)) {
            preload = Robot.getInstance().getSpecimenIntakeSubsystem().checkForPreload();
        }

        RevBlinkinLedDriver.BlinkinPattern lightPattern;

        if (!preload) {
            lightPattern = LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR;
        } else if (finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                lightPattern = LIGHTING_PARAMS.ALLIANCE_NET_BLUE;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                lightPattern = LIGHTING_PARAMS.ALLIANCE_OBSERVATION_BLUE;
            } else {
                lightPattern = LIGHTING_PARAMS.UNKNOWN_COLOR;
            }
        } else if (finalAllianceColor == FieldConstants.AllianceColor.RED) {
            if (finalSideOfField == FieldConstants.SideOfField.NET) {
                lightPattern = LIGHTING_PARAMS.ALLIANCE_NET_RED;
            } else if (finalSideOfField == FieldConstants.SideOfField.OBSERVATION) {
                lightPattern = LIGHTING_PARAMS.ALLIANCE_OBSERVATION_RED;
            } else {
                lightPattern = LIGHTING_PARAMS.UNKNOWN_COLOR;
            }
        } else {
            lightPattern = LIGHTING_PARAMS.UNKNOWN_COLOR;
        }

        setLight(lightPattern);
    }
}
