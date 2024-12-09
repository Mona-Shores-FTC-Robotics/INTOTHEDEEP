package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;
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
                LIGHTING_PARAMS.PROBLEM_COLOR = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
                LIGHTING_PARAMS.HAVE_GAME_PIECE_COLOR = 0.5; //RevBlinkinLedDriver.BlinkinPattern.GREEN;
                LIGHTING_PARAMS.BAD_SAMPLE_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.SHADE_OF_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.RED;
                LIGHTING_PARAMS.SHADE_OF_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.BLUE;
                LIGHTING_PARAMS.SHADE_OF_YELLOW = 0.34; //RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                LIGHTING_PARAMS.SHADE_OF_BLACK = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BLACK;
                LIGHTING_PARAMS.ALLIANCE_NET_BLUE = 0.61; // RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                LIGHTING_PARAMS.ALLIANCE_NET_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_RED = 0.28;//RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                LIGHTING_PARAMS.UNKNOWN_COLOR = 0.0;//RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;

            case INTO_THE_DEEP_20245:
                LIGHTING_PARAMS.PROBLEM_COLOR = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
                LIGHTING_PARAMS.HAVE_GAME_PIECE_COLOR = 0.5; //RevBlinkinLedDriver.BlinkinPattern.GREEN;
                LIGHTING_PARAMS.BAD_SAMPLE_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.BAD_SPECIMEN_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
                LIGHTING_PARAMS.SHADE_OF_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.RED;
                LIGHTING_PARAMS.SHADE_OF_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.BLUE;
                LIGHTING_PARAMS.SHADE_OF_YELLOW = 0.34; //RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                LIGHTING_PARAMS.SHADE_OF_BLACK = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BLACK;
                LIGHTING_PARAMS.ALLIANCE_NET_BLUE = 0.61; // RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
                LIGHTING_PARAMS.ALLIANCE_NET_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
                LIGHTING_PARAMS.ALLIANCE_OBSERVATION_RED = 0.28;//RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
                LIGHTING_PARAMS.UNKNOWN_COLOR = 0.0;//RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                break;


            default:
                throw new IllegalArgumentException("Unknown robot type: " + robotType);
        }
    }

    public static class LightingParams {
        public double PROBLEM_COLOR = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY;
        public double HAVE_GAME_PIECE_COLOR = 0.5; //RevBlinkinLedDriver.BlinkinPattern.GREEN;
        public double BAD_SAMPLE_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        public double BAD_SPECIMEN_WARNING_COLOR = 1.0; //RevBlinkinLedDriver.BlinkinPattern.STROBE_WHITE;
        public double SHADE_OF_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.RED;
        public double SHADE_OF_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.BLUE;
        public double SHADE_OF_YELLOW = 0.34; //RevBlinkinLedDriver.BlinkinPattern.YELLOW;
        public double SHADE_OF_BLACK = 0.0; //RevBlinkinLedDriver.BlinkinPattern.BLACK;
        public double ALLIANCE_NET_BLUE = 0.61; // RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE;
        public double ALLIANCE_NET_RED = 0.28; //RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_RED;
        public double ALLIANCE_OBSERVATION_BLUE = 0.61; //RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
        public double ALLIANCE_OBSERVATION_RED = 0.28;//RevBlinkinLedDriver.BlinkinPattern.DARK_RED;
        public double UNKNOWN_COLOR = 0.0;//RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    }


    private final Servo indicator;
    private SpecimenDetector specimenDetector;
    private SampleDetector sampleDetector;
    private double currentPattern;

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final Robot.RobotType robotType, final String indicatorName) {
        configureParamsForRobotType(robotType);
        indicator = hMap.get(Servo.class, indicatorName);
        currentPattern = 0.0; // Start with no pattern set
    }

    public void init() {
        sampleDetector = Robot.getInstance().getSampleIntakeSubsystem().getSampleDetector();
        specimenDetector = Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector();
    }

    public void setLight(double pattern) {
        if (currentPattern != pattern) {
            currentPattern = pattern;
            indicator.setPosition(pattern);
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

        double lightPattern;

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
