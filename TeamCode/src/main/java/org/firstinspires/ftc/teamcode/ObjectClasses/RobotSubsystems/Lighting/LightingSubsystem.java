package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Lighting;

import android.annotation.SuppressLint;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.FieldConstants;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SampleHandling.SampleIntake.SampleIntakeSubsystem;
@Config
public class LightingSubsystem {

    public static class LightingParams extends SubsystemBase {



    }

    public static LightingSubsystem.LightingParams LIGHTING_PARAMS = new LightingSubsystem.LightingParams();


    private final RevBlinkinLedDriver blinkinLeft;
    private final RevBlinkinLedDriver blinkinRight;

    // Constructor with color sensor
    public LightingSubsystem(final HardwareMap hMap, final String leftLights, final String rightLights) {
        blinkinLeft = hMap.get(RevBlinkinLedDriver.class, leftLights);
        blinkinRight = hMap.get(RevBlinkinLedDriver.class, rightLights);

    }


    // Initialize lighting system
    public void init() {

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
