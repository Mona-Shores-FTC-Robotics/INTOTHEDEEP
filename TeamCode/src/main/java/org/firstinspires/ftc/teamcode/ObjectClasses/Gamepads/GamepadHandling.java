package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static com.example.sharedconstants.FieldConstants.*;

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getNextRobotType;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getPreviousRobotType;

import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

public class GamepadHandling {
    private double DEAD_ZONE = .1;

    private GamepadEx driverGamepad;
    private GamepadEx operatorGamepad;

    public boolean LockedSettingsFlag = false;


    public boolean ManualOverrideInitSettingsFlag = false;

    private Gamepad.RumbleEffect endGameRumbleEffect;
    private Gamepad.RumbleEffect problemRumbleEffect;
    private Gamepad.LedEffect problemLedEffect;

    private int timeoutRumbleCounter;

    public GamepadHandling(LinearOpMode opMode) {
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        //Set Driver Gamepad to Blue
        opMode.gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);

        //Set Operator Gamepad to White
        opMode.gamepad2.setLedColor(1, 1, 1, LED_DURATION_CONTINUOUS);

        CreateRumbleEffects();
        CreateLEDEffects();
    }

    private void CreateLEDEffects() {
        problemLedEffect = new Gamepad.LedEffect.Builder()
                .addStep(0, 1, 0, 500) // Show green for 250ms
                .addStep(0, 0, 0, 500) // Show white for 250ms
                .addStep(0, 1, 0, LED_DURATION_CONTINUOUS) // Show white for 250ms
                .build();
    }

    private void CreateRumbleEffects() {
        endGameRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                .build();

        problemRumbleEffect = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble both motors 100% for 500 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
                .addStep(.5, .5, 250)  //  Rumble both motors 50% for 250 mSec
                .addStep(0.0, 0.0, 1000)  //  Pause for 1 Sec
                .build();

        //set the rumble counter to 0
        timeoutRumbleCounter = 0;
    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }

    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }
    public void SelectAndLockColorAndSideAndRobotType() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");

        if (LockedSettingsFlag) {
            telemetry.addLine("Settings Locked");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("Press B to unlock all settings");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;
            }
        } else {
            telemetry.addLine("Lock settings with B");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            // Allow selection of alliance color
            telemetry.addLine("Color (DPAD-UP/DOWN)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
            }

            // Allow selection of side of field based on alliance color
            telemetry.addLine("Side Of Field (DPAD-LEFT/RIGHT)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            }

            // Allow selection of robot type using bumpers
            telemetry.addLine("Robot Type (Left/Right Bumper)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                finalRobotType = getPreviousRobotType(finalRobotType);
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                finalRobotType = getNextRobotType(finalRobotType);
            }
        }
    }

    public void SelectAndLockColorAndSide() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");

        if (LockedSettingsFlag) {
            telemetry.addLine("Alliance Color and Side of Field Locked");
            telemetry.addLine(finalAllianceColor + " " + finalSideOfField);
            telemetry.addLine("Press B to unlock Alliance Color and Side of Field");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;
            }
        } else {
            telemetry.addLine("Lock Alliance Color and Side of Field with B");
            telemetry.addLine(finalAllianceColor + " " + finalSideOfField);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            telemetry.addLine("Color (DPAD-UP/DOWN) - Side Of Field (DPAD-LEFT/RIGHT)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
            }

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                if (finalAllianceColor == AllianceColor.RED) {
                    finalSideOfField = SideOfField.NET;
                } else if (finalAllianceColor == AllianceColor.BLUE) {
                    finalSideOfField = SideOfField.OBSERVATION;
                }
            }
        }
    }






}
