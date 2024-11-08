package org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads;

import static com.qualcomm.robotcore.hardware.Gamepad.LED_DURATION_CONTINUOUS;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.example.sharedconstants.RoutesToRun;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.example.sharedconstants.FieldConstants;
import static com.example.sharedconstants.FieldConstants.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.*;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getNextRobotType;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.getPreviousRobotType;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.BindingManagement.GamePadBindingManager;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;
import com.example.sharedconstants.Routes.Routes;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class GamepadHandling {
    private final GamepadEx driverGamepad;
    private final GamepadEx operatorGamepad;
    private final GamePadBindingManager bindingManager;


    public boolean LockedSettingsFlag = false;

    public GamepadHandling(LinearOpMode opMode) {
        bindingManager = new GamePadBindingManager();
        driverGamepad = new GamepadEx(opMode.gamepad1);
        operatorGamepad = new GamepadEx(opMode.gamepad2);

        //Set Driver Gamepad to Blue
        opMode.gamepad1.setLedColor(0, 0, 1, LED_DURATION_CONTINUOUS);

        //Set Operator Gamepad to White
        opMode.gamepad2.setLedColor(1, 1, 1, LED_DURATION_CONTINUOUS);
    }

    public GamepadEx getDriverGamepad() {
        return driverGamepad;
    }
    public GamepadEx getOperatorGamepad() {
        return operatorGamepad;
    }
    public GamePadBindingManager getBindingManager() {
        return bindingManager;
    }

    public void SelectAndLockColorAndSideAndRobotType(Telemetry telemetry) {
        if (LockedSettingsFlag) {
            telemetry.addLine("Settings Locked");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("Press B to unlock settings");

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;  // Unlock settings if B is pressed again
            }
        } else {
            telemetry.addLine("Lock settings with B");
            telemetry.addLine("");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("");

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            // Allow selection of alliance color
            telemetry.addLine("Color (DPAD-UP/DOWN)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
                finalOpponentColor = AllianceColor.RED;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
                finalOpponentColor = AllianceColor.BLUE;
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

    public int cycleThroughRoutes(List<Routes> routes, int currentIndex) {
        if (driverGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            currentIndex--;
            if (currentIndex < 0) {
                currentIndex = routes.size() - 1;
            }
        } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            currentIndex++;
            if (currentIndex >= routes.size()) {
                currentIndex = 0;
            }
        }
        return currentIndex;
    }


    public void SelectAndLockColorAndSide() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        if (LockedSettingsFlag) {
            telemetry.addLine("Settings Locked");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("Press B to unlock settings");

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;  // Unlock settings if B is pressed again
            }
        } else {
            telemetry.addLine("Lock settings with B");
            telemetry.addLine("");
            telemetry.addLine("Alliance: " + finalAllianceColor);
            telemetry.addLine("Side: " + finalSideOfField);
            telemetry.addLine("Robot Type: " + finalRobotType);
            telemetry.addLine("");

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            // Allow selection of alliance color
            telemetry.addLine("Color (DPAD-UP/DOWN)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
                finalOpponentColor = AllianceColor.RED;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
                finalOpponentColor = AllianceColor.BLUE;
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
        }
    }

    public void SelectAndLockColor() {
        Telemetry telemetry = Robot.getInstance().getActiveOpMode().telemetry;
        telemetry.addLine("");

        if (LockedSettingsFlag) {
            telemetry.addData("Alliance Color Locked", finalAllianceColor);
            telemetry.addLine("Press B to unlock Alliance Color");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;
            }
        } else {
            telemetry.addLine("Lock Alliance Color with B");
            telemetry.addLine("Alliance Color: " + finalAllianceColor);

            if (driverGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;
            }

            telemetry.addLine("Color (DPAD-UP/DOWN)");
            if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
                finalOpponentColor = AllianceColor.RED;
            } else if (driverGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
                finalOpponentColor = AllianceColor.BLUE;
            }
        }
    }
}
