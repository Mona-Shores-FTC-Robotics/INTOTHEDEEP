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
    public boolean manualOverrideFlag = false;

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

    public int cycleThroughRoutes(List<Routes> routes, int currentIndex) {
        if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
            currentIndex--;
            if (currentIndex < 0) {
                currentIndex = routes.size() - 1;
            }
        } else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.A)) {
            currentIndex++;
            if (currentIndex >= routes.size()) {
                currentIndex = 0;
            }
        }
        return currentIndex;
    }

    public void SelectAllianceAndSide(Telemetry telemetry) {
        if (LockedSettingsFlag) {
            // Display locked state
            telemetry.addData("Alliance Color Locked" , finalAllianceColor);
            telemetry.addData("Side Locked" , finalSideOfField);
            telemetry.addLine("Press B to unlock settings");

            if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = false;  // Unlock settings
            }
        } else {
            // Settings are unlocked: allow manual override, locking, and disabling override
            telemetry.addLine("Settings Override Mode");
            telemetry.addLine("Lock settings with B");
            telemetry.addLine("Disable manual override with X");
            telemetry.addLine();

            // Display whether manual override is active
            telemetry.addData("Manual Override" , manualOverrideFlag ? "Enabled" : "Disabled");

            // Show current settings
            telemetry.addData("Current Alliance Color" , finalAllianceColor);
            telemetry.addData("Current Side of Field" , finalSideOfField);
            telemetry.addLine();

            // Handle locking
            if (operatorGamepad.wasJustPressed(GamepadKeys.Button.B)) {
                LockedSettingsFlag = true;  // Lock settings
            }

            // Allow the driver to disable manual override
            if (operatorGamepad.wasJustPressed(GamepadKeys.Button.X)) {
                manualOverrideFlag = false;  // Revert to sensor-based updates
            }

            // Allow override for alliance color
            telemetry.addLine("Alliance Color (DPAD-UP/DOWN)");
            if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
                finalAllianceColor = AllianceColor.BLUE;
                finalOpponentColor = AllianceColor.RED;
                manualOverrideFlag = true;  // Mark as manually overridden
            } else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                finalAllianceColor = AllianceColor.RED;
                finalOpponentColor = AllianceColor.BLUE;
                manualOverrideFlag = true;  // Mark as manually overridden
            }

            // Allow override for side of the field
            telemetry.addLine("Side of Field (DPAD-LEFT/RIGHT)");
            if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
                finalSideOfField = SideOfField.NET;
                manualOverrideFlag = true;  // Mark as manually overridden
            } else if (operatorGamepad.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                finalSideOfField = SideOfField.OBSERVATION;
                manualOverrideFlag = true;  // Mark as manually
            }
        }
    }
}
