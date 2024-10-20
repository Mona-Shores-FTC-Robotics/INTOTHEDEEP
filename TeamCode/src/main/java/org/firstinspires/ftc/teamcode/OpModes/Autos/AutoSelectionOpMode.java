package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

import java.util.List;

@Autonomous(name = "Auto Selection OP Mode")
public class AutoSelectionOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        // Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        int selectedIndex = 0;
        Routes selectedRoute = null;
        List<Class<? extends Routes>> availableRoutes = null;

        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);

            if (gamepadHandling.LockedSettingsFlag && availableRoutes == null) {
                // Use AutoRouteSelector to get the list of available routes
                AutoRouteSelector autoRouteSelector = new AutoRouteSelector();
                availableRoutes = autoRouteSelector.getAvailableRoutes();

                // Use the gamepadHandling method to cycle through available routes
                selectedIndex = gamepadHandling.cycleThroughOptions(availableRoutes, selectedIndex);

                // Display the currently selected route
                telemetry.addData("Selected Route", availableRoutes.get(selectedIndex).getSimpleName());
            }

            telemetry.update();
            sleep(10);
        }

        if (availableRoutes != null) {
            // Instantiate the RealRobotAdapter after settings are locked
            RealRobotAdapter robotDriveAdapter = new RealRobotAdapter();

// Select the desired route dynamically
            try {
                selectedRoute = availableRoutes.get(selectedIndex).getConstructor(RealRobotAdapter.class).newInstance(robotDriveAdapter);
            } catch (Exception e) {
                telemetry.addData("Error", "Failed to instantiate selected route.");
                telemetry.update();
                return;
            }

            // Build the selected route
            selectedRoute.buildRoute();

            // Pick the route action for the selected side of the field
            Action routeAction = selectedRoute.getRouteAction(MatchConfig.finalSideOfField);

            // Set the starting location of the robot on the field
            Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose = FieldConstants.getStartPose(MatchConfig.finalSideOfField);

            telemetry.clearAll();

            MatchConfig.timestampTimer = new ElapsedTime();
            MatchConfig.timestampTimer.reset();

            // Execute the selected route
            Actions.runBlocking(routeAction);

            Robot.getInstance().getDriveSubsystem().updateInternalIMU();
            MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
            MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
            MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
        } else {
            telemetry.addData("Error", "Available routes were not initialized.");
            telemetry.update();
        }
    }
}
