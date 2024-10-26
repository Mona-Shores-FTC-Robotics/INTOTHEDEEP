package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Auto Route Selector")
public class AutoSelectionOpMode extends LinearOpMode {
    private RealRobotAdapter robotDriveAdapter;
    private GamepadHandling gamepadHandling;
    private List<Routes> availableRoutes = new ArrayList<>();
    private int selectedIndex = 0;

    @Override
    public void runOpMode() {
        // Initialize command scheduler and gamepad handling
        CommandScheduler.getInstance().reset();
        gamepadHandling = new GamepadHandling(this);
        robotDriveAdapter = new RealRobotAdapter();

        // Define available routes
        availableRoutes.add(new NET_Score_1_Specimen_Preload(robotDriveAdapter));
        availableRoutes.add(new OBS_Score_1_Specimen_Preload(robotDriveAdapter));
        // Add additional routes here as needed

        // Allow cycling through routes in init mode
        while (opModeInInit()) {
            // Cycle through routes using DPAD
            selectedIndex = cycleThroughRoutes(selectedIndex);

            // Show currently selected route on telemetry
            telemetry.addData("Selected Route", availableRoutes.get(selectedIndex).getClass().getSimpleName());
            telemetry.update();
            sleep(100);  // Debounce delay for cycling
        }

        // Set up robot and selected route for auto mode
        Robot.createInstance(this, MatchConfig.finalRobotType);
        Robot.getInstance().init(Robot.OpModeType.AUTO);
        Routes selectedRoute = availableRoutes.get(selectedIndex);
        selectedRoute.buildRoute();

        // Set starting position based on alliance configuration
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose =
                FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        Action routeAction = selectedRoute.getRouteAction(MatchConfig.finalSideOfField);
        Actions.runBlocking(routeAction);

        // Log final position and telemetry after route completion
        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;

        telemetry.addData("Autonomous", "Execution Complete");
        telemetry.addData("End Yaw", MatchConfig.endOfAutonomousAbsoluteYawDegrees);
        telemetry.addData("End Offset", MatchConfig.endOfAutonomousOffset);
        telemetry.addData("End Pose", MatchConfig.endOfAutonomousPose);
        telemetry.update();
    }

    /**
     * Cycles through available routes based on DPAD inputs.
     * @param currentIndex Current route index.
     * @return Updated index after cycling.
     */
    private int cycleThroughRoutes(int currentIndex) {
        if (gamepad1.dpad_left) {
            currentIndex = (currentIndex - 1 + availableRoutes.size()) % availableRoutes.size();
            sleep(200);  // Debounce delay
        } else if (gamepad1.dpad_right) {
            currentIndex = (currentIndex + 1) % availableRoutes.size();
            sleep(200);  // Debounce delay
        }
        return currentIndex;
    }
}
