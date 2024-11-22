package org.firstinspires.ftc.teamcode.OpModes.Autos;

import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.finalAllianceColor;
import static org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig.finalSideOfField;
import static org.firstinspires.ftc.teamcode.ObjectClasses.Robot.SubsystemType.SPECIMEN_INTAKE;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET.SpecimenPreload.NET_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.OBS.OBS_Score_1_Specimen_Preload;
import com.example.sharedconstants.Routes.Routes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.SpecimenHandling.SpecimenArm.ActionsAndCommands.AutonomousPeriodicAction;

@Autonomous(name = "Auto Test")
public class AutoTest extends LinearOpMode {

    Routes blueObsRoute;
    Routes blueNetRoute;
    Routes redObsRoute;
    Routes redNetRoute;

    @Override
    public void runOpMode() {
        // Create the robot
        Robot.createInstance(this);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        //Instantiate the robotAdapter so we can use MeepMeep seamlessly
        RealRobotAdapter robotAdapter = new RealRobotAdapter();

        //Make the blue routes
        robotAdapter.setAllianceColor(FieldConstants.AllianceColor.BLUE);
        blueObsRoute = new OBS_Score_1_Specimen_Preload(robotAdapter);
        blueObsRoute.buildRoute();
        blueNetRoute = new NET_Score_1_Specimen_Preload(robotAdapter);
        blueNetRoute.buildRoute();

        //Make the red routes
        robotAdapter.setAllianceColor(FieldConstants.AllianceColor.RED);
        redObsRoute = new OBS_Score_1_Specimen_Preload(robotAdapter);
        redObsRoute.buildRoute();
        redNetRoute = new NET_Score_1_Specimen_Preload(robotAdapter);
        redNetRoute.buildRoute();

        //Guess what side of field we are on to make setup easier
        if (Robot.getInstance().getSpecimenIntakeSubsystem().getSpecimenDetector().haveSpecimen())
        {
            finalSideOfField = FieldConstants.SideOfField.OBSERVATION;
        } else finalSideOfField = FieldConstants.SideOfField.NET;

        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.getOperatorGamepad().readButtons();

            // Monitor preload: respects locked state
            if (Robot.getInstance().hasSubsystem(SPECIMEN_INTAKE)) {
                Robot.getInstance()
                        .getSpecimenIntakeSubsystem()
                        .monitorSpecimenPreload(robotAdapter , gamepadHandling.LockedSettingsFlag , gamepadHandling.manualOverrideFlag);
            }

            // Allow driver to override and lock alliance color and side
            gamepadHandling.SelectAllianceAndSide(telemetry);

            //Handle Lighting During Init
            if (Robot.getInstance().hasSubsystem(Robot.SubsystemType.LIGHTING)) {
                Robot.getInstance().getLightingSubsystem().updateLightBasedOnPreloadPresenceAndAllianceColorAndSideOfField(finalAllianceColor , finalSideOfField);
            }

            telemetry.update();
            sleep(10);
        }
        //set the color of our adapter to the final alliance color - this technically shouldn't matter
        robotAdapter.setAllianceColor(MatchConfig.finalAllianceColor);

        //Select route depending on side of field
        Routes route = selectRoute();

        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRoute = route.getRouteAction(MatchConfig.finalSideOfField);

        //set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose= FieldConstants.getStartPose(MatchConfig.finalSideOfField, MatchConfig.finalAllianceColor);

        //Calculate the Yaw offset based on the starting pose and save it in MatchConfig to use during teleop field oriented control
        Robot.getInstance().getDriveSubsystem().CalculateYawOffset();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        AutonomousPeriodicAction autonomousPeriodicAction = new AutonomousPeriodicAction();
        MatchConfig.telemetryPacket = new TelemetryPacket();
        ParallelAction parallelAction = new ParallelAction(
                selectedRoute,
                autonomousPeriodicAction
        );
        MatchConfig.loopTimer = new ElapsedTime();
        MatchConfig.loopTimer.reset();
        Actions.runBlocking(parallelAction);

        MatchConfig.hasAutoRun=true;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }

    public Routes selectRoute() {
        // Select route depending on side of field and alliance color
        Routes route;

        if (MatchConfig.finalSideOfField == FieldConstants.SideOfField.NET) {
            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                // Case 1: Blue alliance on the NET side
                route = blueNetRoute;
            } else {
                // Case 2: Red alliance on the NET side
                route = redNetRoute;
            }
        } else {
            if (MatchConfig.finalAllianceColor == FieldConstants.AllianceColor.BLUE) {
                // Case 3: Blue alliance on the OBS side
                route = blueObsRoute;
            } else {
                // Case 4: Red alliance on the OBS side
                route = redObsRoute;
            }
        }

        return route;
    }
}

