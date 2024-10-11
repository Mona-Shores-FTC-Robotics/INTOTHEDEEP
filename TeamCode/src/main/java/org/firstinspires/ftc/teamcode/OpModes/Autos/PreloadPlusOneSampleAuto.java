package org.firstinspires.ftc.teamcode.OpModes.Autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.example.sharedconstants.FieldConstants;
import com.example.sharedconstants.Routes.NET_Preload_and_One_Sample;
import com.example.sharedconstants.Routes.OBS_Preload_and_One_Specimen;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ObjectClasses.Gamepads.GamepadHandling;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.RealRobotAdapter;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

@Autonomous(name = "Preload Auto Plus One Sample")
public class PreloadPlusOneSampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Reset the Singleton CommandScheduler and Robot
        CommandScheduler.getInstance().reset();

        //Initialize the Game-pads
        GamepadHandling gamepadHandling = new GamepadHandling(this);

        while (opModeInInit()) {
            // Allow driver to override/lock the vision
            gamepadHandling.getDriverGamepad().readButtons();
            gamepadHandling.SelectAndLockColorAndSideAndRobotType(telemetry);
            telemetry.update();
            sleep(10);
        }

        //TODO: before competition we must hardcode for a single robot and move this to before the init to save precious auto time

        // Create and Initialize the robot
        Robot.createInstance(this, MatchConfig.finalRobotType);

        // Initialize Gamepad and Robot - Order Important
        Robot.getInstance().init(Robot.OpModeType.AUTO);

        //Instantiate the robotDriveAdapter so we can use MeepMeep seamlessly
        RealRobotAdapter robotDriveAdapter = new RealRobotAdapter();

        //Build all the routes using the adapter so we can select one quickly later
        NET_Preload_and_One_Sample preloadAndOneSample = new NET_Preload_and_One_Sample(robotDriveAdapter);
        preloadAndOneSample.BuildRoutes();


        //Pick one of the routes built previously based on the final Alliance Color and Side of Field
        Action selectedRoute = preloadAndOneSample.getRouteAction(MatchConfig.finalSideOfField);

        //set the starting location of the robot on the field
        Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose= FieldConstants.getStartPose(MatchConfig.finalAllianceColor, MatchConfig.finalSideOfField);

        //Reset Gyro
        //TODO i suspect this is not needed or might be duplicative with what is happening in MecanumDrive
        // can we run a test without this and see what happens?
        //Robot.getInstance().getGyroSubsystem().synchronizeGyroAndPoseHeading();

        telemetry.clearAll();

        MatchConfig.timestampTimer = new ElapsedTime();
        MatchConfig.timestampTimer.reset();

        Actions.runBlocking(selectedRoute);
        Robot.getInstance().getDriveSubsystem().updateInternalIMU();
        MatchConfig.endOfAutonomousAbsoluteYawDegrees = Robot.getInstance().getDriveSubsystem().getInternalIMUYawDegrees();
        MatchConfig.endOfAutonomousOffset = Robot.getInstance().getDriveSubsystem().yawOffsetDegrees;
        MatchConfig.endOfAutonomousPose = Robot.getInstance().getDriveSubsystem().getMecanumDrive().pose;
    }
}

