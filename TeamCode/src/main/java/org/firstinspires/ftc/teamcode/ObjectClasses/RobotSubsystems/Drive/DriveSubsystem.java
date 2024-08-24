package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import static com.example.sharedconstants.FieldConstants.BLUE_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.BLUE_BACKSTAGE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_AUDIENCE_START_POSE;
import static com.example.sharedconstants.FieldConstants.RED_BACKSTAGE_START_POSE;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.example.sharedconstants.RobotDriveAdapter;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveClasses.MecanumDriveMona;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionProcessors.InitVisionProcessor;
import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Vision.VisionSubsystem;

@Config
public class DriveSubsystem extends SubsystemBase {

    public MecanumDriveMona mecanumDrive;
    public VisionSubsystem visionSubsystem;
    public boolean aprilTagAutoDriving;
    public boolean fieldOrientedControl;

    public double drive;
    public double strafe;
    public double turn;

    public double leftYAdjusted;
    public double leftXAdjusted;
    public double rightXAdjusted;


    public DriveSubsystem(HardwareMap hardwareMap) {
        //Make the mecanumDrive at pose 0,0,0 because we don't know where we are yet
        mecanumDrive = new MecanumDriveMona(hardwareMap, new Pose2d(0, 0, 0));
    }

    public void init()
    {
        visionSubsystem = Robot.getInstance().getVisionSubsystem();
        aprilTagAutoDriving =false;
        fieldOrientedControl=false;
        mecanumDrive.init();
    }

    public void periodic(){
        DashboardTelemetryDriveTrain();
    }

    private void DashboardTelemetryDriveTrain() {
        //Add code to help with telemetry
    }


    public MecanumDriveMona getMecanumDrive()
    {
       return mecanumDrive;
    }
}


