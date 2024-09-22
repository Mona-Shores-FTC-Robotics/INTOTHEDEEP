package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;
import org.firstinspires.ftc.teamcode.ObjectClasses.Robot;

//TODO is this necessary now that we extend MecanumDrive and it has its own instantiation of the imu?
public class GyroSubsystem extends SubsystemBase {

    public IMU imu;

    private final RevHubOrientationOnRobot hubOrientation =
            new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, // direction of control hub logo on robot
                    RevHubOrientationOnRobot.UsbFacingDirection.UP); // direction of USB ports on robot

    public double currentAbsoluteYawDegrees;
    public double currentAbsoluteYawRadians;

    public double currentRelativeYawDegrees;
    public double currentRelativeYawRadians;

    private double lastRelativeYawDegrees;

    public double offsetFromAbsoluteYawDegrees;

    public GyroSubsystem() {
    }

    public void init() {
        imu = Robot.getInstance().getDriveSubsystem().getMecanumDrive().lazyImu.get();
    }

    public void periodic() {
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
        updateCurrentRelativeYaw();
    }

    //This method is helpful to synchronize our Gyro and robot pose
    //By resetting the gyro we know that it is 0.
    //Then, we set the relativeYaw to the current pose, which should always be correct
        //At the start of the match, the pose has been preset to one of our four starting locations
        //When we are at the backdrop, our pose is reset to 0, so if we also synchronize it should make sure the gyro and pose are synchronized

    public void synchronizeGyroAndPoseHeading() {
        imu.resetYaw();
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
        currentRelativeYawRadians = Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log();
        currentRelativeYawDegrees = Math.toDegrees(currentRelativeYawRadians);
        offsetFromAbsoluteYawDegrees = currentAbsoluteYawDegrees+currentRelativeYawDegrees;
    }

    public void setRelativeYawTo0() {
        imu.resetYaw();
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
        currentRelativeYawRadians = 0;
        currentRelativeYawDegrees = Math.toDegrees(currentRelativeYawRadians);
        offsetFromAbsoluteYawDegrees = currentAbsoluteYawDegrees+currentRelativeYawDegrees;
    }

    public void updateCurrentRelativeYaw(){
        //They gyro is only updating currentAbsoluteYawDegrees in the periodic()
        double relativeYaw = currentAbsoluteYawDegrees + offsetFromAbsoluteYawDegrees;

        if (relativeYaw>180){
            relativeYaw-=360;
        } else if(relativeYaw <=-180)
        {
            relativeYaw +=360;
        }
            currentRelativeYawDegrees = relativeYaw;
            currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);
    }

    public double getCurrentRelativeYawRadians(){
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);
        updateCurrentRelativeYaw();
        return currentRelativeYawRadians;
    }

    public void DriverStationTelemetry() {
        Robot.getInstance().getActiveOpMode().telemetry.addLine("");
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Abs (Degrees)" + JavaUtil.formatNumber(currentAbsoluteYawDegrees, 6, 0));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Yaw Angle Rel (Degrees)" + JavaUtil.formatNumber(currentRelativeYawDegrees, 6, 0));
        Robot.getInstance().getActiveOpMode().telemetry.addLine("Robot Pose (Degrees)" + JavaUtil.formatNumber(Robot.getInstance().getDriveSubsystem().mecanumDrive.pose.heading.log(), 4, 0));
    }

    public void postAutoGyroReset() {
        //reset gyro
        imu.resetYaw();
        YawPitchRollAngles angle = imu.getRobotYawPitchRollAngles();
        currentAbsoluteYawDegrees = angle.getYaw(AngleUnit.DEGREES);
        currentAbsoluteYawRadians = angle.getYaw(AngleUnit.RADIANS);

        //Calculate new offset
        offsetFromAbsoluteYawDegrees = MatchConfig.endOfAutonomousAbsoluteYawDegrees;

        // Adjust relative yaw
        currentRelativeYawDegrees = MatchConfig.endOfAutonomousRelativeYawDegrees - offsetFromAbsoluteYawDegrees;
        currentRelativeYawRadians = Math.toRadians(currentRelativeYawDegrees);
    }
}
