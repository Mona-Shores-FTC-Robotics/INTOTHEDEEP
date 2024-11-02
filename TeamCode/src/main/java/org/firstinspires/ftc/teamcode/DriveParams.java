package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.DriveSubsystem;

public class DriveParams {
    public static void configureIntoTheDeep19429RRParams() {
        MecanumDrive.PARAMS.logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        MecanumDrive.PARAMS.usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        // drive model parameters
        MecanumDrive.PARAMS.inPerTick = 1;
        MecanumDrive.PARAMS.lateralInPerTick = 0.810201231201328;
        MecanumDrive.PARAMS.trackWidthTicks = 10.120120108884029;

        // feedforward parameters (in tick units)
        MecanumDrive.PARAMS.kS = 0.8731944849469611;
        MecanumDrive.PARAMS.kV = 0.1932813914842022;
        MecanumDrive.PARAMS.kA = 0.0375 ;

        // path profile parameters (in inches)
        MecanumDrive.PARAMS.maxWheelVel = 40;
        MecanumDrive.PARAMS.minProfileAccel = -40;
        MecanumDrive.PARAMS.maxProfileAccel = 40;

        // turn profile parameters (in radians)
        MecanumDrive.PARAMS.maxAngVel = Math.PI; // shared with path
        MecanumDrive.PARAMS.maxAngAccel = Math.PI;

        // path controller gains
        MecanumDrive.PARAMS.axialGain = 3.5;
        MecanumDrive.PARAMS.lateralGain = 1;
        MecanumDrive.PARAMS.headingGain = 6; // shared with turn

        MecanumDrive.PARAMS.axialVelGain = 0;
        MecanumDrive.PARAMS.lateralVelGain = 0;
        MecanumDrive.PARAMS.headingVelGain = 0; // shared with turn

        PinpointDrive.PARAMS.xOffset = 0;
        PinpointDrive.PARAMS.yOffset = 2;
        PinpointDrive.PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;
        PinpointDrive.PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        PinpointDrive.PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    }
    public static void configureIntoTheDeep19429Directions(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        //Set motor encoder directions
        driveSubsystem.leftFrontEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.leftBackEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.rightFrontEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.rightBackEncoder.setDirection(DcMotorEx.Direction.REVERSE);
    }
}

