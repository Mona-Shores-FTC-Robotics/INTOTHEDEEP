package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;

public class DriveParams {

    public static void configureChassis19429A(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
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

        if (mecanumDrive instanceof PinpointDrive) {
            MecanumDrive.PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            MecanumDrive.PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            MecanumDrive.PARAMS.inPerTick = 1.0;
            MecanumDrive.PARAMS.lateralInPerTick = 0.7643640655048315;
            MecanumDrive.PARAMS.trackWidthTicks = 12.756341533315288;
            MecanumDrive.PARAMS.kS = 0.8015924760256286;
            MecanumDrive.PARAMS.kV = 0.13660250281616085;
            MecanumDrive.PARAMS.kA = 0.0009;
            MecanumDrive.PARAMS.maxWheelVel = 40;
            MecanumDrive.PARAMS.minProfileAccel = -30;
            MecanumDrive.PARAMS.maxProfileAccel = 40;
            MecanumDrive.PARAMS.maxAngVel = Math.PI;
            MecanumDrive.PARAMS.maxAngAccel = Math.PI;
            MecanumDrive.PARAMS.axialGain = 15.0;
            MecanumDrive.PARAMS.lateralGain = 6.5;
            MecanumDrive.PARAMS.headingGain = 8.0;
            MecanumDrive.PARAMS.axialVelGain = 0.1;
            MecanumDrive.PARAMS.lateralVelGain = 0.0;
            MecanumDrive.PARAMS.headingVelGain = 0.1;
            PinpointDrive.PARAMS.xOffset = 4;
            PinpointDrive.PARAMS.yOffset = 2;
            PinpointDrive.PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_SWINGARM_POD;
            PinpointDrive.PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            PinpointDrive.PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

            mecanumDrive.localizer = mecanumDrive.new DriveLocalizer(); //Is this even necessary?

            PinpointDrive pinpointDrive = (PinpointDrive) mecanumDrive;

            pinpointDrive.pinpoint.setOffsets(DistanceUnit.MM.fromInches(PinpointDrive.PARAMS.xOffset), DistanceUnit.MM.fromInches(PinpointDrive.PARAMS.yOffset));
            pinpointDrive.pinpoint.setEncoderResolution(PinpointDrive.PARAMS.encoderResolution);
            pinpointDrive.pinpoint.setEncoderDirections(PinpointDrive.PARAMS.xDirection, PinpointDrive.PARAMS.yDirection);
        } else
        {
            MecanumDrive.PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            MecanumDrive.PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            MecanumDrive.PARAMS.inPerTick = 0.002996;
            MecanumDrive.PARAMS.lateralInPerTick = 0.0023321641061384555;
            MecanumDrive.PARAMS.trackWidthTicks = 4410.968285794866;
            MecanumDrive.PARAMS.kS = 1.1061809171010473;
            MecanumDrive.PARAMS.kV = 0.0003917967378464592;
            MecanumDrive.PARAMS.kA = 0.00001;
            MecanumDrive.PARAMS.maxWheelVel = 50;
            MecanumDrive.PARAMS.minProfileAccel = -30;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
            MecanumDrive.PARAMS.maxAngVel = Math.PI;
            MecanumDrive.PARAMS.maxAngAccel = Math.PI;
            MecanumDrive.PARAMS.axialGain = 10;
            MecanumDrive.PARAMS.lateralGain = 10;
            MecanumDrive.PARAMS.headingGain = 8;
            MecanumDrive.PARAMS.axialVelGain = 1.5;
            MecanumDrive.PARAMS.lateralVelGain = 0;
            MecanumDrive.PARAMS.headingVelGain = 1.1;
        }

        if (mecanumDrive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.REVERSE);
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.REVERSE);
            TwoDeadWheelLocalizer.PARAMS.parYTicks = -1450.0;
            TwoDeadWheelLocalizer.PARAMS.perpXTicks =  800.0;
        }
    }

    // Static method for configuring motor and encoder directions for CenterStage
    public static void configureCenterStage(MecanumDrive mecanumDrive, DriveSubsystem driveSubsystem) {
        // Set motor directions
        mecanumDrive.leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        mecanumDrive.rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        mecanumDrive.rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set encoder directions
        driveSubsystem.leftFrontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.leftBackEncoder.setDirection(DcMotorEx.Direction.FORWARD);
        driveSubsystem.rightFrontEncoder.setDirection(DcMotorEx.Direction.REVERSE);
        driveSubsystem.rightBackEncoder.setDirection(DcMotorEx.Direction.FORWARD);

        if (mecanumDrive instanceof PinpointDrive) {
            mecanumDrive.localizer = mecanumDrive.new DriveLocalizer();
            MecanumDrive.PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            MecanumDrive.PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            MecanumDrive.PARAMS.inPerTick = 1;
            MecanumDrive.PARAMS.lateralInPerTick = 0.6348918955086741;
            MecanumDrive.PARAMS.trackWidthTicks = 16.373597597387963;
            MecanumDrive.PARAMS.kS = 1.295442485562815;
            MecanumDrive.PARAMS.kV = 0.08935601408956978;
            MecanumDrive.PARAMS.kA = 0.018;
            MecanumDrive.PARAMS.maxWheelVel = 30;
            MecanumDrive.PARAMS.minProfileAccel = -30;
            MecanumDrive.PARAMS.maxProfileAccel = 30;
            MecanumDrive.PARAMS.maxAngVel = Math.PI;
            MecanumDrive.PARAMS.maxAngAccel = Math.PI;
            MecanumDrive.PARAMS.axialGain = 9.6;
            MecanumDrive.PARAMS.lateralGain = 3;
            MecanumDrive.PARAMS.headingGain = 3;
            MecanumDrive.PARAMS.axialVelGain = 0.0;
            MecanumDrive.PARAMS.lateralVelGain = 0.0;
            MecanumDrive.PARAMS.headingVelGain = 0.0;
            PinpointDrive.PARAMS.xOffset = 2;
            PinpointDrive.PARAMS.yOffset = -2;
            PinpointDrive.PARAMS.encoderResolution = GoBildaPinpointDriverRR.goBILDA_4_BAR_POD;
            PinpointDrive.PARAMS.xDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
            PinpointDrive.PARAMS.yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

            PinpointDrive pinpointDrive = (PinpointDrive) mecanumDrive;
            pinpointDrive.pinpoint.setOffsets(DistanceUnit.MM.fromInches(PinpointDrive.PARAMS.xOffset), DistanceUnit.MM.fromInches(PinpointDrive.PARAMS.yOffset));
            pinpointDrive.pinpoint.setEncoderResolution(PinpointDrive.PARAMS.encoderResolution);
            pinpointDrive.pinpoint.setEncoderDirections(PinpointDrive.PARAMS.xDirection, PinpointDrive.PARAMS.yDirection);
        } else {
            //TODO: Needs tuning
            MecanumDrive.PARAMS.logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            MecanumDrive.PARAMS.usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            MecanumDrive.PARAMS.inPerTick = 0.002996;
            MecanumDrive.PARAMS.lateralInPerTick = 0.002996;
            MecanumDrive.PARAMS.trackWidthTicks = 0;
            MecanumDrive.PARAMS.kS = 1.3635356937629455;
            MecanumDrive.PARAMS.kV = 0.00038558904047469167;
            MecanumDrive.PARAMS.kA = 0.0;
            MecanumDrive.PARAMS.maxWheelVel = 50;
            MecanumDrive.PARAMS.minProfileAccel = -30;
            MecanumDrive.PARAMS.maxProfileAccel = 50;
            MecanumDrive.PARAMS.maxAngVel = Math.PI;
            MecanumDrive.PARAMS.maxAngAccel = Math.PI;
            MecanumDrive.PARAMS.axialGain = 0.0;
            MecanumDrive.PARAMS.lateralGain = 0.0;
            MecanumDrive.PARAMS.headingGain = 0.0;
            MecanumDrive.PARAMS.axialVelGain = 0.0;
            MecanumDrive.PARAMS.lateralVelGain = 0.0;
            MecanumDrive.PARAMS.headingVelGain = 0.0;
        }

        if (mecanumDrive.localizer instanceof TwoDeadWheelLocalizer) {
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).par.setDirection(DcMotorEx.Direction.FORWARD);
            ((TwoDeadWheelLocalizer) mecanumDrive.localizer).perp.setDirection(DcMotorEx.Direction.FORWARD);
            TwoDeadWheelLocalizer.PARAMS.parYTicks = -1440.0;
            TwoDeadWheelLocalizer.PARAMS.perpXTicks =  820.0;
        }
    }
}
