package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Drive.Params;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.PinpointDrive;

public class RRParams {

    public static class ChassisTwoDeadWheelInternalIMUParams extends MecanumDrive.Params {
        public ChassisTwoDeadWheelInternalIMUParams() {
            logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
            usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            inPerTick = 0.002996;
            lateralInPerTick = 0.0023321641061384555;
            trackWidthTicks = 4410.968285794866;
            kS = 1.1061809171010473;
            kV = 0.0003917967378464592;
            kA = 0.00001;
            maxWheelVel = 50;
            minProfileAccel = -30;
            maxProfileAccel = 50;
            maxAngVel = Math.PI;
            maxAngAccel = Math.PI;
            axialGain = 10;
            lateralGain = 10;
            headingGain = 8;
            axialVelGain = 1.5;
            lateralVelGain = 0;
            headingVelGain = 1.1;
        }
    }

    public static class CenterStageTwoDeadWheelInternalIMUParams extends MecanumDrive.Params {
        public CenterStageTwoDeadWheelInternalIMUParams() {
            logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            inPerTick = 0.002996;
            lateralInPerTick = inPerTick;
            trackWidthTicks = 0;
            kS = 1.3635356937629455;
            kV = 0.00038558904047469167;
            kA = 0.0;
            maxWheelVel = 50;
            minProfileAccel = -30;
            maxProfileAccel = 50;
            maxAngVel = Math.PI;
            maxAngAccel = Math.PI;
            axialGain = 0.0;
            lateralGain = 0.0;
            headingGain = 0.0;
            axialVelGain = 0.0;
            lateralVelGain = 0.0;
            headingVelGain = 0.0;
        }
    }

    public static class ChassisPinpointParams extends PinpointDrive.Params {
        public ChassisPinpointParams() {
            logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
            usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
            inPerTick = 1.0;
            lateralInPerTick = 0.7643640655048315;
            trackWidthTicks = 12.756341533315288;
            kS = 0.8015924760256286;
            kV = 0.13660250281616085;
            kA = 0.0009;
            maxWheelVel = 40;
            minProfileAccel = -30;
            maxProfileAccel = 40;
            maxAngVel = Math.PI;
            maxAngAccel = Math.PI;
            axialGain = 15.0;
            lateralGain = 6.5;
            headingGain = 8.0;
            axialVelGain = 0.1;
            lateralVelGain = 0.0;
            headingVelGain = 0.1;
        }
    }
}
