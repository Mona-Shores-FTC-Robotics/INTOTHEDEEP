package org.firstinspires.ftc.teamcode.ObjectClasses.RobotSubsystems.Intake;


import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.ObjectClasses.MatchConfig;

@Config
public class IntakeSubsystem extends SubsystemBase {

    public static class IntakeParameters {
        public double INTAKE_ON_POWER = 1;
        public double INTAKE_REVERSE_POWER = -1;
        public double INTAKE_SLOW_POWER = .2;
        public double INTAKE_OFF_POWER = 0;
    }
    public static IntakeParameters intakeParameters = new IntakeParameters();

    public enum IntakeStates {
        INTAKE_ON (30, 1),
        INTAKE_SLOW(30, .2),
        INTAKE_REVERSE (-150,-1),
        INTAKE_OFF (0, 0);

        public double velocity;
        public double power;

        IntakeStates(double vel, double pow) {
            this.velocity = vel; this.power = pow;
        }
        void SetStateVelocity(double vel){
            this.velocity = vel;
        }
    }

    public DcMotorEx intake1;
    public DcMotorEx intake2;

    public void setCurrentIntake1State(IntakeStates currentIntake1State) {
        this.currentIntake1State = currentIntake1State;
    }

    public void setCurrentIntake2State(IntakeStates currentIntake2State) {
        this.currentIntake2State = currentIntake2State;
    }

    public IntakeStates currentIntake1State;
    public IntakeStates currentIntake2State;

    public IntakeSubsystem(final HardwareMap hMap, final String name, final String name2){
        intake1 = hMap.get(DcMotorEx.class, name);
        intake2 = hMap.get(DcMotorEx.class, name2);
    }

    public void init() {
        intake1.setDirection(DcMotor.Direction.FORWARD);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentIntake1State = IntakeStates.INTAKE_OFF;
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setPower(0);

        intake2.setDirection(DcMotor.Direction.REVERSE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        currentIntake2State = IntakeStates.INTAKE_OFF;
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setPower(0);
    }

    @Override
    public void periodic(){
       IntakeStates.INTAKE_ON.SetStateVelocity(intakeParameters.INTAKE_ON_POWER);
       IntakeStates.INTAKE_SLOW.SetStateVelocity(intakeParameters.INTAKE_SLOW_POWER);
       IntakeStates.INTAKE_REVERSE.SetStateVelocity(intakeParameters.INTAKE_REVERSE_POWER);

       //Add the Winch Motor State to our loop telemetry packet
       MatchConfig.telemetryPacket.put("Current Intake 1 State", currentIntake1State);
       MatchConfig.telemetryPacket.put("Current Intake 2 State", currentIntake2State);
    }
}
