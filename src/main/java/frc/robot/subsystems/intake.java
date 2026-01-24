package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class intake extends SubsystemBase{

    private TalonFX IntakeMotor;

    public intake() {
        IntakeMotor = new TalonFX(Constants.Intake.IntakeID);
        IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void IntakeSpeed(double speed){
        IntakeMotor.setVoltage(speed);
    }

}

