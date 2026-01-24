package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SpinnerAndShooter extends SubsystemBase{

    private TalonFX SpinMotor;
    private TalonFX ShootMotor;

    private TalonFX TurnMotor;

    public SpinnerAndShooter() {
        SpinMotor = new TalonFX(Constants.Spin.SpinID);
        SpinMotor.setNeutralMode(NeutralModeValue.Brake);

        TurnMotor = new TalonFX(Constants.Spin.TurnerID);
        TurnMotor.setNeutralMode(NeutralModeValue.Brake);

        ShootMotor = new TalonFX(Constants.Spin.ShootID);
        ShootMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void SpinSpeed(double speed){
        SpinMotor.setVoltage(speed);
    }

    public void ShootSpeed(double speed){
        ShootMotor.setVoltage(speed);
    }
    
    public void TurnSpeed(double speed){
      TurnMotor.setVoltage(speed);
    }
}
