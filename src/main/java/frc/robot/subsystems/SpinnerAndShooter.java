package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SpinnerAndShooter extends SubsystemBase{

    private TalonFX LeftSpinMotor;
    private TalonFX LeftShootMotor;

    private TalonFX RightSpinMotor;
    private TalonFX RightShootMotor;

    public SpinnerAndShooter() {
        LeftSpinMotor = new TalonFX(Constants.Spin.LeftSpinID);
        LeftSpinMotor.setNeutralMode(NeutralModeValue.Brake);

        RightSpinMotor = new TalonFX(Constants.Spin.RightSpinID);
        RightSpinMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftShootMotor = new TalonFX(Constants.Spin.LeftShootID);
        LeftShootMotor.setNeutralMode(NeutralModeValue.Brake);

        RightShootMotor = new TalonFX(Constants.Spin.RightShootID);
        RightShootMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void SpinSpeed(double speed){
        LeftSpinMotor.setVoltage(speed);
        RightSpinMotor.setVoltage(speed);
    }

    public void ShootSpeed(double speed){
        LeftShootMotor.setVoltage(speed);
        RightShootMotor.setVoltage(speed);
    }
}
