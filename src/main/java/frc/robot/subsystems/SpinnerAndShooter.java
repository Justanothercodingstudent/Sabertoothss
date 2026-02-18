package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SpinnerAndShooter extends SubsystemBase{

    private TalonFX LeftSpinMotor;
    private TalonFX LeftShootMotor;

    private TalonFX RightSpinMotor;
    private TalonFX RightShootMotor;
     
    private TalonFX Roller;

    


    public SpinnerAndShooter() {
        LeftSpinMotor = new TalonFX(Constants.Spin.LeftSpinID);
        LeftSpinMotor.setNeutralMode(NeutralModeValue.Brake);

        RightSpinMotor = new TalonFX(Constants.Spin.RightSpinID);
        RightSpinMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftShootMotor = new TalonFX(Constants.Spin.LeftShootID);
        LeftShootMotor.setNeutralMode(NeutralModeValue.Brake);

        RightShootMotor = new TalonFX(Constants.Spin.RightShootID);
        RightShootMotor.setNeutralMode(NeutralModeValue.Brake);

        Roller = new TalonFX(Constants.Spin.RollerID );
        Roller.setNeutralMode( NeutralModeValue.Brake); 
    }
     


    public void SpinSpeed(double speed){
        LeftSpinMotor.set(-speed);
        RightSpinMotor.set(speed);
    }

    public void ShootSpeed(double speed){
        LeftShootMotor.setVoltage(speed);
        RightShootMotor.setVoltage(-speed);
    }

    public void roller(double speed){
        Roller.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Shoot Volts", LeftShootMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Shoot Volts", RightShootMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Stator", LeftShootMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Stator", RightShootMotor.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Supply", LeftShootMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Supply", RightShootMotor.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Stall", LeftShootMotor.getMotorStallCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Stall", RightShootMotor.getMotorStallCurrent().getValueAsDouble());
    }
}
   