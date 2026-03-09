package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.security.Key;
import java.security.KeyPair;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SpinnerAndShooter extends SubsystemBase{

    private TalonFX UptakeMotor;

    private TalonFX LeftFront;
    private TalonFX LeftBack;

    private TalonFX RightFront;
    private TalonFX RightBack;

    private TalonFX BackRoll;
    private TalonFX FrontRoll;
   
    


    public SpinnerAndShooter() {
        UptakeMotor = new TalonFX(Constants.Spin.UptakeID);
        UptakeMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftFront = new TalonFX(Constants.Spin.LeftFrontID);
        LeftFront.setNeutralMode(NeutralModeValue.Coast);

        LeftBack = new TalonFX(Constants.Spin.LeftBackID);
        LeftBack.setNeutralMode(NeutralModeValue.Coast);

        RightFront = new TalonFX(Constants.Spin.RightFrontID);
        RightFront.setNeutralMode(NeutralModeValue.Coast);

        RightBack = new TalonFX(Constants.Spin.RightBackID);
        RightBack.setNeutralMode(NeutralModeValue.Coast);

        BackRoll = new TalonFX(Constants.Spin.BackRollID );
        BackRoll.setNeutralMode( NeutralModeValue.Brake); 

        FrontRoll = new TalonFX(Constants.Spin.FrontRollID );
        FrontRoll.setNeutralMode( NeutralModeValue.Brake); 
    }

     public double FrontLeftRPM(){
        return LeftFront.getVelocity().getValueAsDouble();
     }

    public void SpinSpeed(double speed){
        UptakeMotor.set(speed);
    }

    public void ShootSpeed(double speed){
        LeftFront.set(-speed);
        RightFront.set(speed);

        LeftBack.set(-speed);
        RightBack.set(speed);
    }
    

    public void roller(double speed){
        BackRoll.set(speed);
        FrontRoll.set(speed);
    }

    @Override
    public void periodic() {

        

        SmartDashboard.putNumber("LeftFront Shoot Speed", FrontLeftRPM());

       /*SmartDashboard.putNumber("Left Front Shoot Volts", LeftFront.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Front Shoot Volts", RightFront.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Front Stator", LeftFront.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Front Stator", RightFront.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Front Supply", LeftFront.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Front Supply", RightFront.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Front Stall", LeftFront.getMotorStallCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Front Stall", RightFront.getMotorStallCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Back Shoot Volts", LeftBack.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Right Back Shoot Volts", RightBack.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber("Left Back Stator", LeftBack.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Back Stator", RightBack.getStatorCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Back Supply", LeftBack.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Back Supply", RightBack.getSupplyCurrent().getValueAsDouble());

        SmartDashboard.putNumber("Left Back Stall", LeftBack.getMotorStallCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Right Back Stall", RightBack.getMotorStallCurrent().getValueAsDouble());*/
        
    }
}
   