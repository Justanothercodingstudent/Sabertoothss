package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.security.Key;
import java.security.KeyPair;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class SpinnerAndShooter extends SubsystemBase{

    private TalonFX SpinMotor;

    private TalonFX LeftFront;
    private TalonFX LeftBack;

    private TalonFX RightFront;
    private TalonFX RightBack;

    private TalonFX LeftRoller;
    private TalonFX RightRoller;
   
    


    public SpinnerAndShooter() {
        SpinMotor = new TalonFX(Constants.Spin.LeftSpinID);
        SpinMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftFront = new TalonFX(Constants.Spin.LeftShootID);
        LeftFront.setNeutralMode(NeutralModeValue.Brake);

        LeftBack = new TalonFX(Constants.Spin.LeftBackID);
        LeftBack.setNeutralMode(NeutralModeValue.Brake);

        RightFront = new TalonFX(Constants.Spin.RightShootID);
        RightFront.setNeutralMode(NeutralModeValue.Brake);

        RightBack = new TalonFX(Constants.Spin.RightBackID);
        RightBack.setNeutralMode(NeutralModeValue.Brake);

        LeftRoller = new TalonFX(Constants.Spin.LeftRollerID );
        LeftRoller.setNeutralMode( NeutralModeValue.Brake); 

        RightRoller = new TalonFX(Constants.Spin.RightRollerID );
        RightRoller.setNeutralMode( NeutralModeValue.Brake); 
    }


     public double FrontLeftRPM(){
        return LeftFront.getVelocity().getValueAsDouble();
     }

     public double BackLeftRPM(){
        return LeftBack.getVelocity().getValueAsDouble();
     }

     public double FrontRightRPM(){
        return RightFront.getVelocity().getValueAsDouble();
     }

     public double BackRightRPM(){
        return RightBack.getVelocity().getValueAsDouble();
     }

     public double TotalRPM(){
        return FrontLeftRPM() + FrontRightRPM() + BackLeftRPM() + BackRightRPM();
     }

    public void SpinSpeed(double speed){
        SpinMotor.set(-speed);
        SpinMotor.set(speed);
    }

    public void ShootSpeed(double speed){
        LeftFront.set(speed);
        RightFront.set(-speed);

        LeftBack.set(speed);
        RightBack.set(-speed);
    }
    

    public void roller(double speed){
        LeftRoller.set(speed);
        RightRoller.set(-speed);
    }

    @Override
    public void periodic() {

        ShootSpeed(Constants.Spin.CoastSpeed);

        SmartDashboard.putNumber("LeftFront Shoot Speed", FrontLeftRPM());
        SmartDashboard.putNumber("LeftBack Shoot Speed", BackLeftRPM());
        SmartDashboard.putNumber("RightFront Shoot Speed", FrontRightRPM());
        SmartDashboard.putNumber("RightBack Shoot Speed", BackRightRPM());

        SmartDashboard.putNumber("Total Shoot Speed", TotalRPM());

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
   