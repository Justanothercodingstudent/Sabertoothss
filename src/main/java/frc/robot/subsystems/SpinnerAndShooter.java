package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


import java.security.Key;
import java.security.KeyPair;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;


public class SpinnerAndShooter extends SubsystemBase{
    private static final double DASHBOARD_UPDATE_INTERVAL_SECONDS = 0.10;

    private TalonFX UptakeMotor;

    private TalonFX LeftFront;
    private TalonFX LeftBack;

    private TalonFX RightFront;
    private TalonFX RightBack;

    private TalonFX BackRoll;
    private TalonFX FrontRoll;

    private final VelocityVoltage LeftRequest = new VelocityVoltage(0);
    private final VelocityVoltage RightRequest = new VelocityVoltage(0);
    private double lastDashboardUpdateSeconds = -1.0;
   
    


    public SpinnerAndShooter() {
        UptakeMotor = new TalonFX(Constants.Spin.UptakeID, Constants.CTRE.CANIVORE_NAME);
        UptakeMotor.setNeutralMode(NeutralModeValue.Brake);

        LeftFront = new TalonFX(Constants.Spin.LeftFrontID, Constants.CTRE.CANIVORE_NAME);
        LeftFront.setNeutralMode(NeutralModeValue.Coast);

        LeftBack = new TalonFX(Constants.Spin.LeftBackID, Constants.CTRE.CANIVORE_NAME);
        LeftBack.setNeutralMode(NeutralModeValue.Coast);

        RightFront = new TalonFX(Constants.Spin.RightFrontID, Constants.CTRE.CANIVORE_NAME);
        RightFront.setNeutralMode(NeutralModeValue.Coast);

        RightBack = new TalonFX(Constants.Spin.RightBackID, Constants.CTRE.CANIVORE_NAME);
        RightBack.setNeutralMode(NeutralModeValue.Coast);

        BackRoll = new TalonFX(Constants.Spin.BackRollID, Constants.CTRE.CANIVORE_NAME);
        BackRoll.setNeutralMode( NeutralModeValue.Brake); 

        FrontRoll = new TalonFX(Constants.Spin.FrontRollID, Constants.CTRE.CANIVORE_NAME);
        FrontRoll.setNeutralMode( NeutralModeValue.Brake); 

        TalonFXConfiguration leftConfig = new TalonFXConfiguration();
        leftConfig.MotorOutput.Inverted = Constants.Spin.LeftInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        LeftFront.getConfigurator().apply(leftConfig);

        TalonFXConfiguration rightConfig = new TalonFXConfiguration();
        rightConfig.MotorOutput.Inverted = Constants.Spin.RightInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
        RightFront.getConfigurator().apply(rightConfig);

        

        LeftBack.setControl(new Follower(LeftFront.getDeviceID(), MotorAlignmentValue.Aligned));
        RightBack.setControl(new Follower(RightFront.getDeviceID(), MotorAlignmentValue.Aligned));

        var LeftGains = new Slot0Configs();
        LeftGains.kV = Constants.Spin.LeftSideShooterkV;
        LeftGains.kP = Constants.Spin.LeftSideShooterkP;
        LeftGains.kI = Constants.Spin.LeftSideShooterkI;
        LeftGains.kD = Constants.Spin.LeftSideShooterkD;

        var RightGains = new Slot0Configs();
        RightGains.kV = Constants.Spin.RightSideShooterkV;
        RightGains.kP = Constants.Spin.RightSideShooterkP;
        RightGains.kI = Constants.Spin.RightSideShooterkI;
        RightGains.kD = Constants.Spin.RightSideShooterkD;

        LeftFront.getConfigurator().apply(LeftGains);
        RightFront.getConfigurator().apply(RightGains);
    }

    public void SpinSpeed(double speed){
        UptakeMotor.set(speed);
    }

    // public void ShootSpeed(double speed){
    //     LeftFront.set(-speed);
    //     RightFront.set(speed);

    //     LeftBack.set(-speed);
    //     RightBack.set(speed);
    // }

    public void OpenShootSpeed(double speed){
        LeftFront.set(speed);
        RightFront.set(speed);
    }

    public void setShooterRPS( double LeftRPS, double RightRPS){
        LeftFront.setControl(LeftRequest.withVelocity(LeftRPS));
        RightFront.setControl(RightRequest.withVelocity(RightRPS));
    }

    public void STOPSHOOTER(){
        setShooterRPS(0, 0);
    }

    public void STOPALL(){
        STOPSHOOTER();
        SpinSpeed(0);
        roller(0);
    }

    public double getLeftRPS(){
        return LeftFront.getVelocity().getValueAsDouble();
     }

     public double getRightRPS(){
        return RightFront.getVelocity().getValueAsDouble();
    }

    public boolean isAtSpeed(){
        return Math.abs(getLeftRPS() - Constants.Spin.TestTargetRPS) < Constants.Spin.ShooterToleranceRPS
            && Math.abs(getRightRPS() - Constants.Spin.TestTargetRPS) < Constants.Spin.ShooterToleranceRPS;
    }

    public boolean isAtOrAboveRPS(double requiredRPS) {
        return getLeftRPS() >= requiredRPS && getRightRPS() >= requiredRPS;
    }
    

    public void roller(double speed){
        BackRoll.set(speed);
        FrontRoll.set(speed);
    }

    

    @Override
    public void periodic() {
        double nowSeconds = Timer.getFPGATimestamp();
        if (lastDashboardUpdateSeconds >= 0.0
            && nowSeconds - lastDashboardUpdateSeconds < DASHBOARD_UPDATE_INTERVAL_SECONDS) {
            return;
        }
        lastDashboardUpdateSeconds = nowSeconds;

        double leftRPS = getLeftRPS();
        double rightRPS = getRightRPS();
        boolean atSpeed = Math.abs(leftRPS - Constants.Spin.TestTargetRPS) < Constants.Spin.ShooterToleranceRPS
            && Math.abs(rightRPS - Constants.Spin.TestTargetRPS) < Constants.Spin.ShooterToleranceRPS;

        SmartDashboard.putNumber("Left Shooter RPS", leftRPS);
        SmartDashboard.putNumber("Right Shooter RPS", rightRPS);
        SmartDashboard.putNumber("Target Shooter RPS", Constants.Spin.TestTargetRPS);
        SmartDashboard.putBoolean("Shooter At Speed", atSpeed);

        SmartDashboard.putNumber("Speed of Shooter", Constants.Spin.ShootSpeed);

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
