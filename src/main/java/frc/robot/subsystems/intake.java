package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class intake extends SubsystemBase{

    private TalonFX IntakeMotor;

    private double Intakespeed;

    public intake() {
        IntakeMotor = new TalonFX(Constants.Intake.IntakeID);
        IntakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void IntakeSpeed(double speed){
        IntakeMotor.setVoltage(speed);
    }

}

