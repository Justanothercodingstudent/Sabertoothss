package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class climber extends SubsystemBase{

    private TalonFX climber;

    private double ClimbPos;

    public climber(){
        climber = new TalonFX(Constants.climber.climberID);
        climber.setNeutralMode(NeutralModeValue.Brake);

        ClimbPos = 0.5;
    }

    public void climbSpeed(double speed){
        climber.set(speed);
    }

    public void setClimberPosition(double position) {
        climber.setPosition(position);
        clampClimbSetPos();
    }

    public void clampClimbSetPos() {
        ClimbPos = Math.max(
            Constants.climber.minClimb, // Ensure minimum position
            Math.min(Constants.climber.minClimb, ClimbPos) // Ensure maximum position
        );
    }

    public void climbStop(){
        climber.set(0);
    }

    public double getclimberPos(){
        return climber.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climb value", getclimberPos());
    }
    
}
