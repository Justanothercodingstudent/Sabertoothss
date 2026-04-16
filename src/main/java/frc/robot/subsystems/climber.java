// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.NeutralModeValue;

// import frc.robot.Constants;

// public class climber extends SubsystemBase{

//     private TalonFX climber;
//     private final PositionDutyCycle ClimbRequest = new PositionDutyCycle(0);

//     private double ClimbPos;

//     public climber(){
//         climber = new TalonFX(Constants.climber.climberID);
//         climber.setNeutralMode(NeutralModeValue.Brake);
//         TalonFXConfiguration ClimbConfig = new TalonFXConfiguration();
//         ClimbConfig.Slot0.kP = Constants.climber.climbP;
//         ClimbConfig.Slot0.kI =Constants.climber.climbI;
//         ClimbConfig.Slot0.kD = Constants.climber.climbD;
//         ClimbConfig.MotorOutput. PeakForwardDutyCycle = Constants.climber.ClimbSpeed;
//         ClimbConfig.MotorOutput.PeakReverseDutyCycle = -Constants.climber.ClimbSpeed;
//         ClimbConfig. SoftwareLimitSwitch.ForwardSoftLimitEnable =true;
//         ClimbConfig. SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.climber.maxClimb;
//         ClimbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
//         ClimbConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.climber.minClimb;
//         climber.getConfigurator().apply(ClimbConfig);
//         climber.setNeutralMode(NeutralModeValue.Brake);
//         climber.setPosition(0.0);

//         ClimbPos = Constants.climber.minClimb;
//     }

//     public void ClimbSpeed(double speed){
//         climber.set(speed);
//     }
//     public void setClimberPosition(double position) {
//         ClimbPos = position;
//         clampClimbSetPos();
//     }

//     public void clampClimbSetPos() {
//         ClimbPos = Math.max(
//             Constants.climber.minClimb, // Ensure minimum position
//             Math.min(Constants.climber.maxClimb, ClimbPos) // Ensure maximum position
//         );
//     }

//     public void nextArmPID() {
//         clampClimbSetPos();
//         climber.setControl(ClimbRequest.withPosition(ClimbPos));
//     }

//     public double getclimberPos(){
//         return climber.getPosition().getValueAsDouble();
//     }

//     public void rotateArmMotor(double speed) {
//         climber.set(speed * Constants.climber.ClimbSpeed);
//     }

//     @Override
//     public void periodic(){
//         nextArmPID();
//         SmartDashboard.putNumber("Climb value", getclimberPos());
//         SmartDashboard.putNumber("Climb target", ClimbPos);
//     }

// }
