package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase{

    private TalonFX leftMotor;
    private TalonFX rightMotor;
    TalonFX motor;
    int id;

    public Climber(boolean left){
        if(left){
            id = Constants.MechConstants.climbMotor1;
        }
        else{
            id = Constants.MechConstants.climbMotor2;
        }
        motor = new TalonFX(id);
        Constants.MechConstants.configureCTREClimber(left);
        motor.getConfigurator().apply(Constants.MechConstants.climberMotorConfig);
    }
    
    public Climber(){
        leftMotor = new TalonFX(Constants.MechConstants.climbMotor1);
        rightMotor = new TalonFX(Constants.MechConstants.climbMotor2);

        Constants.MechConstants.climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        Constants.MechConstants.climberMotorConfig2.MotorOutput.NeutralMode = NeutralModeValue.Brake; 

        Constants.MechConstants.climberMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        Constants.MechConstants.climberMotorConfig2.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;


        Constants.MechConstants.climberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        Constants.MechConstants.climberMotorConfig2.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
        Constants.MechConstants.climberMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.MechConstants.climbForwardLimit;
        Constants.MechConstants.climberMotorConfig2.SoftwareLimitSwitch.ForwardSoftLimitThreshold = Constants.MechConstants.climbForwardLimit2;
        
        Constants.MechConstants.climberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Constants.MechConstants.climberMotorConfig2.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        Constants.MechConstants.climberMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.MechConstants.climbReverseLimit;
        Constants.MechConstants.climberMotorConfig2.SoftwareLimitSwitch.ReverseSoftLimitThreshold = Constants.MechConstants.climbReverseLimit2;

        leftMotor.getConfigurator().apply(Constants.MechConstants.climberMotorConfig);
        rightMotor.getConfigurator().apply(Constants.MechConstants.climberMotorConfig2);
    }

    public void climbUp() {
        leftMotor.setControl(new DutyCycleOut(Constants.MechConstants.climbSpeed));
        rightMotor.setControl(new DutyCycleOut(Constants.MechConstants.climbSpeed));
    }

    public void climbDown(){
        leftMotor.setControl(new DutyCycleOut(-Constants.MechConstants.climbSpeed));
        rightMotor.setControl(new DutyCycleOut(-Constants.MechConstants.climbSpeed));
    }

    public void brake(){
        leftMotor.setControl(new DutyCycleOut(0));
        rightMotor.setControl(new DutyCycleOut(0));
    }
}
