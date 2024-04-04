package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    TalonFX intake;
    TalonFX feeder;
    DutyCycleOut voltage;

    public Intake(int intakeID, int feederID){
        // intake = new TalonFX(feederID);
        feeder = new TalonFX(intakeID);
        voltage = new DutyCycleOut(0);
    
        Constants.MechConstants.configureCTREIntake();
        // intake.getConfigurator().apply(Constants.MechConstants.intakeMotorConfig);
        feeder.getConfigurator().apply(Constants.MechConstants.feederMotorConfig);

    }
    
    public Command takeIn(){
        voltage.Output = Constants.MechConstants.intakeTakeIn;
        return this.run(() -> {
            // intake.setControl(voltage);
            feeder.setControl(voltage);
        });
    }

    public Command takeInShooter(){
        voltage.Output = Constants.MechConstants.intakeTakeIn *-1;
        return this.run(() -> {
            // intake.setControl(voltage);
            feeder.setControl(voltage);
        });
    }

    public Command brake(){
        voltage.Output = 0;
        return this.run(() -> {
            // intake.setControl(voltage);
            feeder.setControl(voltage);
        });
    }
}
