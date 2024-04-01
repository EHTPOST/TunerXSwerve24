package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    TalonFX intake;
    DutyCycleOut voltage;

    public Intake(int intakeID){
        intake = new TalonFX(4);
        voltage = new DutyCycleOut(0);
    
        // //config
        Constants.MechConstants.configureCTREIntake();
        intake.getConfigurator().apply(Constants.MechConstants.intakeMotorConfig);
    }
    
    public Command takeIn(){
        voltage.Output = Constants.MechConstants.intakeTakeIn;
        return this.run(() -> intake.set(.8));
    }

    public Command takeInShooter(){
        voltage.Output = Constants.MechConstants.intakeTakeIn *-1;
        return this.run(() -> intake.setControl(voltage));
    }

    public Command brake(){
        voltage.Output = 0;
        return this.run(() -> intake.setControl(voltage));
    }
}
