package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.pathplanner.lib.util.*;
import edu.wpi.first.math.util.Units;


public class Constants {
    
    public class SwerveConstants{

        public static final double maxSpeed = 4;
        public static final double baseY = Units.inchesToMeters(26); 
        public static final double baseX = Units.inchesToMeters(21 + (7/8));
        public static final HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0), 
            maxSpeed,
            Math.sqrt((baseX * baseX) + (baseY * baseY)),
            new ReplanningConfig()
        );
    }

    public class MechConstants{
        //motorIDs
        public static final int shooterMotor1 = 1;
        public static final int shooterMotor2 = 2;
        public static final int intakeMotor = 3;
        public static final int feederMotor = 4;
        public static final int climbMotor1 = 5;
        public static final int climbMotor2 = 6;

        //shooter motor percent output
        public static final double at60InFromSpeaker1 = .88;
        public static final double at60InFromSpeaker2 = .132;
        public static final double at37InFromSpeaker1 = .7; 
        public static final double at37InFromSpeaker2 = .5;
        public static final double takeInShooter = -.2; 
        public static final double atAmpMotor1 = .095; 
        public static final double atAmpMotor2 = .245; 
        public static final double ferryMotor1 = 1;
        public static final double ferryMotor2 = .2;

        //Other motor constants
        public static final double intakeTakeIn = .4;

        //limelight constants
        public static final double speakerAprilHeightInches = 53.88;
        public static final double shootLLHeightInches = 15.5; //check
        public static final double shootLLAngleDegrees = 30; //check

        //configs
        public static final TalonFXConfiguration shootMotor1Config = new TalonFXConfiguration();
        public static final TalonFXConfiguration shootMotor2Config = new TalonFXConfiguration();
        public static void configureCTREShoot(){
            //ramp ups
            shootMotor1Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .25;
            shootMotor2Config.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = .25;
            shootMotor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = .25;
            shootMotor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = .25;

            shootMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            shootMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            shootMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            shootMotor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public static final TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration feederMotorConfig = new TalonFXConfiguration();
        public static void configureCTREIntake(){
            intakeMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            feederMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

            intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            feederMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        }

        public static final TalonFXConfiguration climberMotorConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration climberMotorConfig2 = new TalonFXConfiguration();

        public static double climbMotorOffset = 0.261230;
        public static double climbForwardLimit = 0 - climbMotorOffset;
        public static double climbForwardLimit2 = 0 - climbMotorOffset;        
        public static double climbReverseLimit = -69.050293 + climbMotorOffset;
        public static double climbReverseLimit2 = -69.050293 + climbMotorOffset;
        public static double climbSpeed = 0.35;

        public static void configureCTREClimber(boolean left){
            climberMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            if(left){
                feederMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            }
            else{
                feederMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            }
        }
    }
}