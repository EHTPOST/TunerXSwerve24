package frc.robot;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.*;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.*;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;


public class Constants {
    
    public class SwerveConstants{

        public static final double maxAngularVelocity = 2.5 * Math.PI;
        public static final double maxSpeed = 4;
        public static final double maxAccel = 3;
        public static final double maxAngularAccel = maxAngularVelocity * 2;
        public static final double stickDeadband = 0.1;
        public static final int pigeonID = 0;
        public static final double baseY = Units.inchesToMeters(26); 
        public static final double baseX = Units.inchesToMeters(21 + (7/8));
        public static final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(baseX / 2.0, baseY / 2.0), 
            new Translation2d(baseX / 2.0, -baseY / 2.0), 
            new Translation2d(-baseX / 2.0, baseY / 2.0), 
            new Translation2d(-baseX / 2.0, -baseY / 2.0)
        );
        public static final double driveP = 0.12;
        public static final double driveD = 0.0;
        public static final double steerP = 100.0;
        public static final double steerD = 0.0;
        public static final double driveA = 0.0;
        public static final double driveV = 0.0;
        public static final double driveS = 0.0;
        // public static final double driveA = 0.32;
        // public static final double driveV = 1.51;
        // public static final double driveS = 0.27;
        // private static final double autoDriveP = .12;
        // private static final double autoDriveD = 0;
        // private static final double autoSteerP = 25;
        // private static final double autoSteerD = .1;
        public static final HolonomicPathFollowerConfig pathConfig = new HolonomicPathFollowerConfig(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0), 
            maxSpeed,
            Math.sqrt((baseX * baseX) + (baseY * baseY)),
            new ReplanningConfig()
        );
        public static final PathConstraints constraints = new PathConstraints(.25* maxSpeed, maxAccel, .25* maxAngularVelocity, maxAngularAccel);

        // public class frontRight{ 
        //     public static final int driveMotorID = 10;
        //     public static final int angleMotorID = 11;
        //     public static final int encoderID = 12;
        //     // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.312012 * 360);
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.107177734375 * 360);
        //     public static final boolean isInverted = true;
        //     public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, isInverted);
        // }
        // public class backRight{ 
        //     public static final int driveMotorID = 20;
        //     public static final int angleMotorID = 21;
        //     public static final int encoderID = 22;
        //     // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.071289 * 360);
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.150146484375 * 360);
        //     public static final boolean isInverted = true;
        //     public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, isInverted);
        // }
        // public class backLeft{ 
        //     public static final int driveMotorID = 30;
        //     public static final int angleMotorID = 31;
        //     public static final int encoderID = 32;
        //     // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.381104  * 360);
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.30517578125  * 360);
        //     public static final boolean isInverted = false;
        //     public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, isInverted);
        // }
        // public class frontLeft{
        //     public static final int driveMotorID = 40;
        //     public static final int angleMotorID = 41;
        //     public static final int encoderID = 42;
        //     // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.347412 * 360);
        //     public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.068115234375 * 360);
        //     public static final boolean isInverted = false;
        //     public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID, encoderID, angleOffset, isInverted);
        // }

        public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
        public static TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
        public static CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();
        public static void configureCTRESwerve(){
            //encoder
            swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            //angle
            swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
            swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 150/7;
            swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;

            swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true; 
            swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 25;
            swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = 40;
            swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1; 
            
            swerveAngleFXConfig.Slot0.kP = steerP;
            swerveAngleFXConfig.Slot0.kI = 0.0; 
            swerveAngleFXConfig.Slot0.kD = steerD; 

            //drive
            swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
            swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            swerveDriveFXConfig.Feedback.SensorToMechanismRatio = 6.75;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = 35;
            swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = 60;
            swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = 0.1;

            swerveDriveFXConfig.Slot0.kP = driveP;
            swerveDriveFXConfig.Slot0.kI = 0.0;
            swerveDriveFXConfig.Slot0.kD = driveD;

            swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.45;
            swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.45; 
            swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0; 
            swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0; 
        }
    }
    public class MechConstants{
        //motorIDs
        public static final int shooterMotor1 = 1;
        public static final int shooterMotor2 = 2;
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