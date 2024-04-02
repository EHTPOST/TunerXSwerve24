// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 2.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

    //driver
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
  private final JoystickButton zeroGyro = new JoystickButton(joystick.getHID(), XboxController.Button.kY.value); //y = zeroGyro
  
    //operator
  private final Joystick operator = new Joystick(1);
  private final JoystickButton rampSpeaker = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton rampAmp = new JoystickButton(operator, XboxController.Button.kB.value); 
  private final JoystickButton takeInGround = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton takeInTop = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton climbUp = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton climbDown = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final int shootAxis = XboxController.Axis.kRightTrigger.value;
  private final Trigger shoot = new Trigger(() -> operator.getRawAxis(shootAxis) > .5);
  private final int ferryAxis = XboxController.Axis.kLeftTrigger.value;
  private final Trigger rampFerry = new Trigger(() -> operator.getRawAxis(ferryAxis) > .5);

  //subsystems
  public final Shooter shooter;
  public final Intake intake;
  public final Climber climb;
  private final Candle candle;
  private final MetaSubsystem meta;

  private final SendableChooser<String> autoChooser;
  
  
  public RobotContainer() {
    //subsystems
    candle = new Candle(new CANdle(0, "rio"));
    shooter = new Shooter(Constants.MechConstants.shooterMotor1, Constants.MechConstants.shooterMotor2); 
    intake = new Intake(Constants.MechConstants.intakeMotor, Constants.MechConstants.feederMotor);
    meta = new MetaSubsystem(shooter, intake, candle);
    climb = new Climber();

    //autos
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Default", "Default");
    autoChooser.addOption("Amp Shot", "startAmp");
    autoChooser.addOption("Speak AMP", "startLeft");
    autoChooser.addOption("Speak Center", "startCenter");
    autoChooser.addOption("Speak Source", "startRight");
    autoChooser.addOption("Shoot", "Shoot");
    autoChooser.addOption("Grinch Source", "operationLitterBoxSource");
    autoChooser.addOption("Grinch Center", "operationLitterBoxCenter");
    autoChooser.addOption("Grinch Amp", "operationLitterBoxAmp");
    autoChooser.addOption("Grinch with Amp Shot", "operationLitterBoxAmpShot");
    SmartDashboard.putData("Auto Chooser", autoChooser); 

    //Named Commands for auto
    NamedCommands.registerCommand("shootSpeaker", new InstantCommand(() -> meta.shootVersion(1)));
    NamedCommands.registerCommand("shootAmp", new InstantCommand(() -> meta.shootVersion(2)));
    NamedCommands.registerCommand("shoot", new InstantCommand(() -> meta.shoot(true)));

    configureBindings();
  }

  private void configureBindings() {
    //driver
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (spin)
    ));
    zeroGyro.onTrue(new InstantCommand(() -> drivetrain.seedFieldRelative()));
    
    //operator
    rampSpeaker.onTrue(new InstantCommand(() -> meta.shootVersion(1)));
    rampAmp.onTrue(new InstantCommand(() -> meta.shootVersion(2)));
    rampFerry.onTrue(new InstantCommand(() -> meta.shootVersion(3)));
    shoot.onTrue(new InstantCommand(() -> meta.shoot(true)));
    takeInGround.onTrue(new InstantCommand(() -> meta.intakeVersion(1)));
    takeInTop.onTrue(new InstantCommand(() -> meta.intakeVersion(2)));
    climbUp.onTrue(new InstantCommand(() -> {
      climb.climbUp();
    })).onFalse(new InstantCommand(() -> {
      climb.brake();
    }));
    climbDown.onTrue(new InstantCommand(() -> {
      climb.climbDown();
    })).onFalse(new InstantCommand(() -> {
      climb.brake();
    }));
  }
  
  //Hi emmett
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoChooser.getSelected());
  }
}
