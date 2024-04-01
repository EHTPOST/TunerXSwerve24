package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

class Tuple<X, Y> {
    public X x;
    public Y y;
    public Tuple(X x, Y y){
        this.x = x;
        this.y = y;
    }
}

public class Shooter extends SubsystemBase{
    private TalonFX motor1;
    private TalonFX motor2;
    private DutyCycleOut voltage1;
    private DutyCycleOut voltage2;
    
    public Shooter(int id1, int id2){
        motor1 = new TalonFX(id1); //top
        motor2 = new TalonFX(id2); //bottom
        voltage1 = new DutyCycleOut(0);
        voltage2 = new DutyCycleOut(0);
    
        //config
        Constants.MechConstants.configureCTREShoot();
        motor1.getConfigurator().apply(Constants.MechConstants.shootMotor1Config);
        motor2.getConfigurator().apply(Constants.MechConstants.shootMotor2Config);
    }

    public Command shoot(int version){
        switch(version){
            case 1:
                return shootSpeaker();
            case 2:
                return shootAmp();
            case 3:
                return shootFerry();
        }
        return null;
    }

    public Command shootSpeaker(){
        Tuple<Double, Long> distance = calculateShootDistance();
        return this.runOnce(() -> {
            if(distance.y == 4 || distance.y == 8 || distance.y == 3 || distance.y == 7){
                Tuple<Double, Double> speeds = calculateSpeedSpeaker();
                voltage1.Output = speeds.x;
                voltage2.Output = speeds.y;
            }
            else {
                voltage1.Output = Constants.MechConstants.at37InFromSpeaker1;
                voltage2.Output = Constants.MechConstants.at37InFromSpeaker1;
            }    
            motor1.setControl(voltage1);
            motor2.setControl(voltage2);
        });
    }

    public Command shootAmp(){
        // voltage1.Output = SmartDashboard.getNumber("motor1", .1);
        // voltage2.Output = SmartDashboard.getNumber("motor2", .22);
        // voltage1.Output = Constants.MechConstants.ferryMotor1;
        // voltage2.Output = Constants.MechConstants.ferryMotor2;
        voltage1.Output = Constants.MechConstants.atAmpMotor1;
        voltage2.Output = Constants.MechConstants.atAmpMotor2;
        return this.run(()-> 
        {
            motor1.setControl(voltage1);
            motor2.setControl(voltage2);
        });
    }

    public Command shootFerry(){
        return this.runOnce(() -> {
            voltage1.Output = Constants.MechConstants.ferryMotor1;
            voltage2.Output = Constants.MechConstants.ferryMotor2;
            motor1.setControl(voltage1);
            motor2.setControl(voltage2);
        });
    }

    public Command takeInShooter(){
        return this.runOnce(() -> {
            voltage1.Output = Constants.MechConstants.takeInShooter;
            voltage2.Output = Constants.MechConstants.takeInShooter;
            motor1.setControl(voltage1);
            motor2.setControl(voltage2);
        });
    }
    
    public Command brake(){
        return this.runOnce(() -> {
            motor1.setControl(new DutyCycleOut(0));
            motor2.setControl(new DutyCycleOut(0));
        });
    }

    private Tuple<Double, Double> calculateSpeedSpeaker(){
        double speed1Slope = (Constants.MechConstants.at60InFromSpeaker1-Constants.MechConstants.at37InFromSpeaker1)/(60 - 37);
        double speed2Slope = (Constants.MechConstants.at60InFromSpeaker2 - Constants.MechConstants.at37InFromSpeaker2)/(60 - 37);
        double speed1B = -1 *((speed1Slope * 60) - Constants.MechConstants.at60InFromSpeaker1);
        double speed2B = -1 * ((speed2Slope * 60) - Constants.MechConstants.at60InFromSpeaker2);
        double distance = calculateShootDistance().x;
        double speed1 = speed1Slope*distance + speed1B;
        double speed2 = speed2Slope*distance + speed2B;
        Tuple<Double, Double> speeds = new Tuple<>(speed1, speed2);
        return speeds;
    }

    private Tuple<Double, Long> calculateShootDistance() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        Long tid = table.getEntry("tid").getInteger(-1);
        double h1 = Constants.MechConstants.speakerAprilHeightInches;
        double h2 = Constants.MechConstants.shootLLHeightInches;
        double angleRadians = (Constants.MechConstants.shootLLAngleDegrees + targetOffsetAngle_Vertical) * (3.14159 /180);
        double distance = ( h1 - h2) / Math.tan(angleRadians);
        if(targetOffsetAngle_Vertical == 0){
            distance = 36.125;
        }
        return new Tuple<Double, Long>(distance, tid);
    }
    
    @Override
    public void periodic(){
        calculateSpeedSpeaker();
    }
}
