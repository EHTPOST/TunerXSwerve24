package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

enum Stage {
    START,
    TAKINGIN,
    LOADED,
    RAMPINGUP,
    SHOOTING
}

public class MetaSubsystem extends SubsystemBase {

    private Shooter shooter;
    private Intake intake;
    private Candle candle;
    private Stage stage;
    private int blinkCount;
    private int countToHalf;
    private DigitalInput whatTheNoteDoing;
    private int intakeVersion;
    private int shootVersion;
    private boolean shootPressed;

    public MetaSubsystem(Shooter shooter, Intake intake, Candle candle) {
        this.shooter = shooter;
        this.intake = intake;
        this.candle = candle;
        stage = Stage.START;
        blinkCount = 0;
        countToHalf = 0;
        whatTheNoteDoing = new DigitalInput(0);
        shootVersion = 0;
        intakeVersion = 0;
        shootPressed = false;
    }

    public void intakeVersion(int version) {
        intakeVersion = version;
    }

    public void shootVersion(int version){
        shootVersion = version;
    }

    public void shoot(boolean pressed) {
        shootPressed = pressed;
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("stage", stage.toString());
        switch (stage) {
            case START:
                shootVersion = 0;
                shootPressed = false;
                shooter.brake().schedule();
                intake.brake().schedule();
                candle.red();
                if (!whatTheNoteDoing.get()) {
                    stage = Stage.LOADED;
                    break;
                }
                if (intakeVersion != 0) {
                    stage = Stage.TAKINGIN;
                    break;
                }
                break;
            case TAKINGIN:
                if(intakeVersion == 1){
                    intake.takeIn().schedule();
                }
                if(intakeVersion == 2){
                    shooter.takeInShooter().schedule();
                    intake.takeInShooter().schedule();
                }
                candle.red();
                if (!whatTheNoteDoing.get()) {
                    stage = Stage.LOADED;
                    break;
                }
                break;
            case LOADED:
                intake.brake().schedule();
                shooter.brake().schedule();
                if (blinkCount <= 10) {
                    candle.blue();
                } else if (blinkCount > 10) {
                    candle.white();
                    if (blinkCount >= 20) {
                        blinkCount = 0;
                    }
                }
                blinkCount++;
                if (shootVersion != 0) {
                    stage = Stage.RAMPINGUP;
                    break;
                }
                break;
            case RAMPINGUP:
                shooter.shoot(shootVersion).schedule();
                //LEDs
                if (shootPressed) {
                    stage = Stage.SHOOTING;
                    shootPressed = false;
                }
                break;
            case SHOOTING:
                intakeVersion = 0;
                candle.blue();
                intake.takeIn().schedule();
                shooter.shoot(shootVersion).schedule();
                if (countToHalf ==  50) {
                    stage = Stage.START;
                    countToHalf = 0;
                } else {
                    countToHalf++;
                }
                break;
        }
    }
}
