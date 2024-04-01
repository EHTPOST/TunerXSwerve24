package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase{
    private final CANdle candle;
    private boolean partyTime;
    // private int partyStart;
    private int[][] rgbVals = {{255, 0, 0}, {255, 127, 0}, {255,255,0}, 
                            {127, 255, 0}, {0,255,0},{0,255,127}, 
                            {0,255,255}, {0,127,255}, {0,0,255},
                            {127,0,255}, {255,0,255}, {255,0,127}};

    public Candle(CANdle candle){
        System.out.println("Running");
        this.candle = candle;

        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = .8;
        config.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(config);
        candle.clearStickyFaults();
        candle.clearAnimation(0);
        candle.setLEDs(0,0,0,0,0,500);

        partyTime = false;
        // partyStart = 0;
    }

    public void red(){
        candle.setLEDs(255, 0,0,0, 0, 500);
    }

    public void redLarsons() {
        candle.animate(new LarsonAnimation(255, 0, 0, 0, 0.3, 50, BounceMode.Front, 5, 0));
    }

    public void blue(){
        candle.setLEDs(0,0,255,0,0,500);
    }

    public void white(){
        candle.setLEDs(0,0,0,0,0,500);
    }

    public void party(int start){
        for(int i = 0; i < rgbVals.length; i++){
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6)), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 72), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 144), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 216), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 288), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 360), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 432), 6);
            candle.setLEDs(rgbVals[i][0], rgbVals[i][1], rgbVals[i][2], 0, partyLocationLooper(start + (i*6) + 504), 6);
        }
    }

    public int partyLocationLooper(int location){
        return location % 500;
    }

    public boolean getPartyTime(){
        return partyTime;
    }

    @Override
    public void periodic(){
        // if(Timer.getMatchTime() >= 133){
        //     partyTime = true;
        // }
        // if (partyTime){
        //     partyStart++;
        //     partyStart = partyLocationLooper(partyStart);
        //     party(partyStart);  
        // }
    }
}
