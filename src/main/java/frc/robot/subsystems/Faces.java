package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.util.Color;
import java.io.FileReader;
//import java.io.IOException;
//import java.util.HashMap;
//import java.util.Map;
import java.util.Properties;
//import edu.wpi.first.wpilibj.AddressableLED;
//import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import frc.util.Elastic.ElasticFace;

public class Faces extends SubsystemBase {
    //private static final int LED_PIN = 1;//port number
    private static final int NUM_LEDS = 264;//number of lights (16 by 16 plus 8)
    //private static final double BRIGHTNESS = 0.5;

    private Timer time = new Timer();
    //private AddressableLED led = new AddressableLED(LED_PIN);
    //private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(NUM_LEDS);
    private CANdle candle = new CANdle(1);

    private final double dampenFactor = 3;

    //private static final Map<String, short[]> Face = new HashMap<String, short[]>();

    public short[] chosenFace = {  
        50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0,50,50,0
    };
    public short[]previousFace = chosenFace;
    public short[]defaultFace = chosenFace;
    //private final LEDPattern blue = LEDPattern.solid(new Color(.2, .2, .2));
    public Faces() {
        //led = new AddressableLED(LED_PIN);
        //led.setLength(NUM_LEDS);
        //led.start();
        //blue.applyTo(ledBuffer);
    }
    public void resetTime(){
       time.reset();
    }
    public void startTime(){
        time.start();
     }
    public double getTime(){
        return time.get();
    }

    public void clear() {
        candle.setControl(new SolidColor(0, 263).withColor( new RGBWColor(0, 0, 0, 0)));
    }
    public void makeFace(short[] pixelsArray){
        for (int i = 0, j = 8; j < NUM_LEDS && i + 2 < pixelsArray.length; i += 3, j++) {  
            candle.setControl(new SolidColor(j, j).withColor( new RGBWColor((int) (pixelsArray[i] * ElasticFace.kPOWER_FACTOR.get()), (int) (pixelsArray[i + 1] * ElasticFace.kPOWER_FACTOR.get()), (int) (pixelsArray[i + 2] * ElasticFace.kPOWER_FACTOR.get()), 0)));
        }
        //led.setData(ledBuffer);
    }
    public void setFace(short[] face){
        chosenFace = face;
    }
    public short[] getFace(String name) {
        try {
            FileReader faceFile = new FileReader(Filesystem.getDeployDirectory() + "/faces.properties");
            Properties p = new Properties();
            p.load(faceFile);
            return convertToArray(p.getProperty(name));
        } catch (Exception e) {
            System.err.println("Error loading face properties: " + e);
            return defaultFace;
        }
    }
    public short[] convertToArray(String input) {
        String[] stringArray = input.split(",");
        short[] shortArray = new short[stringArray.length];
        for (int i = 0; i < stringArray.length; i++) {
            try {
                shortArray[i] = Short.parseShort(stringArray[i].trim());
            } catch (NumberFormatException e) {
                System.err.println("Error parsing short: " + e);
                return new short[0];
            }
        }
        return shortArray;
    }


    public void smile(){
        setFace(getFace("smile"));
        makeFace(chosenFace);
    }
    
    public void nextDizzy(int mode){
        if(mode == 0){
            setFace(getFace("dizzyOne"));
        }else if(mode == 1){
            setFace(getFace("dizzyTwo"));
        }else if(mode == 2){
            setFace(getFace("dizzyThree"));
        }else if(mode == 3){
            setFace(getFace("dizzyFour"));
        }
        makeFace(chosenFace);
    }
    public void submerge(int mode){
        if(mode == 0){
            setFace(getFace("waterOne"));
        }else if(mode == 1){
            setFace(getFace("waterTwo"));
        }else if(mode == 2){
            setFace(getFace("waterThree"));
        }else if(mode == 3){
            setFace(getFace("waterFour"));
        }else if(mode == 4){
            setFace(getFace("waterFive"));
        }else if(mode == 5){
            setFace(getFace("waterSix"));
        }else if(mode == 6){
            setFace(getFace("waterSeven"));
        }else if(mode == 7){
            setFace(getFace("waterEight"));
        }else if(mode == 8){
            setFace(getFace("waterNine"));
        }else if(mode == 9){
            setFace(getFace("waterTen"));
        }else if(mode == 10){
            setFace(getFace("waterEleven"));
        }else if(mode == 11){
            setFace(getFace("waterTwelve"));
        }else if(mode == 12){
            setFace(getFace("waterThirteen"));
        }else if(mode == 13){
            setFace(getFace("waterFourteen"));
        }else if(mode == 14){
            setFace(getFace("waterFifteen"));
        }else if(mode == 15){
            setFace(getFace("waterSixteen"));
        }
        makeFace(chosenFace);
    }
    public void jeremy(int mode){
        if(mode == 0){
            setFace(getFace("jeremyOne"));
        }else if(mode == 1){
            setFace(getFace("jeremyTwo"));
        }else if(mode == 2){
            setFace(getFace("jeremyThree"));
        }else if(mode == 3){
            setFace(getFace("jeremyTwo"));
        }
        makeFace(chosenFace);
    }
    public void angry(int mode){
        if(mode == 0){
            setFace(getFace("smile"));
        }else if(mode == 1){
            setFace(getFace("angryTwo"));
        }else if(mode == 2){
            setFace(getFace("angryThree"));
        }else if(mode == 3){
            setFace(getFace("angryFour"));
        }else if(mode == 4){
            setFace(getFace("dead"));
        }
        makeFace(chosenFace);
    }
    public void money(int mode){
        if(mode == 0){
            setFace(getFace("moneyOne"));
        }else if(mode == 1){
            setFace(getFace("moneyTwo"));
        }
        makeFace(chosenFace);
    }
    public void party(int mode){
        if(mode == 0){
            setFace(getFace("partyOne"));
        }else if(mode == 1){
            setFace(getFace("partyTwo"));
        }
        makeFace(chosenFace);
    }
    public void sleepy(){
        setFace(getFace("sleepy"));
        makeFace(chosenFace);
    }
    public void frown(){
        setFace(getFace("frown"));
        makeFace(chosenFace);
    }
    public void crying(){
        setFace(getFace("crying"));
        makeFace(chosenFace);
    }
    public void wink(){
        setFace(getFace("wink"));
        makeFace(chosenFace);
    }
    public void speed(){
        setFace(getFace("speed"));
        makeFace(chosenFace);
    }
    public void sick(){
        setFace(getFace("sick"));
        makeFace(chosenFace);
    }
    public void heartEyes(){
        setFace(getFace("heartEyes"));
        makeFace(chosenFace);
    }
    public void fearShock(){
        setFace(getFace("fearShock"));
        makeFace(chosenFace);
    }
    public void eyebrowRaise(){
        setFace(getFace("eyebrowRaise"));
        makeFace(chosenFace);
    }
    public void pirate(){
        setFace(getFace("pirate"));
        makeFace(chosenFace);
    }
    public void red(){
        System.out.println("Hello");
        setFace(getFace("red"));
        makeFace(chosenFace);
    }
    public void blue(){
        setFace(getFace("blue"));
        makeFace(chosenFace);
    }
    public void cupcake(){
        setFace(getFace("cupcake"));
        makeFace(chosenFace);
    }
    public void clown(){
        setFace(getFace("clown"));
        makeFace(chosenFace);
    }
    public void bad(){
        setFace(getFace("exclamationPoint"));
        makeFace(chosenFace);
    }

    @Override
    public void periodic() {
       //led.setData(ledBuffer);
       //led.setData(ledBuffer);
       /*try {
        getFace("smile");
       } catch (Exception e) {
        // TODO: handle exception
        System.err.println(e);
       }*/
    }
}
