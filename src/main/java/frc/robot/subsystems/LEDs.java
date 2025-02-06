package frc.robot.subsystems;

import java.lang.invoke.MethodHandles;
// import java.util.function.DoubleSupplier;

// import edu.wpi.first.units.measure.Distance;
// import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.units.Units;
// import edu.wpi.first.units.TimeUnit;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants;
// import frc.robot.motors.TalonFXLance;

/**
 * This is an example of what a subsystem should look like.
 */
public class LEDs extends SubsystemLance
{
    // This string gets the full name of the class, including the package name
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();

    // *** STATIC INITIALIZATION BLOCK ***
    // This block of code is run first when the class is loaded
    static
    {
        System.out.println("Loading: " + fullClassName);
    }
    

    // *** INNER ENUMS and INNER CLASSES ***
    // Put all inner enums and inner classes here
    // public enum CustomColor
    // {
    //     kRed(255, 0, 0),
    //     kBlue(0, 102, 204),
    //     kCyan(51, 255, 255),
    //     kPurple(204, 0, 204),
    //     kOrange(255, 128, 0),
    //     kGreen(0, 204, 0),
    //     kPink(255, 51, 153),
    //     kWhite(255, 255, 255);

    //     int r;
    //     int g;
    //     int b;

    //     private CustomColor(int r, int g, int b)
    //     {
    //         this.r = r;
    //         this.g = g;
    //         this.b = b;
    //     }
    // }

    
    // *** CLASS VARIABLES & INSTANCE VARIABLES ***
    // Put all class variables and instance variables here
    // private final TalonFXLance motor1 = new TalonFXLance(4, Constants.ROBORIO, "Motor 1");
    // private final TalonFXLance motor2 = new TalonFXLance(12, Constants.ROBORIO, "Motor 2");
    // private final AddressableLED m_led = new AddressableLED(1);
    // private final AddressableLEDBuffer blankBuffer;
    // private final AddressableLEDBuffer setBuffer;
    // private final Timer timer = new Timer();
    public LEDPattern gradient;
    private AddressableLED led = new AddressableLED(1);
    private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(49);

    private LEDPattern off = LEDPattern.solid(Color.kBlack);
    private final LEDPattern rainbow = LEDPattern.rainbow(255, 255);
    private LEDPattern solid;

    // Scroll Pattern
    // Distance ledSpacing = Meters.of(1 / 120.0);
    
    private LEDPattern base;
    private LEDPattern blinkPattern;
    // LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));
    // LEDPattern absolute = base.scrollAtAbsoluteSpeed(Centimeters.per(Second).of(12.5), ledSPacing);

    // LEDPattern coolPattern = LEDPattern.progressMaskLayer(() -> 4/5);
    // private final LEDPattern scrollingRainbow = rainbow.scrollAtAboutSpeed(MetersPerSecond.of(1), kLedSpacing);

    // private static final Distance kLedSpacing = Meters.of(1/120.0);
    // *** CLASS CONSTRUCTORS ***
    // Put all class constructors here
  
    /** 
     * Creates a new ExampleSubsystem. 
     */
    public LEDs()
    {
        super("LEDs");
        System.out.println("  Constructor Started:  " + fullClassName);

        // timer.start();

        // m_led = new AddressableLED(5);
        // m_ledBuffer = new AddressableLEDBuffer(5);

        led.setLength(200);
        // blankBuffer = new AddressableLEDBuffer(5);
        // setBuffer = new AddressableLEDBuffer(5);

        // gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kOrange, Color.kPink);
        
        configLEDs();
        // configBuffers();
        System.out.println("  Constructor Finished: " + fullClassName);
    }

    private void configLEDs()
    {
        led.start();
        // m_ledBuffer.setRGB(Color.kRed);
        // getRed(Color.kRed);
    }

    // private void configBuffers()
    // {
    //     for(int i = 0; i < m_led.getLength(); i++)
    //     {
    //         m_led.setRGB(i, 0, 0, 0);
    //     }
    // }

    private void off()
    {
        off.applyTo(ledBuffer);
        led.setData(ledBuffer);
    }

    private void setColorSolid(Color color)
    {
        solid = LEDPattern.solid(color);
        solid.applyTo(ledBuffer);
    }

    // private void setCoolPattern()
    // {
    //     coolPattern.applyTo(ledBuffer);
    //     led.setData(ledBuffer);
    // }

    // private void setColorBlink(int r, int g, int b)
    // {
        // for(int i = 0; i < blankBuffer.getLength(); i++)
        // {
        //     setBuffer.setRGB(i, r, g, b);
        // }

        // if(timer.get() % 0.4 < 0.2)
        // {
        //     ledStrip.setData(setBuffer);
        // }
        // else
        // {
        //     ledStrip.setData(blankBuffer);
        // }
    // }

    private void setColorRainbow()
    {
        rainbow.applyTo(ledBuffer);
    }

    public void setColorGradient(Color color1, Color color2, Color color3, Color color4, Color color5)
    {
        gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2, color3, color4, color5);
        gradient.applyTo(ledBuffer);
    }

    public void setColorBlink(Color color1, Color color2, Color color3, Color color4, Color color5)
    {
        base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, color1, color2, color3, color4, color5);
        blinkPattern = base.breathe(Units.Seconds.of(2));
        blinkPattern.applyTo(ledBuffer);
    }

    // public void setColorMix

    //COMMANDS

    public Command offCommand()
    {
        return run(() -> off()).withName("Turn Off");
    }

    // public Command stopCommand()
    // {
    //     return runOnce(() -> off()).withName("Turn Off");
    // }

    public Command setColorSolidCommand(Color color)
    {
        return runOnce(() -> setColorSolid(color)).withName("Set LED Solid");
    }

    // public Command setColorBlinkCommand(CustomColor color)
    // {
    //     return run(() -> setColorBlink(color.r, color.g, color.b)).withName("Set LED Blink");
    // }

    public Command setColorRainbowCommand()
    {
        return run(() -> setColorRainbow()).withName("Set LED Rainbow");
    }

    public Command setColorGradientCommand(Color color1, Color color2, Color color3, Color color4, Color color5)
    {
        return run(() -> setColorGradient(color1, color2, color3, color4, color5)).withName("Set LED Gradient");
    }

    public Command setColorBlinkCommand(Color color1, Color color2, Color color3, Color color4, Color color5)
    {
        return run(() -> setColorBlink(color1, color2, color3, color4, color5)).withName("Set LED Blink");
    }



    // *** CLASS METHODS & INSTANCE METHODS ***
    // Put all class methods and instance methods here

    // Use a method reference instead of this method
    // public Command stopCommand()
    // {
    //     return run( () -> stop() );
    // }


    // *** OVERRIDEN METHODS ***
    // Put all methods that are Overridden here

    @Override
    public void periodic()
    {
        led.setData(ledBuffer);
        // This method will be called once per scheduler run
        // Use this for sensors that need to be read periodically.
        // Use this for data that needs to be logged.
    }

    @Override
    public String toString()
    {
        return "";
    }
}
