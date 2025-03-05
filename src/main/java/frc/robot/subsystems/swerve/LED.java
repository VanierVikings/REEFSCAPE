// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.servohub.config.ServoChannelConfig.PulseRange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {

    //Instantiate the led.
    public static AddressableLED m_led = new AddressableLED(LEDConstants.PWM_PORT);

    //Create a tangable LED object
    public static AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LED);
    
    //Set length of led
    


    public LED(){


        /*
         * 
         * Fire Gradiant 
         * Rainbow
         * Red to Gold
         * Solid colors
         * 
         * Make leds pulse slow when robot is deactivated (make it as easy as possible for the driver to know when the robot is activated
         * or deactivated)
         * 
         */
        
        //Allow commands to be applied to led, i.e., start the led
        m_led.start();

        m_led.setLength(LEDConstants.NUM_LED);

        // setDefaultCommand(runPattern(m_FlamePatternAttempt));

        // Set the default command to turn the strip off, otherwise the last colors written by
        // the last command to run will continue to be displayed.
        // Note: Other default patterns could be used instead!

    }
    


    public void pulsePatternCreator(){
        //Create subsection to apply a pattern to.

        // AddressableLEDBufferView subSection1 = m_ledBuffer.createView(0, 14);

        // subSection1.reversed();

        LEDPattern basePattern = LEDPattern.gradient(GradientType.kDiscontinuous,Color.kRed, Color.kBlue).breathe(Seconds.of(5));

        basePattern.applyTo(m_ledBuffer);

        m_led.setData(m_ledBuffer);

    }


    public void flamePatternCreator(){

        LEDPattern m_FlamePatternAttempt = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kYellow);
        
        m_FlamePatternAttempt.applyTo(m_ledBuffer);

        m_led.setData(m_ledBuffer);
        

    }


    public void rainbowPatternCreator(){

        LEDPattern rainbowPattern = LEDPattern.rainbow(2555, 128);

        rainbowPattern.applyTo(m_ledBuffer);

        m_led.setData(m_ledBuffer);
    }
    



    public void periodic() {
        // Periodically send the latest LED color data to the LED strip for it to display
        m_led.setData(m_ledBuffer);
    }
    
    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     */
    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledBuffer));
    }
}
