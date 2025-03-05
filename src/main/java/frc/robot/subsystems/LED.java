// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {

    // Instantiate the LED object
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public LED() {
        m_led = new AddressableLED(1);
        m_ledBuffer = new AddressableLEDBuffer(58);

        // Set the LED strip length before starting it
        m_led.setLength(58);
        m_led.start();
    }

    /** Creates a pulsing gradient pattern. */
    public void pulsePatternCreator() {
        LEDPattern basePattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlue)
                .breathe(Seconds.of(5));
        basePattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /** Creates a flame gradient pattern. */
    public void flamePatternCreator() {
        LEDPattern m_FlamePatternAttempt = LEDPattern.gradient(GradientType.kContinuous, Color.kRed, Color.kYellow);
        m_FlamePatternAttempt.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /** Creates a rainbow pattern with proper hue values. */
    public void rainbowPatternCreator() {
        LEDPattern rainbowPattern = LEDPattern.rainbow(360, 128);
        rainbowPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    /** Periodically updates the LED strip with the latest buffer data. */
    @Override
    public void periodic() {
        m_led.setData(m_ledBuffer);
    }

    /**
     * Creates a command that runs a pattern on the entire LED strip.
     *
     * @param pattern the LED pattern to run
     * @return a command to execute the LED pattern
     */
    public Command runPattern(LEDPattern pattern) {
        return this.run(() -> {
            pattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        }).withName("LEDPatternCommand");
    }
}