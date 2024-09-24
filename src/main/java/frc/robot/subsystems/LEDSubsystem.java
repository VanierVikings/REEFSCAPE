// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private int m_rainbowFirstPixelHue;


    public LEDSubsystem() {
        m_led = new AddressableLED(LEDConstants.PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LED);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    /**
     * Called to set LEDs to specified colour from Color object.
     * @param startIndex - Index of first LED to set colour.
     * @param endIndex - Index of last LED to set colour.
     * @param color - Color object to set LEDs to.
     */
    public void setColor(int startIndex, int endIndex, Color color) {
        int red = (int) Math.round(color.red * 255);
        int green = (int) Math.round(color.green * 255);
        int blue = (int) Math.round(color.blue * 255);
        for (var i = startIndex; i < endIndex; i++) {
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
    }



    /**
     * Called to set rainbow LEDs colours.
     * @param startIndex - Index of first LED to set rainbow.
     * @param endIndex - Index of last LED to set rainbow.
     */
    public void setRainbow(int startIndex, int endIndex) {
        for (var i = startIndex; i < endIndex; i++) {
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
        m_led.setData(m_ledBuffer);
    }

    /**
     * Called periodically to update rainbow LED colours.
     */
    public void updateRainbow(int startIndex, int endIndex){
        m_rainbowFirstPixelHue += 1;
        m_rainbowFirstPixelHue %= 180;
        setRainbow(startIndex, endIndex);
    }

    @Override
    public void periodic() {
        updateRainbow(LEDConstants.RAINBOW_START_INDEX, LEDConstants.RAINBOW_END_INDEX); 
    }
}