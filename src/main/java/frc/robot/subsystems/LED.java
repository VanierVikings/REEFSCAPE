// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    public static Color color = Color.kDarkRed;
    public int loopNum = 0;
    public boolean on = true;
    public int currentRed;
    public int currentGreen;
    public int currentBlue;
    public boolean blink = false;


    public LED() {
        m_led = new AddressableLED(LEDConstants.PWM_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LED);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void setColor(Color color, boolean blinking) {
        blink = blinking;
        currentRed = (int)(color.red*255);
        currentGreen = (int)(color.green*255);
        currentBlue = (int)(color.blue*255);
    }

    @Override
    public void periodic() {
        loopNum++;
        if(blink){
            if (loopNum % 50 == 0) {
                on = !on;
            }
            if (on) {
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, currentRed, currentGreen, currentBlue);
                }
            } else {
                // Off
                for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, (int)(Color.kBlack.red*255),(int)(Color.kBlack.green*255),(int)(Color.kBlack.blue*255));
                }
            }
            
            // Write the data to the LED strip
            
        } else{
            for (int i = 0; i < m_ledBuffer.getLength(); i++) {
                    m_ledBuffer.setRGB(i, currentRed, currentGreen, currentBlue);
                }
        }
        m_led.setData(m_ledBuffer);
    }
    
}