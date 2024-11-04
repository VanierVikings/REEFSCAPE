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
    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;

    public LED() {
        LED.m_led = new AddressableLED(LEDConstants.PWM_PORT);
        LED.m_ledBuffer = new AddressableLEDBuffer(LEDConstants.NUM_LED);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
    }

    public static void setColor(Color color) {
        m_led.start();
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, (int)(color.red* 255 ), (int)(color.green * 255), (int)(color.blue * 255));
        }
        m_led.setData(m_ledBuffer);
    }

    @Override
    public void periodic() {
    }
}