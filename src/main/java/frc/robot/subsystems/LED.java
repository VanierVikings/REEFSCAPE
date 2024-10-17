// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.TreeSet;
import java.util.function.BiConsumer;
import java.util.function.Supplier;
import frc.robot.Constants;

public class LED {
  private static LED m_instance = new LED();

  private AddressableLED led = new AddressableLED(Constants.LEDConstants.PWM_PORT);
  private AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.LEDConstants.NUM_LED);
  private final PersistentLedState persistentLedState = new PersistentLedState();
  private States previousState = States.Default;

  private static class PersistentLedState {
    public int rainbowFirstPixelHue = 0;
    public int pulseOffset = 0;
  }

  /** Creates a new LedS. */
  private LED() {
    led.setLength(buffer.getLength());

    States.Disabled.setter.accept(buffer, persistentLedState);
    led.setData(buffer);
    led.start();
  }

  public static LED getInstance() {
    return m_instance;
  }


  private TreeSet<States> m_states = new TreeSet<>();
  /**
   * Different states of the robot, with an integer that determines the priority of the state (the
   * lower the number, the higher the priority)
   */
  public static enum States {
    SetupDone(setColor(0, 128, 0)), // set in robotPeriodic
    Disabled(setColor(255, 0, 0)), // set in robotPeriodic
    Error(pulse(0.25, setColor(255, 0, 0))),
    IntakeActive(pulse(0.25, setColor((int)(Color.kLightYellow.red*255), (int)(Color.kLightYellow.green*255), (int)(Color.kLightYellow.blue*255)))),
    IntakedNote(setColor((int)(Color.kDarkRed.red*255), (int)(Color.kDarkRed.green*255), (int)(Color.kDarkRed.blue*255))),
    ShooterEnabled(setColor((int)(Color.kLimeGreen.red*255), (int)(Color.kLimeGreen.green*255), (int)(Color.kLimeGreen.blue*255))),
    ShooterSpeedReached(pulse(0.25,setColor((int)(Color.kLimeGreen.red*255), (int)(Color.kLimeGreen.green*255), (int)(Color.kLimeGreen.blue*255)))),
    Default(setColor((int)(Color.kAliceBlue.red*255), (int)(Color.kAliceBlue.green*255), (int)(Color.kAliceBlue.blue*255)));

    public final BiConsumer<AddressableLEDBuffer, PersistentLedState> setter;

    private States(BiConsumer<AddressableLEDBuffer, PersistentLedState> setter) {
      this.setter = setter;
    }
  }

  // currentStates = {Disabled, Climbing, EjectingWrongColor, Intaking, Shooting,
  // Default}

  /**
   * Requests the current state of the robot, determines whether the requested state is a higher
   * priority than the current state, sets the current state to the requested state
   *
   * @param state The requested state of the robot when the method is called
   */
  public void requestState(States state) {
    m_states.add(state);
  }

  public Command stateC(Supplier<States> state) {
    return Commands.run(() -> requestState(state.get()));
  }

  /**
   * Periodically checks the current state of the robot and sets the LEDs to the corresponding light
   * pattern
   */
  public void periodic() {
    requestState(States.Default);
    // if (DriverStation.isDisabled()) {
    //   requestState(States.Disabled);
    // }
    States state = m_states.first();
    if (state != previousState) {
      persistentLedState.pulseOffset = 0;
    }
    // spark.set(m_states.first().lightSpeed);
    state.setter.accept(buffer, persistentLedState);
    previousState = state;
    // Do other things with the buffer
    led.setData(buffer);
    m_states.removeAll(Set.of(States.values()));
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> setColor(
      int r, int g, int b) {
    return (buffer, state) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        buffer.setRGB(i, r, g, b);
      }
    };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> barFill(
      BiConsumer<AddressableLEDBuffer, PersistentLedState> start,
      BiConsumer<AddressableLEDBuffer, PersistentLedState> end) {
    return (buffer, state) -> {
      for (int i = 0; i < buffer.getLength(); i++) {
        if (i > state.pulseOffset) {
          start.accept(buffer, state);
        } else {
          end.accept(buffer, state);
        }
      }
      state.pulseOffset++;
    };
  }

  private static BiConsumer<AddressableLEDBuffer, PersistentLedState> pulse(
      double period, BiConsumer<AddressableLEDBuffer, PersistentLedState> pattern) {
    return (buffer, state) -> {
      if (Timer.getFPGATimestamp() % period < 0.5 * period) {
        pattern.accept(buffer, state);
      } else {
        for (int i = 0; i < buffer.getLength(); i++) {
          buffer.setRGB(i, 0, 0, 0);
        }
      }
    };
  }
}