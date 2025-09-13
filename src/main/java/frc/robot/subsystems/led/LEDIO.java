// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

/** Add your docs here. */
public interface LEDIO {
  public class ledData {
    public boolean connected = false;
    public String currentColor = "000000"; // Current Color Hexcode
    public double temperature = 0.0;
    public double current = 0.0;
  }

  public default void setAnimation(Animation anim) {}

  public default void getData(ledData data) {}
}
