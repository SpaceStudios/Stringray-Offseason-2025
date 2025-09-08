// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;

/** Add your docs here. */
public class LEDIOSim implements LEDIO {
  private Animation kAnimation = LEDConstants.disabledAnim;

  public LEDIOSim() {}

  @Override
  public void getData(ledData data) {}
}
