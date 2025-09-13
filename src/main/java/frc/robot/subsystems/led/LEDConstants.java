// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Superstructure.state;
import java.util.Map;

/** Add your docs here. */
public class LEDConstants {
  private static final Alliance kAlliance =
      DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
  public static final Map<state, Animation> animMap =
      Map.of(
          state.IDLE,
          new SingleFadeAnimation(
              convertColorToInt(Color.kGold)[0],
              convertColorToInt(Color.kGold)[1],
              convertColorToInt(Color.kGold)[2],
              0,
              0,
              -1));

  public static final Animation disabledAnim =
      new SingleFadeAnimation(
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.red * 255)
              : (int) (Color.kBlue.red * 255),
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.green * 255)
              : (int) (Color.kBlue.green * 255),
          (kAlliance == Alliance.Red)
              ? (int) (Color.kRed.blue * 255)
              : (int) (Color.kBlue.blue * 255));

  public static int[] convertColorToInt(Color color) {
    return new int[] {(int) (color.red * 255), (int) (color.blue * 255), (int) (color.green * 255)};
  }

  public class DeviceConstants {
    public static final double brightness = 0.5;
  }
}
