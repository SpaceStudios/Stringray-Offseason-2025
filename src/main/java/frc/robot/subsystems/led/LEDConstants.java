// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Superstructure.state;
import java.util.Map;

/** Add your docs here. */
public class LEDConstants {
  private static final int length = 308;
  private static final Alliance kAlliance =
      DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
  public static final Map<state, Animation> animMap =
      Map.of(
          state.IDLE,
          new SingleFadeAnimation(
              convertColorToInt(Color.kGold)[0],
              convertColorToInt(Color.kGold)[1],
              convertColorToInt(Color.kGold)[2],
              255,
              0.5,
              length),
          state.CORAL_INTAKE,
          new StrobeAnimation(
              convertColorToInt(Color.kWhite)[0],
              convertColorToInt(Color.kWhite)[1],
              convertColorToInt(Color.kWhite)[2],
              255,
              0.5,
              length),
          state.CORAL_READY,
          new SingleFadeAnimation(255, 255, 255, 255, 0.5, length),
          state.CORAL_PRESCORE,
          new StrobeAnimation(
              convertColorToInt(Color.kLimeGreen)[0],
              convertColorToInt(Color.kLimeGreen)[1],
              convertColorToInt(Color.kLimeGreen)[2],
              255,
              0.5,
              length),
          state.ALGAE_INTAKE,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          state.ALGAE_READY,
          new SingleFadeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          state.ALGAE_PRESCORE,
          new StrobeAnimation(
              convertColorToInt(Color.kPurple)[0],
              convertColorToInt(Color.kPurple)[1],
              convertColorToInt(Color.kPurple)[2],
              255,
              0.5,
              length),
          state.MANUAL_ELEVATOR,
          new StrobeAnimation(
              convertColorToInt(Color.kBlue)[0],
              convertColorToInt(Color.kBlue)[1],
              convertColorToInt(Color.kBlue)[2],
              255,
              0.5,
              length),
          state.CLIMB_READY,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length),
          state.CLIMB_PULL,
          new StrobeAnimation(
              convertColorToInt(Color.kTeal)[0],
              convertColorToInt(Color.kTeal)[1],
              convertColorToInt(Color.kTeal)[2],
              255,
              0.5,
              length));

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
              : (int) (Color.kBlue.blue * 255),
          255,
          0.5,
          length);

  public static int[] convertColorToInt(Color color) {
    return new int[] {(int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255)};
  }

  public class DeviceConstants {
    public static final double brightness = 0.5;
  }
}
