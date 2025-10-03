// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/** Add your docs here. */
public class AutoMaker {
  LoggedDashboardChooser<Supplier<Command>> positionChooser =
      new LoggedDashboardChooser<Supplier<Command>>("FirstScore");
}
