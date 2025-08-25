// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.proximity;

import com.reduxrobotics.sensors.canandcolor.Canandcolor;

import edu.wpi.first.math.filter.Debouncer;

/** Add your docs here. */
public class ProximityIOCanAndColor implements ProximityIO {
  private final Canandcolor sensor;
    private final double threshold;
  private final Debouncer connectedDebouncer = new Debouncer(0.5);

  public ProximityIOCanAndColor(int id) {
    this(id, 0.5);
  }

  public ProximityIOCanAndColor(int id, double threshold) {
    sensor = new Canandcolor(id);
    this.threshold = threshold;
  }

  @Override
  public void getData(ProximityDataAutoLogged data) {
    data.connected = connectedDebouncer.calculate(sensor.isConnected());
    
    data.distance = sensor.getProximity();
    data.detected = data.distance <= threshold;

    data.temperature = sensor.getTemperature();
  }
}
