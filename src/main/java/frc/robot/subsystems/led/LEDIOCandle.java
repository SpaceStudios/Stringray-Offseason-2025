// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;

/** Add your docs here. */
public class LEDIOCandle implements LEDIO {
    private final CANdle candle = new CANdle(11);
    public LEDIOCandle() {
        final CANdleConfiguration config = new CANdleConfiguration();
        config.brightnessScalar = LEDConstants.DeviceConstants.brightness;
        config.disableWhenLOS = false;
        config.enableOptimizations = true;
        
        candle.configAllSettings(config);
    }

    @Override
    public void getData(ledData data) {
        data.temperature = candle.getTemperature();
        data.current = candle.getCurrent();
    }

    @Override
    public void setAnimation(Animation animation) {
        candle.animate(animation);
    }
}
