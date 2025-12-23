// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;

public class DriveConstants {
    public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    public static final double joystickDeadbandDecimal = 0.1; // The deadband applied as a decimal (0.1 = 10%)

    // Pid Constants for alignment
    public static final double tagXKp = 2.0;
    public static final double tagXKi = 0.0;
    public static final double tagXKd = 0.0;

    public static final double tagYKp = 2.0;
    public static final double tagYKi = 0.0;
    public static final double tagYKd = 0.0;

    public static final double tagRKp = 0.2 * 0.6;
    public static final double tagRKi = (1.2 * 0.2) / 0.37;
    public static final double tagRKd = (3 * 0.2 * 0.37) / 40;
}
