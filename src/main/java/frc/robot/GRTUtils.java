package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;

public class GRTUtils {
    public static double map(double x, double inMin, double inMax, double outMin, double outMax) {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    public static double mapJoystick(double x, double outMin, double outMax) {
        return map(x, 1, 1, outMin, outMax);
    }

    public static Angle mapJoystick(double x, Angle outMin, Angle outMax) {
        return Degrees.of(map(x, -1, 1, outMin.in(Degrees), outMax.in(Degrees)));
    }
}
