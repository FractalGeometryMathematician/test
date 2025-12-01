package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ControllerTest extends SubsystemBase {
    public ControllerTest() {

    }

    public Command joystickInput(
            DoubleSupplier l2,
            DoubleSupplier r2,
            DoubleSupplier rightX,
            DoubleSupplier rightY,
            DoubleSupplier leftX,
            DoubleSupplier leftY,
            BooleanSupplier povUp,
            BooleanSupplier povDown) {

        return this.run(() -> {

            // L2 trigger (already conditional)
            if (l2.getAsDouble() != 0) {
                System.out.print("l2 trigger: ");
                System.out.println(l2.getAsDouble());
            }

            // R2 trigger
            if (r2.getAsDouble() != 0) {
                System.out.print("r2 trigger: ");
                System.out.println(r2.getAsDouble());
            }

            // Right stick X
            if (rightX.getAsDouble() != 0) {
                System.out.print("rightX: ");
                System.out.println(rightX.getAsDouble());
            }

            // Right stick Y
            if (rightY.getAsDouble() != 0) {
                System.out.print("rightY: ");
                System.out.println(rightY.getAsDouble());
            }

            // Left stick X
            if (leftX.getAsDouble() != 0) {
                System.out.print("leftX: ");
                System.out.println(leftX.getAsDouble());
            }

            // Left stick Y
            if (leftY.getAsDouble() != 0) {
                System.out.print("leftY: ");
                System.out.println(leftY.getAsDouble());
            }

            // POV Up (already conditional)
            if (povUp.getAsBoolean()) {
                System.out.print("povUp: ");
                System.out.println(true);
            }

            // POV Down
            if (povDown.getAsBoolean()) {
                System.out.print("povDown: ");
                System.out.println(true);
            }

        });
    }
}
