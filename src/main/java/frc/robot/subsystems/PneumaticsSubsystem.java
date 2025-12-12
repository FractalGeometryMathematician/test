package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PneumaticsSubsystem extends SubsystemBase {
    private final DoubleSolenoid doubleSolenoid = new DoubleSolenoid(
            Constants.PneumaticsConstants.pneumaticCANId,
            PneumaticsModuleType.CTREPCM,
            Constants.PneumaticsConstants.pneumaticForwardChannel,
            Constants.PneumaticsConstants.pneumaticReverseChannel);
            
    public PneumaticsSubsystem() {
        // Starts with pneumatic retracted
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

       /**
     * Toggles solenoid between extended and retracted 
     * If extended, it'll retract & if retracted, it'll extend.
     */
    public void togglePneumatic() {
        doubleSolenoid.toggle();
    }
    public void extend() {
        doubleSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void retract() {
        doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public boolean isExtended() {
        return doubleSolenoid.get() == DoubleSolenoid.Value.kForward;
    }
}