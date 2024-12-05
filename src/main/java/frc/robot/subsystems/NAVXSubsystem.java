package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;

public class NAVXSubsystem {
    public AHRS ahrs;

    public NAVXSubsystem() {
        _init_();
    }

    public void _init_() {
        try {
            ahrs = new AHRS(SPI.Port.kMXP);
        } 
        catch (RuntimeException ex) {
            DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
        }
    }

    public double getAngle() {
        return Double.valueOf(Constants.ModuleConstants.decimalFormat.format(ahrs.getAngle()));
    }
}
