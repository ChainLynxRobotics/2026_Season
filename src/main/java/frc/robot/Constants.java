package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.units.measure.*;

public class Constants {
  public static final Time kDT = Seconds.of(0.02);
  public static final boolean tuningMode = true;
  public static final CANBus kCanBus = new CANBus("blinky");
}
