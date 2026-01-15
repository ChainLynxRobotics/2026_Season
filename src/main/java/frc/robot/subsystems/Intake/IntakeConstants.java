package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.measure.Angle;

public class IntakeConstants{
    public static Angle kIntakeLowAngle;
    public static Angle kIntakeHighAngle;
    
    public static Slot0Configs intakeHeightSlot0Config = new Slot0Configs();

    public enum IntakeState{
        HIGH(kIntakeHighAngle),LOW(kIntakeLowAngle);

        public final Angle angle;

        IntakeState(Angle angle){
            this.angle=angle;
        }

        public Angle getAngle(){
            return angle;
        }
    }
}