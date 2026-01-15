package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase{
    private TalonFX indexerMotor = new TalonFX(0);

    public IndexerSubsystem(){

    } 
   
    @Logged
    public double getIndexerPosition(){
        return indexerMotor.getPosition().getValueAsDouble();
    }

    @Logged
    public double getIndexerVelocity(){
        return indexerMotor.getVelocity().getValueAsDouble();
    }

    public Command spin(){
        return run(() -> indexerMotor.setVoltage(0));
    }
}