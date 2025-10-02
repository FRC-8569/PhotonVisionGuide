package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Shooter {
    public SparkMax ShooterMotor;
    public Command shoot(){
        return Commands.none().withTimeout(Seconds.of(1));
    }
}
