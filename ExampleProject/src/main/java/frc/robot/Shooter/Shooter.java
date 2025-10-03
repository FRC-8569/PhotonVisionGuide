package frc.robot.Shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{
    public SparkMax ShooterMotor;
    public RelativeEncoder Encoder;
    public static Shooter shooter;
    private SparkMaxConfig config;
    public StringPublisher NowDoing;

    private Shooter(){
        ShooterMotor = new SparkMax(Constants.ShooterID, MotorType.kBrushless);
        NowDoing = NetworkTableInstance.getDefault().getTable("RobotDoing").getStringTopic("Shooter").publish();
        Encoder = ShooterMotor.getEncoder();

        config = new SparkMaxConfig();

        config
            .idleMode(IdleMode.kCoast)
            .inverted(false)
            .voltageCompensation(12);

        ShooterMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        NowDoing.setDefault("null");
    }

    public Command shoot(){
        return runEnd(
            () -> {
                ShooterMotor.set(0.8);
                NowDoing.accept("正在轉");
            },
            () -> {
                ShooterMotor.stopMotor();
                NowDoing.set("停下來");
            }).withTimeout(Seconds.of(0.5));
    }
    
    public static Shooter getInstance(){
        if(shooter == null) shooter = new Shooter();
        return shooter;
    }
}
