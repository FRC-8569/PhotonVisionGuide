package frc.robot.Drivetrain;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Vision.Vision;
import frc.utils.FieldPieces;
import frc.utils.FieldPieces.ReefSide;

public class Drivetrain extends SubsystemBase{
    public SparkMax LeftMotor, LeftBackMotor, RightMotor, RightBackMotor;
    public RelativeEncoder LeftEncoder, RightEncoder;
    public SparkClosedLoopController LeftPID, RightPID;
    private SparkMaxConfig LeftConfig, LeftBackConfig, RightConfig, RightBackConfig;
    
    public AHRS gyro;
    public DifferentialDrivePoseEstimator PoseEstmator;
    public Vision vision;

    public StructPublisher<ChassisSpeeds> TargetSpeeds, CurrentSpeeds;
    public StructArrayPublisher<SwerveModuleState> WheelState;
    public StructPublisher<Pose2d> RobotPose;
    public Field2d FieldEasy;

    public SparkMaxSim LeftMotorSim, RightMotorSim;
    public SparkRelativeEncoderSim LeftEncdoerSim, RightEncoderSim;
    public DifferentialDrivetrainSim SimSystem;
    public Notifier SimNotifier;

    private Drivetrain(){
        LeftMotor = new SparkMax(Constants.Motors[0], MotorType.kBrushed);
        LeftBackMotor = new SparkMax(Constants.Motors[1], MotorType.kBrushed);
        RightMotor = new SparkMax(Constants.Motors[2], MotorType.kBrushed);
        RightBackMotor = new SparkMax(Constants.Motors[3], MotorType.kBrushed);
        LeftEncoder = LeftMotor.getEncoder();
        RightEncoder = RightMotor.getEncoder();
        LeftPID = LeftMotor.getClosedLoopController();
        RightPID = RightMotor.getClosedLoopController();
        
        gyro = new AHRS(NavXComType.kMXP_SPI);
        PoseEstmator = new DifferentialDrivePoseEstimator(Constants.kinematics, gyro.getRotation2d(), getPositions().leftMeters, getPositions().rightMeters, Constants.InitialPose);
        vision = Vision.getInstance();

        TargetSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/TargetSpeeds", ChassisSpeeds.struct).publish();
        CurrentSpeeds = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/CurrentSpeeds", ChassisSpeeds.struct).publish();
        WheelState = NetworkTableInstance.getDefault().getStructArrayTopic("Drivetrain/WheelState", SwerveModuleState.struct).publish();
        RobotPose = NetworkTableInstance.getDefault().getStructTopic("Drivetrain/RobotPose", Pose2d.struct).publish();
        FieldEasy = new Field2d();
        SmartDashboard.putData(FieldEasy);

        LeftConfig = new SparkMaxConfig();
        LeftBackConfig = new SparkMaxConfig();
        RightConfig = new SparkMaxConfig();
        RightBackConfig = new SparkMaxConfig();

        LeftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        LeftConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        LeftConfig.apply(Constants.LeftPIDConfig);
        
        LeftBackConfig.follow(LeftMotor);

        RightConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .voltageCompensation(12)
            .smartCurrentLimit(40);
        RightConfig.encoder
            .positionConversionFactor(Constants.PositionConvertionFactor)
            .velocityConversionFactor(Constants.VelocityConvertionFactor);
        RightConfig.apply(Constants.RightPIDConfig);
        
        RightBackConfig.follow(RightMotor);

        LeftMotor.configure(LeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        LeftBackMotor.configure(LeftBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        RightMotor.configure(RightConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        RightBackMotor.configure(RightBackConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        autoInit();
        if(RobotBase.isSimulation()) simInit();
    }

    public DifferentialDriveWheelPositions getPositions(){
        return new DifferentialDriveWheelPositions(
            LeftEncoder.getPosition(), 
            RightEncoder.getPosition());
    }

    public DifferentialDriveWheelSpeeds getVelocity(){
        return new DifferentialDriveWheelSpeeds(
            LeftEncoder.getVelocity(),
            RightEncoder.getVelocity()
        );
    }

    public void drive(double leftMPS, double rightMPS){
        if(RobotBase.isReal()){
            LeftPID.setReference(leftMPS, ControlType.kVelocity);
            RightPID.setReference(rightMPS, ControlType.kVelocity);
        }else{
            LeftMotorSim.setAppliedOutput(leftMPS/Constants.MaxVelocity);
            RightMotorSim.setAppliedOutput(rightMPS/Constants.MaxVelocity);
        }
        TargetSpeeds.accept(Constants.kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMPS, rightMPS)));
    }

    public Command drive(Supplier<Double> Throttle, Supplier<Double> Rotation){
        return run(() -> drive((Throttle.get()+Rotation.get())*Constants.MaxVelocity, (Throttle.get()-Rotation.get())*Constants.MaxVelocity));
    }

    public void drive(ChassisSpeeds speeds){
        drive(Constants.kinematics.toWheelSpeeds(speeds).leftMetersPerSecond,Constants.kinematics.toWheelSpeeds(speeds).rightMetersPerSecond);
    }

    public Command drive(Pose2d pose){
        try{
            return new PathfindingCommand(
            pose, 
            Constants.constraints, 
            () -> this.PoseEstmator.getEstimatedPosition(), 
            () -> Constants.kinematics.toChassisSpeeds(getVelocity()), 
            (speeds ,ff) -> drive(speeds),new PPLTVController(VecBuilder.fill(0.0625, 0.0625, .0625), VecBuilder.fill(.5, 1), 0.02, Constants.MaxVelocity), 
            RobotConfig.fromGUISettings(), 
            this);
        }catch(Exception e){
            DriverStation.reportWarning("The path cannot generated with ", e.getStackTrace());
            return Commands.none();
        }
    }

    public Command drive(FieldPieces pieces, ReefSide side){
        return drive(pieces.getPose(PoseEstmator.getEstimatedPosition()).plus(side.getOffset()));
    }

    public void resetPose(Pose2d pose){
        PoseEstmator.resetPose(pose);
    }

    @Override
    public void periodic(){
        PoseEstmator.update(gyro.getRotation2d(), getPositions());
        CurrentSpeeds.accept(Constants.kinematics.toChassisSpeeds(getVelocity()));
        WheelState.accept(getState());
        RobotPose.accept(PoseEstmator.getEstimatedPosition());
        Pose2d pose = vision.getPose();
        if(pose != null && RobotBase.isReal()) PoseEstmator.addVisionMeasurement(pose, RobotController.getFPGATime());
        FieldEasy.setRobotPose(PoseEstmator.getEstimatedPosition());
    }

    private void autoInit(){
        try{
            AutoBuilder.configure(
                () -> PoseEstmator.getEstimatedPosition(), 
                this::resetPose, 
                () -> Constants.kinematics.toChassisSpeeds(getVelocity()), 
                (speeds, ff) -> drive(speeds), 
                new PPLTVController(VecBuilder.fill(0.0625, 0.0625, .0625), VecBuilder.fill(.5, 1), 0.02, Constants.MaxVelocity), 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElseThrow() == Alliance.Red, 
                this);
        }catch(Exception e){
            DriverStation.reportError("Fuced up at loading PathPlanner with", e.getStackTrace());
        }
    }

    private void simInit(){
        LeftMotorSim = new SparkMaxSim(LeftMotor, DCMotor.getCIM(2));
        RightMotorSim = new SparkMaxSim(RightMotor, DCMotor.getCIM(2));
        LeftEncdoerSim = LeftMotorSim.getRelativeEncoderSim();
        RightEncoderSim = RightMotorSim.getRelativeEncoderSim();
        SimSystem = DifferentialDrivetrainSim.createKitbotSim(
            KitbotMotor.kDualCIMPerSide, 
            KitbotGearing.k10p71, 
            KitbotWheelSize.kSixInch, 
            null);
        SimNotifier = new Notifier(this::simUpdate);
        SimNotifier.setName("SimNotifier");
        SimNotifier.startPeriodic(0.02);
    }
    private void simUpdate(){
        SimSystem.setInputs(LeftMotor.getAppliedOutput()*12, RightMotor.getAppliedOutput()*12);
        LeftEncdoerSim.setVelocity(SimSystem.getLeftVelocityMetersPerSecond());
        RightEncoderSim.setVelocity(SimSystem.getRightVelocityMetersPerSecond());
        LeftEncdoerSim.setPosition(SimSystem.getLeftPositionMeters());
        RightEncoderSim.setPosition(SimSystem.getRightPositionMeters());
        gyro.setAngleAdjustment(-SimSystem.getHeading().getDegrees());
        SimSystem.update(0.02);
    }

    private SwerveModuleState[] getState(){
        return new SwerveModuleState[]{
            new SwerveModuleState(getVelocity().leftMetersPerSecond, Rotation2d.kZero),
            new SwerveModuleState(getVelocity().rightMetersPerSecond, Rotation2d.kZero),
            new SwerveModuleState(getVelocity().leftMetersPerSecond, Rotation2d.kZero),
            new SwerveModuleState(getVelocity().rightMetersPerSecond, Rotation2d.kZero)
        };
    }

    public static Drivetrain getInstance(){
        return new Drivetrain();
    }
}
