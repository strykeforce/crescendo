package frc.robot.commands.robotState;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.climb.ZeroClimbCommand;
import frc.robot.commands.drive.LockZeroCommand;
import frc.robot.subsystems.climb.ClimbIOFX;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ForkIOSRX;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.elbow.ElbowIOFX;
import frc.robot.subsystems.elbow.ElbowSubsystem;
import frc.robot.subsystems.intake.IntakeIOFX;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.magazine.MagazineIOFX;
import frc.robot.subsystems.magazine.MagazineSubsystem;
import frc.robot.subsystems.shooter.ShooterIOFX;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.wrist.WristIOSRX;
import frc.robot.subsystems.wrist.WristSubsystem;
import java.util.List;
import org.strykeforce.healthcheck.IOHealthCheckCommand;

public class AirwaveHealthCheck extends SequentialCommandGroup {
  private Swerve swerve;
  private DriveSubsystem driveSubsystem;
  private IntakeIOFX intakeIOFX;
  private IntakeSubsystem intakeSubsystem;
  private MagazineIOFX magazineIOFX;
  private MagazineSubsystem magazineSubsystem;
  private ShooterIOFX shooterIOFX;
  private ShooterSubsystem shooterSubsystem;
  private ElbowIOFX elbowIOFX;
  private ElbowSubsystem elbowSubsystem;
  private WristIOSRX wristIOSRX;
  private WristSubsystem wristSubsystem;
  private ClimbIOFX climbIOFX;
  private ForkIOSRX forkIOSRX;
  private ClimbSubsystem climbSubsystem;

  public AirwaveHealthCheck(
      Swerve swerve,
      DriveSubsystem driveSubsystem,
      IntakeIOFX intakeIOFX,
      IntakeSubsystem intakeSubsystem,
      MagazineIOFX magazineIOFX,
      MagazineSubsystem magazineSubsystem,
      ShooterIOFX shooterIOFX,
      ShooterSubsystem shooterSubsystem,
      ElbowIOFX elbowIOFX,
      ElbowSubsystem elbowSubsystem,
      WristIOSRX wristIOSRX,
      WristSubsystem wristSubsystem,
      ClimbIOFX climbIOFX,
      ForkIOSRX forkIOSRX,
      ClimbSubsystem climbSubsystem) {

    this.swerve = swerve;
    this.driveSubsystem = driveSubsystem;
    this.intakeIOFX = intakeIOFX;
    this.intakeSubsystem = intakeSubsystem;
    this.magazineIOFX = magazineIOFX;
    this.magazineSubsystem = magazineSubsystem;
    this.shooterIOFX = shooterIOFX;
    this.shooterSubsystem = shooterSubsystem;
    this.elbowIOFX = elbowIOFX;
    this.elbowSubsystem = elbowSubsystem;
    this.wristIOSRX = wristIOSRX;
    this.wristSubsystem = wristSubsystem;
    this.climbIOFX = climbIOFX;
    this.forkIOSRX = forkIOSRX;
    this.climbSubsystem = climbSubsystem;

    addCommands(
        new ZeroClimbCommand(climbSubsystem),
        new IOHealthCheckCommand(
            List.of(
                driveSubsystem,
                intakeSubsystem,
                magazineSubsystem,
                shooterSubsystem,
                elbowSubsystem,
                wristSubsystem,
                climbSubsystem),
            swerve,
            intakeIOFX,
            magazineIOFX,
            elbowIOFX,
            wristIOSRX,
            climbIOFX,
            forkIOSRX),
        new LockZeroCommand(driveSubsystem));
  }
}
