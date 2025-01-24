package frc.robot.commands.auton;

import WallEye.Point;
import WallEye.WallEyeTagResult;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.led.LedSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class TestWalleyeAlignCommand extends Command {
  private VisionSubsystem walleye;
  private DriveSubsystem driveSubsystem;
  private ProfiledPIDController walleyeYDrive;
  private ProfiledPIDController walleyeXDrive;
  private ProfiledPIDController walleyeOmega;
  private LedSubsystem ledSubsystem;
  private AlignStates curState;
  private final int CAM = 0;
  private final int ALIGN_TAG = 1;
  private final double TARGET_YAW_DEGS = 0;
  private final double TARGET_TAG_AREA = 205_000;
  private final double TARGET_X = 800;

  public TestWalleyeAlignCommand(
      VisionSubsystem walleye, DriveSubsystem driveSubsystem, LedSubsystem ledSubsystem) {
    addRequirements(driveSubsystem);
    this.walleye = walleye;
    this.driveSubsystem = driveSubsystem;
    this.ledSubsystem = ledSubsystem;
    this.curState = AlignStates.YAW;

    walleyeYDrive = new ProfiledPIDController(0.0019, 0, 0, new Constraints(3.0, 3.0));
    walleyeXDrive = new ProfiledPIDController(0.00001, 0, 0, new Constraints(2.0, 1.0));
    walleyeOmega = new ProfiledPIDController(5.0, 0, 0, new Constraints(1.0, 1.0));
    walleyeOmega.enableContinuousInput(Math.toRadians(-180), Math.toRadians(180));
  }

  @Override
  public void initialize() {
    ledSubsystem.setColor(120, 38, 109);
    walleyeOmega.reset(driveSubsystem.getPoseMeters().getRotation().getRadians());

    driveSubsystem.setIsAligningShot(false);
    this.curState = AlignStates.YAW;
  }

  @Override
  public void execute() {
    switch (curState) {
      case YAW:
        driveSubsystem.move(0, 0, calculateSpinToTarget(), false);

        if (Math.abs(
                driveSubsystem.getPoseMeters().getRotation().getRadians()
                    - Units.degreesToRadians(TARGET_YAW_DEGS))
            < Units.degreesToRadians(1)) {
          curState = AlignStates.DRIVE;
        }
        break;

      case DRIVE:
        WallEyeTagResult result = (WallEyeTagResult) walleye.getLastResult(CAM);

        if (result.getNumTags() > 0) {
          int tagIndex = -1;
          int[] tags = result.getTagIDs();

          for (int i = 0; i < tags.length; i++) {
            if (tags[i] == ALIGN_TAG) {
              tagIndex = i;
              break;
            }
          }

          if (tagIndex == -1) {
            return;
          }

          Point center = result.getTagCenters().get(tagIndex);
          double area = result.getTagAreas()[tagIndex];

          org.littletonrobotics.junction.Logger.recordOutput("VisionSubsystem/TargetArea", area);

          double ySpeed = -walleyeYDrive.calculate(TARGET_X - center.x(), 0);
          double xSpeed = -walleyeXDrive.calculate(TARGET_TAG_AREA - area, 0);

          if (TARGET_TAG_AREA - area < 10_000 || area > TARGET_TAG_AREA) {
            xSpeed = 0;

            if (Math.abs(TARGET_X - center.x()) < 3) {
              curState = AlignStates.DONE;
            }
          }

          driveSubsystem.move(xSpeed, ySpeed, calculateSpinToTarget(), false);
        }
        break;
      case DONE:
        ledSubsystem.setColor(0, 200, 0);
        break;
    }
  }

  private double calculateSpinToTarget() {
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Yaw Error", walleyeOmega.getPositionError());
    org.littletonrobotics.junction.Logger.recordOutput(
        "DriveSubsystem/Setpoint", walleyeOmega.getSetpoint().position);

    return walleyeOmega.calculate(
        driveSubsystem.getPoseMeters().getRotation().getRadians(),
        Units.degreesToRadians(TARGET_YAW_DEGS));
  }

  @Override
  public boolean isFinished() {
    return curState == AlignStates.DONE;
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.move(0, 0, 0.0, false);
  }

  private enum AlignStates {
    YAW,
    DRIVE,
    DONE
  }
}
