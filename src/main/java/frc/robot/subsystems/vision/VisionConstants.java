package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.vision.Vision.CameraConfig;

public class VisionConstants {
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  public static final CameraConfig FRONT_CAMERA =
      new CameraConfig(
            "Arducam_OV9281_USB_Camera",
            new Transform3d(
                // new Translation3d(-0.09, -0.004, .4587), // x, y, z meters
                // new Rotation3d(0.0, Units.degreesToRadians(-4.0), Units.degreesToRadians(180.0)))); // roll, pitch, yaw radians
                new Translation3d(-0.246, 0.19, 0.208),
                new Rotation3d(0, Units.degreesToRadians(-16.875), Units.degreesToRadians(180.0))));

  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(1.5, 1.5, 7);
  public static final Matrix<N3, N1> MULTIPLE_TAG_STD_DEVS = VecBuilder.fill(0.3, 0.3, 4);

  public static final boolean IGNORE_YAW = true;

  public static final double MAX_HEIGHT = 0.305;
  public static final double MAX_ANGLE = 0.3;
}