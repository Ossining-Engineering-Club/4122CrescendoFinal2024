package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
    private final String m_name;
    private double[] m_botpose = new double[7];

    public Limelight(String name) {
        m_name = name;
    }

    @Override
    public void periodic() {
        m_botpose = NetworkTableInstance.getDefault().getTable(m_name).getEntry("botpose_wpiblue").getDoubleArray(new double[7]);        
    }

    public boolean hasTarget() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("tv").getDouble(0) == 1;
    }

    public void setPipeline(int pipeline) {
        NetworkTableInstance.getDefault().getTable(m_name).getEntry("pipeline").setNumber(pipeline);
    }

    public double getBotX() {
        return m_botpose[0];
    }
    
    public double getBotY() {
        return m_botpose[1];
    }

    public double getBotZ() {
        return m_botpose[2];
    }

    public double getBotRoll() {
        return m_botpose[3];
    }

    public double getBotPitch() {
        return m_botpose[4];
    }

    public double getBotYaw() {
        return m_botpose[5];
    }

    public double getLatencyMilliseconds() {
        return m_botpose[6];
    }

    public Pose2d getBotPose() {
        return new Pose2d(m_botpose[0], m_botpose[1], Rotation2d.fromDegrees(m_botpose[5]));
    }

    public int getNumTargets() {
        return countStringOccurrences(NetworkTableInstance.getDefault().getTable(m_name).getEntry("json").getString(""), "pts");
    }

    private int countStringOccurrences(String str, String substr) {
        int occurrences = 0;
        for (int i = 0; i < str.length()-substr.length()+1; i++) {
            if (str.substring(i, i+substr.length()).equals(substr)) {
                occurrences++;
            }
        }
        return occurrences;
    }

    public double getTX() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("tx").getDouble(0.0);
    }

    public double getTY() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("ty").getDouble(0.0);
    }

    public double getTA() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("ta").getDouble(0.0);
    }

    public double getScreenspaceX() {
        return NetworkTableInstance.getDefault().getTable(m_name).getEntry("tx0").getDouble(0.0);
    }
}
