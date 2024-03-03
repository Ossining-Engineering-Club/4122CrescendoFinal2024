package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.Spark;


public class Leds extends SubsystemBase {
    private final Spark m_Led;

    public Leds(int ledPin) {
        this.m_Led = new Spark(ledPin);
    }

    public void setGreen() {
        m_Led.set(0.77);
    }
    public void setRed() {
        m_Led.set(0.61);
    }

    public void setOrange(){
        m_Led.set(0.65);
    }
    public void setYellow() {
        m_Led.set(0.69);
    }

    
}
