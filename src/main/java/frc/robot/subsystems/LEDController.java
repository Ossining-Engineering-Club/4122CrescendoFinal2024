package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.NoteState;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLED;


public class LEDController extends SubsystemBase {

    private final AddressableLED m_addressable;
    private final AddressableLEDBuffer m_addressableBuffer;

    private boolean state;

    private NoteState m_noteState = NoteState.EMPTY;

    ////////////

    public LEDController(int pwmPin, int ledCount) {
        m_addressable = new AddressableLED(pwmPin);
        m_addressableBuffer = new AddressableLEDBuffer(ledCount);

        this.state = false;

        m_addressable.setLength(m_addressableBuffer.getLength());
        m_addressable.setData(m_addressableBuffer);
        m_addressable.start();

        addressableRed();
    }
    

    public void addressableGreen() {
        for(int i = 0; i<m_addressableBuffer.getLength(); i++) {
            m_addressableBuffer.setRGB(i, 0,150,0);
            m_addressable.setData(m_addressableBuffer);
        }
    }
    public void addressableRed() {
        for(int i = 0; i<m_addressableBuffer.getLength(); i++) {
            m_addressableBuffer.setRGB(i, 150,0,0);
             m_addressable.setData(m_addressableBuffer);
        }
    }
    public void addressableOff() {
        for(int i = 0; i<m_addressableBuffer.getLength(); i++) {
            m_addressableBuffer.setRGB(i, 0,0,0);
             m_addressable.setData(m_addressableBuffer);
        }
    }
    public void addressableWhite() {
        for(int i = 0; i<m_addressableBuffer.getLength(); i++) {
            m_addressableBuffer.setRGB(i, 150,150,150);
            m_addressable.setData(m_addressableBuffer);
        }
    }

    public void setState(NoteState newState) {
        m_noteState = newState;
    }

    public void updateLEDs() {
        switch (m_noteState) {
            case EMPTY:
                addressableRed();
                break;
            case INTAKING:
            case INTERMEDIATE:
            case READY_TO_SHOOT:
                addressableGreen();
                break;
        }
    }

    @Override
    public void periodic() {
        updateLEDs();
    }
    
}
