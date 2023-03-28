package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDsystem extends SubsystemBase {
    private AddressableLED strip1;
    // private AddressableLED strip2;
    private AddressableLEDBuffer buffer;
    private boolean state = false;
    
    public LEDsystem(int port1, int port2) {
        strip1 = new AddressableLED(9);
        // strip2=new AddressableLED(port2);
        buffer = new AddressableLEDBuffer(Constants.LED_STRIP_LENGTH);
                                                                    
        strip1.setLength(buffer.getLength());
        // strip2.setLength(buffer.getLength());
        strip1.setData(buffer);
        // strip2.setData(buffer);
        strip1.start();
        // strip2.start();
    }

    public void setColor(boolean color) {
        if (color == state) {
            return;
        }
        if (color) {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 255, 0, 255);
            }
        } else {
            for (var i = 0; i < buffer.getLength(); i++) {
                buffer.setRGB(i, 255, 255, 0);
            }
        }
        state = color;
        strip1.setData(buffer);
        // strip2.setData(buffer);
    }

    public void turnOff() {
        for (var i = 0; i < buffer.getLength(); i++) {
            buffer.setRGB(i, 0, 0, 0);
        }
        strip1.setData(buffer);
        // strip2.setData(buffer);
    }
}
