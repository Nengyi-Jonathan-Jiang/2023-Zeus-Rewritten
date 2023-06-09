package frc.robot.misc;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;

import java.util.Objects;

public class LEDStrip {
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;
    private final int length;

    public LEDStrip(int port, int length){
        this.length = length;

        led = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);
        led.setLength(length);
        led.setData(buffer);
        led.start();
    }

    public static final class LEDPattern {
        public final Color8Bit[] colors;
        public final int runLength;

        public LEDPattern(Color8Bit[] colors, int runLength) {
            this.colors = colors;
            this.runLength = runLength;
        }

        public LEDPattern(int runLength, Color8Bit... colors) {
            this.colors = colors;
            this.runLength = runLength;
        }

        public static LEDPattern createRainbowPattern(int cycleSize) {
            Color8Bit[] colors = new Color8Bit[cycleSize];
            for (int i = 0; i < cycleSize; i++) {
                Color hsv = Color.fromHSV(180 * i / cycleSize, 255, 255);
                colors[i] = new Color8Bit(hsv);
            }
            return new LEDPattern(colors, 1);
        }

        public Color8Bit get(int led){
            int colorIndex = led / runLength % colors.length;
            return colors[colorIndex];
        }
    }

    /**
     * Sets the LED strip to a repeating pattern. For example,
     * <pre>setPattern([a, b], 4, 1)</pre>
     * results in the LED strip being set to
     * <pre>b, a, a, a, a, b, b, b, b, a, a, a, a, b, ...</pre>
     * <br>
     */
    public void setPattern(LEDPattern pattern, int offset){
        for(int i = 0; i < length; i++){
            Color8Bit color = pattern.get(math.mod(i - offset, length));
            buffer.setLED(i, color);
        }
    }

    public void setColor(Color8Bit c){
        setColor(c, 0, length);
    }
    public void setColor(Color8Bit c, int id) {
        setColor(c, id, 1);
    }
    public void setColor(Color8Bit c, int start, int length) {
        for(int i = 0; i < length; i++){
            buffer.setLED(i + start, c);
        }
    }
    public void setRGB(int r, int g, int b){
        setColor(new Color8Bit(r, g, b));
    }
    public void setRGB(int r, int g, int b, int id) {
        setColor(new Color8Bit(r, g, b), id);
    }
    public void setRGB(int r, int g, int b, int id, int length) {
        setColor(new Color8Bit(r, g, b), id, length);
    }
    public void setHSV(float h, float s, float v){
        setHSV(h, s, v, 0, length);
    }
    public void setHSV(float h, float s, float v, int id) {
        setHSV(h, s, v, id, 1);
    }
    public void setHSV(float h, float s, float v, int start, int length) {
        Color8Bit hsv = new Color8Bit(Color.fromHSV((int)(h * 180), (int)(s * 255), (int)(v * 255)));
        setColor(hsv, start, length);
    }
}
