package frc.robot.subsystems.LED;

public class LED{
    public static final int NUM_LEDS = 36;


    private static LED instance;

    public static LED getInstance() {

        if (instance == null) {
            instance = new LED(new LEDIOSim());
        }

        return instance;
    }

    private final LEDIO io;
    private LEDIOInputsAutoLogged inputs = new LEDIOInputsAutoLogged() ;
    private int[] buffer = new int[NUM_LEDS * 3];

    private LED(LEDIO io) {
        this.io = io;
    }

    public void setRGB(int i, int r, int g, int b) {
        buffer[i] = r;        
        buffer[i+1] = g;
        buffer[i+2] = b;
    }

    public void setHSV(int i, int h, int s, int v) {
        buffer[i] = h;        
        buffer[i+1] = s;
        buffer[i+2] = v;
    }

    public void setRangeRGB(int low, int high, int r, int g, int b) {
        for (int i=low; i<high; i+=3) {
            buffer[i] = r;
            buffer[i+1] = g;
            buffer[i+2] = b;
        }
    }

    public void setRangeHSV(int low, int high, int h, int s, int v) {
        for (int i=low; i<high; i+=3) {
            buffer[i] = h;
            buffer[i+1] = s;
            buffer[i+2] = v;
        }
    }
    
    public void setAllRGB(int total, int r, int g, int b) {
        for (int i=0; i<total; i+=3) {
            buffer[i] = r;
            buffer[i+1] = g;
            buffer[i+2] = b;
        }
    }

    public void setAllHSV(int total, int h, int s, int v) {
        for (int i=0; i<total; i+=3) {
            buffer[i] = h;
            buffer[i+1] = s;
            buffer[i+2] = v;
        }
    }

    public void angleToPositionRGB(int angle, int r, int b, int g) {
        buffer[angle/10] = r;
        buffer[angle/10+1] = g;
        buffer[angle/10+2] = b;
    }

        public void angleToPositionHSV(int angle, int h, int s, int v) {
        buffer[angle/10] = h;
        buffer[angle/10+1] = s;
        buffer[angle/10+2] = v;
    }

    public void setfixedPosition() {}

    public void render() {
        io.updateInputs(inputs);

        io.setColours(buffer);
    }

}
