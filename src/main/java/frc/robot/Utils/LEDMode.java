package frc.robot.Utils;

public enum LEDMode{
    BLUE(0), RED(1), GREEN(2), YELLOW(3), RAINBOW(4);

    double mode;
    private LEDMode(double mode){
        this.mode = mode;
    }

    public double getValue(){
        return this.mode;
    }
}