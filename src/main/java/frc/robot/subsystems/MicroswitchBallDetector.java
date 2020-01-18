package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;

public class MicroswitchBallDetector implements BallDetector{
    private DigitalInput detectorSwitch;

    public MicroswitchBallDetector(int port){
        detectorSwitch = new DigitalInput(port);
    }

    @Override
    public boolean isBallDetected() {
        return detectorSwitch.get();
    }

}