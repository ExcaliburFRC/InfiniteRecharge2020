package frc.robot.subsystems;

public class BooleanAverager{
    private int location, size;
    private boolean[] bucket;
    private double trueStrength;
    public BooleanAverager(int bucketSize){
        this(bucketSize, 0.5);
    }
    public BooleanAverager(int bucketSize, double trueStrength){
        this.trueStrength = trueStrength;
        this.size = bucketSize;
        this.location = 0;
        reset();
    }

    public void update(boolean val){
        bucket[location] = val;
        location = (location + 1) % size;
    }

    public boolean getAverage(){
        int trueNum = 0;
        for (boolean i : bucket){
            if (i) trueNum++;
        }
        return trueNum > size*trueStrength;
    }

    public void reset(){
        bucket = new boolean[this.size];

        for (int i = 0 ; i < size; i++){
            bucket[i] = false;
        }
    }
}