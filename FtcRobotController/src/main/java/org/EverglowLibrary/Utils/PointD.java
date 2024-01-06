package org.EverglowLibrary.Utils;

public class PointD {
    private double x;
    private double y;

    public PointD(double x, double y){
        this.x = x;
        this.y = y;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public PointD add(PointD anotherPoint){
        return new PointD(x+anotherPoint.x, y+anotherPoint.y);
    }
}
