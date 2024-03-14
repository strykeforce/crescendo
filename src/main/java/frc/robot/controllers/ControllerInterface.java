package frc.robot.controllers;

public interface ControllerInterface {

  public double getFwd();

  public double getStr();

  public double getYaw();

  public void setRumble(boolean on);
}
