// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class EightBitDoController extends CommandGenericHID {
  public static final int EIGHT_BIT_DO_A = 1;
  public static final int EIGHT_BIT_DO_B = 2;
  public static final int EIGHT_BIT_DO_X = 4;
  public static final int EIGHT_BIT_DO_Y = 5;
  public static final int EIGHT_BIT_DO_LEFT_BUMPER = 7;
  public static final int EIGHT_BIT_DO_RIGHT_BUMPER = 8;
  public static final int EIGHT_BIT_DO_LEFT_TRIGGER = 9;
  public static final int EIGHT_BIT_DO_RIGHT_TRIGGER = 10;
  public static final int EIGHT_BIT_DO_MINUS = 11;
  public static final int EIGHT_BIT_DO_PLUS = 12;
  public static final int EIGHT_BIT_DO_LEFT_JOYSTICK_CLICK = 14;
  public static final int EIGHT_BIT_DO_RIGHT_JOYSTICK_CLICK = 15;
  private final XboxController m_hid;

  public EightBitDoController(int port) {
    super(port);
    m_hid = new XboxController(port);
  }

  public Trigger a() { return button(EIGHT_BIT_DO_A); }
  public Trigger b() { return button(EIGHT_BIT_DO_B); }
  public Trigger x() { return button(EIGHT_BIT_DO_X); }
  public Trigger y() { return button(EIGHT_BIT_DO_Y); }

  public Trigger leftBumper() { return button(EIGHT_BIT_DO_LEFT_BUMPER); }
  public Trigger rightBumper() { return button(EIGHT_BIT_DO_RIGHT_BUMPER); }
  public Trigger leftTrigger() { return button(EIGHT_BIT_DO_LEFT_TRIGGER); }
  public Trigger rightTrigger() { return button(EIGHT_BIT_DO_RIGHT_TRIGGER); }

  public Trigger pressLeftJoystick() {return button(EIGHT_BIT_DO_LEFT_JOYSTICK_CLICK);}
  public Trigger pressRightJoystick() {return button(EIGHT_BIT_DO_RIGHT_JOYSTICK_CLICK);}

  public double getLeftX() {
    return m_hid.getRawAxis(0);
  }

  public double getLeftY() {
    return m_hid.getRawAxis(1);
  }

  public double getRightX() {
    return m_hid.getRawAxis(3);
  }

  public double getRightY() {
    return m_hid.getRawAxis(4);
  }



}
