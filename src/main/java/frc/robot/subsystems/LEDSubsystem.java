// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.LarsonBounceValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  private final CANdle candle;
  private final CANdleConfiguration candelConfig;
  private final StrobeAnimation strobeAnimation;
  private final FireAnimation fireAnimation;
  private final LarsonAnimation larsonAnimation;
  private final SingleFadeAnimation singleFadeAnimation;
  private final SolidColor solidColor;

  public LEDSubsystem() {
    candle = new CANdle(LEDConstants.candle_ID);
    candelConfig = new CANdleConfiguration();
    strobeAnimation = new StrobeAnimation(0, LEDConstants.ledNumber - 1);
    fireAnimation = new FireAnimation(0, LEDConstants.ledNumber - 1);
    solidColor = new SolidColor(0, LEDConstants.ledNumber - 1);
    larsonAnimation = new LarsonAnimation(0, LEDConstants.ledNumber - 1);
    singleFadeAnimation = new SingleFadeAnimation(0, LEDConstants.ledNumber - 1);

    candelConfig.LED.StripType = StripTypeValue.GRB;
    candelConfig.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
    candelConfig.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;

    candle.getConfigurator().apply(candelConfig);

  }

  public void hasTarget(){
    candle.setControl(new EmptyAnimation(0));
    candle.setControl(solidColor.withColor(new RGBWColor(0, 127, 0)));
  }

  public void shootHUB(){
    candle.setControl(fireAnimation.withDirection(AnimationDirectionValue.Forward).withBrightness(1).withCooling(0.2).withSparking(1).withFrameRate(20).withUpdateFreqHz(80));
  }

  public void shootHome(){
    candle.setControl(larsonAnimation.withBounceMode(LarsonBounceValue.Center).withColor(new RGBWColor(255, 0, 150)).withSize(10).withFrameRate(120).withUpdateFreqHz(200));
  }

  public void intake(){
    candle.setControl(strobeAnimation.withColor(new RGBWColor(127, 0, 127)).withFrameRate(10));
  }

  public void disable(){
    candle.setControl(singleFadeAnimation.withColor(new RGBWColor(255, 0, 150)).withFrameRate(30).withUpdateFreqHz(80));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(LEDConstants.ledFlag){
      LEDConstants.ledFlag = false;
      if(LEDConstants.isEnable){
        if (LEDConstants.hasTarget){    
          hasTarget();
        }
        else if(LEDConstants.shootHUB) shootHUB();
        else if(LEDConstants.shootHome) shootHome();
        else if(LEDConstants.intake) intake();
        else if(LEDConstants.outAlliance) shootHome();
        else  shootHUB();
      }else disable();
    }

    SmartDashboard.putBoolean("LED/Flag", LEDConstants.ledFlag);
    SmartDashboard.putBoolean("LED/hasTarget", LEDConstants.hasTarget);
  }
}
