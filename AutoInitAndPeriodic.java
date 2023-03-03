@Override
  public void autonomousInit() {
    // get a time for auton start to do events based on time later
    autoStart = Timer.getFPGATimestamp();
    // check dashboard icon to ensure good to do auto

    /*targetArmAngle = 0.5;
    armAngleThread.start();
    targetExtensionLength = 0.755;
    armExtensionThread.start();*/

    //clawSolenoid1.set(DoubleSolenoid.Value.kReverse);
    //clawSolenoid2.set(DoubleSolenoid.Value.kReverse);

    rampUp(0.4, 0.4, 0.025, 25);

    /*for(double speed = 0; speed <= 0.4; speed += 0.025) {
      if(speed == 0.025) {
        double currTime = Timer.getFPGATimestamp();
        leftAEncoder.setPosition(0.0);
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);

        try {
          Thread.sleep(25);
        } catch(InterruptedException e) {

        }

        experimentalVelocityConstant = 1.0 / ((leftAEncoder.getPosition() * leftAEncoder.getPositionConversionFactor() / (Timer.getFPGATimestamp() - currTime)) * speed);
      } else {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);

        try {
          Thread.sleep(25);
        } catch(InterruptedException e) {

        }
      }
    }*/
    
    rampUp(-0.1, -0.1, 0.05, 15);

    //leftSpeed = -0.1;
    //rightSpeed = -0.1;

    rampUp(-0.5, -0.5, 0.03, 20);

    rampUp(0.0, 0.0, 0.05, 15);

    driveLeftA.set(0);
    driveLeftB.follow(driveLeftA);
    driveRightA.set(0);
    driveRightB.follow(driveRightA);

    rampUp(0.5, 0.5, 0.015, 25);

    rampUp(0, 0, 0.025, 25);

    /**for(double speed = 1; speed >= -1; speed -= 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);
      try {
        Thread.sleep(25);
      } catch(InterruptedException e) {
      }
    }
    if(SmartDashboard.getBoolean("Station 1", true)) {
      station = 1;
      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      }
      for(double speed = 0; speed <= 1; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      }
    } else if(SmartDashboard.getBoolean("Station 2", true)) {
      station = 2;
      for(double speed = 0; speed <= -1; speed -= 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
    } else if(SmartDashboard.getBoolean("Station 3", true)) {
      station = 3;
      for(double speed = -1; speed <= 0; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveLeftA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      }
      for(double speed = 0; speed <= 1; speed += 0.025) {
        driveLeftA.set(speed);
        driveLeftB.follow(driveLeftA);
        driveRightA.set(speed);
        driveRightB.follow(driveRightA);
        try {
          Thread.sleep(10);
        } catch(InterruptedException e) {
        }
      }
      if(Constants.blueAlliance) {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(-1 * speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      } else {
        for(double speed = 0; speed <= 1; speed += 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
        for(double speed = 1; speed >= 0; speed -= 0.025) {
          driveLeftA.set(-1 * speed);
          driveLeftB.follow(driveLeftA);
          driveRightA.set(speed);
          driveRightB.follow(driveRightA);
          try {
            Thread.sleep(10);
          } catch(InterruptedException e) {
          }
        }
      }
    }
    for(double speed = 0; speed <= 1; speed += 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);
      try {
        Thread.sleep(10);
      } catch(InterruptedException e) {
      }
    }
    for(double speed = 1; speed >= 0; speed -= 0.025) {
      driveLeftA.set(speed);
      driveLeftB.follow(driveLeftA);
      driveRightA.set(speed);
      driveRightB.follow(driveRightA);
      try {
        Thread.sleep(10);
      } catch(InterruptedException e) {
        
      }
    }
    driveLeftA.setInverted(false);
    driveLeftA.burnFlash();
    driveLeftB.setInverted(false);
    driveLeftB.burnFlash();
    driveRightA.setInverted(true);
    driveRightA.burnFlash();
    driveRightB.setInverted(true);
    driveRightB.burnFlash();*/
    /*if(!SmartDashboard.getBoolean("Go For Auto", true)) {
      return;
    }
    if(SmartDashboard.getBoolean("Has Cone", true)) {
      //I'm not entirely sure but I'm hoping this will reset the variable value
      SmartDashboard.putBoolean("Has Cone", false);
      scoreConeTop();
    } else if(SmartDashboard.getBoolean("Has Cube", true)) {
      //I might change this to listen to the controller to know when it has a cube/cone and stuff, it depends
      SmartDashboard.putBoolean("Has Cube", false);
      scoreCubeTop();
    }
    //This and other instances of -1 drive power might have to be changed to 1
    //goTo(Constants.autoPieceX, Constants.autoPieceY[station - 1], 180.0);
      
    if(SmartDashboard.getBoolean("Grab Cone", true)) {
      SmartDashboard.putBoolean("Grab Cone", false);
      grabCone();
    } else if(SmartDashboard.getBoolean("Grab Cube", true)) {
      SmartDashboard.putBoolean("Grab Cube", false);
      grabCube();
    }
  //balanceOnTeeter(14.9, false, true, false);*/
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    try {
      Thread.sleep(100);
    } catch(InterruptedException e) {

    }
    /*if (goForAuto) {
      // series of timed events making up the flow of auto
      if (Timer.getFPGATimestamp() - autoStart < 4) {
        // spit out the ball for three seconds
        // intake.set(ControlMode.PercentOutput, -1);
      } else if (Timer.getFPGATimestamp() - autoStart < 7) {
        // stop spitting out the ball and drive backwards *slowly* for three seconds
        // intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      } else {
        // do nothing for the rest of auto
        // intake.set(ControlMode.PercentOutput, 0);
        driveLeftA.set(0);
        driveLeftB.set(0);
        driveRightA.set(0);
        driveRightB.set(0);
      }
    }*/
  }