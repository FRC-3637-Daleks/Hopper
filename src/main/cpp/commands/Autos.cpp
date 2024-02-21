// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/Autos.h"
#include <frc2/command/Commands.h>


frc2::CommandPtr autos::ExampleAuto(Shooter* subsystem) {
  return frc2::cmd::Sequence(frc2::cmd::Wait(1_s),
                             frc2::cmd::Print("Hello World!"));
}
