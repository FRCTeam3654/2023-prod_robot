add all documentations for season 2020 robot in this folder


     addCommands(
          new ParallelCommandGroup(
            new ArmSetPositionsCommand(2000), // raise arm a little bit, 1.5 seconds
            new  AutoWrist(1) // lowers wrist , 2 seconds
          )
     );

     addCommands( new AutoPneumatics(1 ) ); // opens pnematic to drop, 1 second

     addCommands(
          new ParallelCommandGroup(
            new AutoWrist(2), // raise wrist, 2 seconds, don't wait for full 2 seoonds to do next command
            new  SequentialCommandGroup (
              new WaitCommand(1),   // wait for 1 second for wrist to raise above group
              new ParallelCommandGroup(
                new AutoPneumatics(2),  // 1 second
                new  SequentialCommandGroup(
                    mp,                           // estimate about 4 seconds: 1.3 meter/second x 4 = 5.2 meter (~157 inches), after ~ 4 seconds in autonomous
                    new WaitCommand(1),   // wait for 1 second for the balance swing back to nornal
                    new AutoBalanceCommand()       // about 5 to 6 seconds left to auto balance
                )
              )
            )
          )
     );

