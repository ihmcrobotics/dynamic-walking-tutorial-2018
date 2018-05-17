package us.ihmc.robotArmOne;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotArmOneSimulation
{
   public RobotArmOneSimulation()
   {
      // Create an instance of the robot arm.
      RobotArmOne robotArm = new RobotArmOne();
      // Create an instance of the controller.
      RobotArmOneController robotArmController = new RobotArmOneController(robotArm);
      // Make sure to initialize the controller.
      robotArmController.initialize();
      // Attach the controller to the robot.
      robotArm.setController(robotArmController);

      // Creating the simulation.
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArm);

      scs.setSimulateNoFasterThanRealTime(true);
      // 
      scs.setDT(1.0e-4, 10);
      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.setMaxBufferSize(65536);
      // Launch the simulator.
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new RobotArmOneSimulation();
   }
}
