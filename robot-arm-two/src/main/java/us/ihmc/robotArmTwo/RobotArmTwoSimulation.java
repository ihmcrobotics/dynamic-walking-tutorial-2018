package us.ihmc.robotArmTwo;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotArmOne.RobotArmOne;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotArmTwoSimulation
{
   public RobotArmTwoSimulation()
   {
      double gravityMagnitude = 9.81;
      double simulateDT = 1.0e-4;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      // Create an instance of the robot arm.
      RobotArmOne robotArm = new RobotArmOne();
      robotArm.setGravity(-gravityMagnitude);
      // Create an instance of the controller.
      RobotArmTwoController robotArmController = new RobotArmTwoController(robotArm, simulateDT, gravityMagnitude, yoGraphicsListRegistry);
      // Make sure to initialize the controller.
      robotArmController.initialize();
      // Attach the controller to the robot.
      robotArm.setController(robotArmController);

      // Creating the simulation.
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArm);

      scs.setSimulateNoFasterThanRealTime(true);
      // 
      scs.setDT(simulateDT, 10);
      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.setMaxBufferSize(65536);
      // Launch the simulator.
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new RobotArmTwoSimulation();
   }
}
