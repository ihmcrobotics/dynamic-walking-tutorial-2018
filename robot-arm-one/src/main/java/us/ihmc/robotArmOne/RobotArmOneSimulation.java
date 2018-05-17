package us.ihmc.robotArmOne;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class RobotArmOneSimulation
{
   public RobotArmOneSimulation()
   {
      double controlDT = 5.0e-5;

      RobotArmOne robotArm = new RobotArmOne();
      RobotArmOneController robotArmController = new RobotArmOneController(robotArm);
      robotArmController.initialize();
      robotArm.setController(robotArmController);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize((int) Math.pow(2, 16)); // => 65536
      SimulationConstructionSet scs = new SimulationConstructionSet(robotArm, parameters);
      scs.setFastSimulate(true, 15);
      scs.setDT(controlDT, 10);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new RobotArmOneSimulation();
   }
}
