package us.ihmc.robotArmTwo;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RobotArmTwoSimulation
{
   public RobotArmTwoSimulation()
   {
      /*
       *  Three modes are available for the controller core:
       *  - Inverse Dynamics: Given desired accelerations and contact states, the controller core computes desired joint torques.
       *  - Virtual Model Control: It is a generalization of the "Jacobian transpose" method to a whole-body framework, the output is desired joint torques.
       *  - Inverse Kinematics: Given desired velocities, the controller core can integrate the these velocities to output both desired joint velocities and positions.
       */
      WholeBodyControllerCoreMode controlMode = WholeBodyControllerCoreMode.INVERSE_DYNAMICS;
      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;
      // The control frequency, which is equal to simulation frequency in this example, has to be provided to the controller core.
      double simulateDT = 1.0e-4;
      // This is an additional registry that allows to display 3D graphics in the simulation. This feature is not demonstrated in this example.
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      // Create an instance of the robot arm.
      RobotArmTwo robotArm = new RobotArmTwo();
      // Use the simulated definition of the robot to define the simulation environment.
      Robot simulatedRobot = robotArm.getSimulatedRobot();
      // Make sure the simulation and the controller are using the same value for the gravity.
      simulatedRobot.setGravity(-gravityMagnitude);
      // Create an instance of the controller.
      RobotArmTwoController robotArmController = new RobotArmTwoController(robotArm, simulateDT, gravityMagnitude, controlMode, yoGraphicsListRegistry);
      // When using the inverse kinematics mode, the simulation dynamics is disabled to simply provide a direct visualization of the controller core ouptut.
      if (controlMode == WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
         simulatedRobot.setDynamic(false);
      // Make sure to initialize the controller.
      robotArmController.initialize();
      // Attach the controller to the robot.
      simulatedRobot.setController(robotArmController);

      // Creating the simulation.
      SimulationConstructionSet scs = new SimulationConstructionSet(simulatedRobot);
      // As this example simulation is rather simple, let's prevent SCS from simulating faster than real-time.
      scs.setSimulateNoFasterThanRealTime(true);
      // Defining the simulation DT and the frequency at which data is logged.
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
