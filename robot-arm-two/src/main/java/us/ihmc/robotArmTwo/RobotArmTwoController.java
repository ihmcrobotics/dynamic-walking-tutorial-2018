package us.ihmc.robotArmTwo;

import java.util.Collections;
import java.util.EnumMap;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.dwc.utilities.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotArmOne.RobotArmOne;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotArmTwoController implements RobotController
{
   private static final double TWO_PI = 2.0 * Math.PI;
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */
   private final YoVariableRegistry registry = new YoVariableRegistry("Controller");
   /**
    * Desired position for each joint. {@code YoDouble}s are used instead of simple {@code double}
    * so they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> desiredPositions = new EnumMap<>(SevenDoFArmJointEnum.class);
   /**
    * Desired velocity for each joint. {@code YoDouble}s are used instead of simple {@code double}
    * so they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> desiredVelocities = new EnumMap<>(SevenDoFArmJointEnum.class);
   /**
    * Desired acceleration for each joint. {@code YoDouble}s are used instead of simple
    * {@code double} so they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> desiredAccelerations = new EnumMap<>(SevenDoFArmJointEnum.class);

   private final YoPDGains gains = new YoPDGains("jointsGains", registry);

   private final YoDouble time;

   private final RobotArmOne simulatedRobotArm;
   private final InverseDynamicsJointsFromSCSRobotGenerator inverseDynamicsRobot;
   private final SCSToInverseDynamicsJointMap jointMap;

   private final WholeBodyControllerCore wholeBodyControllerCore;
   private final ReferenceFrame centerOfMassFrame;

   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   public RobotArmTwoController(RobotArmOne robotArm, double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.simulatedRobotArm = robotArm;
      time = robotArm.getYoTime();
      inverseDynamicsRobot = new InverseDynamicsJointsFromSCSRobotGenerator(robotArm);
      jointMap = inverseDynamicsRobot.getSCSToInverseDynamicsJointMap();

      FloatingInverseDynamicsJoint rootJoint = null;
      RigidBody elevator = inverseDynamicsRobot.getElevator();
      InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.computeSubtreeJoints(elevator);
      OneDoFJoint[] controlledJoints = ScrewTools.filterJoints(inverseDynamicsJoints, OneDoFJoint.class);

      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", WORLD_FRAME, elevator);
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RobotArmTwoOptimizationSettings();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, inverseDynamicsJoints, centerOfMassFrame,
                                                                            controllerCoreOptimizationSettings, yoGraphicsListRegistry, registry);
      toolbox.setupForInverseKinematicsSolver();
      toolbox.setupForInverseDynamicsSolver(Collections.emptyList());
      toolbox.setupForVirtualModelControlSolver(elevator, Collections.emptyList());

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      JointspaceFeedbackControlCommand command = new JointspaceFeedbackControlCommand();
      for (OneDoFJoint joint : controlledJoints)
      {
         command.addJoint(joint, 0.0, 0.0, 0.0);
      }
      allPossibleCommands.addCommand(command);

      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controlledJoints);
      wholeBodyControllerCore = new WholeBodyControllerCore(toolbox, allPossibleCommands, lowLevelControllerOutput, registry);

      /*
       * YoDoubles need to be created first with a given name that is to represent the variable in
       * the Simulation Construction Set, and the registry so the simulation can find them.
       */
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         String jointName = StringUtils.capitalize(jointEnum.getJointName());
         desiredPositions.put(jointEnum, new YoDouble("desiredPosition" + jointName, registry));
         desiredVelocities.put(jointEnum, new YoDouble("desiredVelocity" + jointName, registry));
         desiredAccelerations.put(jointEnum, new YoDouble("desiredAcceleration" + jointName, registry));
      }
   }

   @Override
   public void initialize()
   {
      gains.setPDGains(100.0, 1.0);
      gains.createDerivativeGainUpdater(true);
      wholeBodyControllerCore.initialize();
   }

   @Override
   public void doControl()
   {
      inverseDynamicsRobot.updateInverseDynamicsRobotModelFromRobot(true);
      centerOfMassFrame.update();

      { // Making the shoulder yaw joint follow a sine wave trajectory:
         double frequency = 0.2;
         double phase = Math.PI;
         double amplitude = 0.5;
         double omega = TWO_PI * frequency;
         double q = amplitude * Math.sin(omega * time.getValue() + phase);
         double qDot = omega * amplitude * Math.cos(omega * time.getValue() + phase);
         double qDDot = -omega * omega * amplitude * Math.sin(omega * time.getValue() + phase);

         desiredPositions.get(SevenDoFArmJointEnum.shoulderYaw).set(q);
         desiredVelocities.get(SevenDoFArmJointEnum.shoulderYaw).set(qDot);
         desiredAccelerations.get(SevenDoFArmJointEnum.shoulderYaw).set(qDDot);
      }

      { // Making the elbow pitch joint follow a sine wave trajectory:
         double offset = 0.5 * Math.PI;
         double frequency = 0.2;
         double phase = -0.5 * Math.PI;
         double amplitude = 0.5;
         double omega = TWO_PI * frequency;
         double q = offset + amplitude * Math.sin(omega * time.getValue() + phase);
         double qDot = omega * amplitude * Math.cos(omega * time.getValue() + phase);
         double qDDot = -omega * omega * amplitude * Math.cos(omega * time.getValue() + phase);

         desiredPositions.get(SevenDoFArmJointEnum.elbowPitch).set(q);
         desiredVelocities.get(SevenDoFArmJointEnum.elbowPitch).set(qDot);
         desiredAccelerations.get(SevenDoFArmJointEnum.elbowPitch).set(qDDot);
      }

      JointspaceFeedbackControlCommand jointCommand = new JointspaceFeedbackControlCommand();

      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         PinJoint simulatedJoint = simulatedRobotArm.getJoint(jointEnum);
         OneDoFJoint inverseDynamicsJoint = jointMap.getInverseDynamicsOneDoFJoint(simulatedJoint);
         double desiredPosition = desiredPositions.get(jointEnum).getValue();
         double desiredVelocity = desiredVelocities.get(jointEnum).getValue();
         jointCommand.addJoint(inverseDynamicsJoint, desiredPosition, desiredVelocity, 0.0);
      }

      jointCommand.setGains(gains);
      jointCommand.setWeightForSolver(1.0);

      controllerCoreCommand.addFeedbackControlCommand(jointCommand);

      wholeBodyControllerCore.submitControllerCoreCommand(controllerCoreCommand);
      wholeBodyControllerCore.compute();
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         PinJoint simulatedJoint = simulatedRobotArm.getJoint(jointEnum);
         OneDoFJoint inverseDynamicsJoint = jointMap.getInverseDynamicsOneDoFJoint(simulatedJoint);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(inverseDynamicsJoint);

         simulatedJoint.setTau(jointDesiredOutput.getDesiredTorque());
      }
   }

   @Override
   public String getName()
   {
      return "RobotArmTwoController";
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return "Controller demonstrating a jointspace control using the IHMC whole-body controller core.";
   }
}
