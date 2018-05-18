package us.ihmc.robotArm.robots;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.StraightLinePoseTrajectoryGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.sensorProcessors.RobotJointLimitWatcher;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameYawPitchRoll;

public class SimpleRobotArmController implements RobotController
{
   private static final boolean USE_PRIVILEGED_CONFIGURATION = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final SimpleRobotArm robotArm;
   private final YoDouble yoTime;
   private final CenterOfMassReferenceFrame centerOfMassFrame;

   public enum FeedbackControlType
   {
      SPATIAL, LINEAR_ANGULAR_SEPARATE
   };

   private final YoEnum<WholeBodyControllerCoreMode> controllerCoreMode = new YoEnum<>("controllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final AtomicBoolean controllerCoreModeHasChanged = new AtomicBoolean(false);
   private final List<ControllerCoreModeChangedListener> controllerModeListeners = new ArrayList<>();

   private final SpatialFeedbackControlCommand handSpatialCommand = new SpatialFeedbackControlCommand();
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   private final WholeBodyControllerCore controllerCore;

   private final YoDouble handWeight = new YoDouble("handWeight", registry);
   private final SymmetricYoPIDSE3Gains handPositionGains = new SymmetricYoPIDSE3Gains("handPosition", registry);
   private final SymmetricYoPIDSE3Gains handOrientationGains = new SymmetricYoPIDSE3Gains("handOrientation", registry);
   private final YoFramePoint3D handTargetPosition = new YoFramePoint3D("handTarget", worldFrame, registry);

   private final YoFrameYawPitchRoll handTargetOrientation = new YoFrameYawPitchRoll("handTarget", worldFrame, registry);
   private final YoBoolean goToTarget = new YoBoolean("goToTarget", registry);
   private final YoDouble trajectoryDuration = new YoDouble("handTrajectoryDuration", registry);
   private final YoDouble trajectoryStartTime = new YoDouble("handTrajectoryStartTime", registry);

   private final StraightLinePoseTrajectoryGenerator trajectory;

   private final YoBoolean controlLinearX = new YoBoolean("controlLinearX", registry);
   private final YoBoolean controlLinearY = new YoBoolean("controlLinearY", registry);
   private final YoBoolean controlLinearZ = new YoBoolean("controlLinearZ", registry);
   private final YoBoolean controlAngularX = new YoBoolean("controlAngularX", registry);
   private final YoBoolean controlAngularY = new YoBoolean("controlAngularY", registry);
   private final YoBoolean controlAngularZ = new YoBoolean("controlAngularZ", registry);

   private final PrivilegedConfigurationCommand privilegedConfigurationCommand = new PrivilegedConfigurationCommand();
   private final RobotJointLimitWatcher robotJointLimitWatcher;

   private final YoBoolean setRandomConfiguration = new YoBoolean("setRandomConfiguration", registry);

   public SimpleRobotArmController(SimpleRobotArm robotArm, double controlDT, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;

      controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      controllerCoreMode.addVariableChangedListener(v -> controllerCoreModeHasChanged.set(true));

      yoTime = robotArm.getYoTime();
      double gravityZ = robotArm.getGravity();
      RigidBody hand = robotArm.getHand();
      RigidBody elevator = robotArm.getElevator();
      InverseDynamicsJoint[] controlledJoints = ScrewTools.computeSupportAndSubtreeJoints(elevator);
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", worldFrame, elevator);

      ControllerCoreOptimizationSettings optimizationSettings = new RobotArmControllerCoreOptimizationSettings();

      WholeBodyControlCoreToolbox controlCoreToolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, null, controlledJoints, centerOfMassFrame,
                                                                                       optimizationSettings, yoGraphicsListRegistry, registry);

      if (USE_PRIVILEGED_CONFIGURATION)
         controlCoreToolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      controlCoreToolbox.setupForInverseDynamicsSolver(new ArrayList<>());
      controlCoreToolbox.setupForInverseKinematicsSolver();

      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();

      handSpatialCommand.set(elevator, hand);
      allPossibleCommands.addCommand(handSpatialCommand);

      JointDesiredOutputList lowLevelControllerCoreOutput = new JointDesiredOutputList(ScrewTools.filterJoints(controlledJoints, OneDoFJoint.class));

      controllerCore = new WholeBodyControllerCore(controlCoreToolbox, allPossibleCommands, lowLevelControllerCoreOutput, registry);

      yoGraphicsListRegistry.registerYoGraphic("desireds", new YoGraphicCoordinateSystem("targetFrame", handTargetPosition, handTargetOrientation, 0.15,
                                                                                         YoAppearance.Red()));

      privilegedConfigurationCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_ZERO);
      privilegedConfigurationCommand.addJoint(robotArm.getElbowPitch(), Math.PI / 3.0);

      trajectory = new StraightLinePoseTrajectoryGenerator("handTrajectory", false, worldFrame, registry, true, yoGraphicsListRegistry);

      robotJointLimitWatcher = new RobotJointLimitWatcher(ScrewTools.filterJoints(controlledJoints, OneDoFJoint.class));
      registry.addChild(robotJointLimitWatcher.getYoVariableRegistry());

      initialize();
   }

   public void registerControllerCoreModeChangedListener(ControllerCoreModeChangedListener listener)
   {
      controllerModeListeners.add(listener);
   }

   @Override
   public void initialize()
   {
      robotArm.updateIDRobot();

      handWeight.set(1.0);

      handPositionGains.setProportionalGains(100.0);
      handPositionGains.setDampingRatios(1.0);

      handOrientationGains.setProportionalGains(100.0);
      handOrientationGains.setDampingRatios(1.0);

      FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
      initialPosition.changeFrame(worldFrame);
      FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
      initialOrientation.changeFrame(worldFrame);

      handTargetPosition.setMatchingFrame(initialPosition);
      handTargetOrientation.setMatchingFrame(initialOrientation);

      trajectoryDuration.set(0.5);
      trajectory.setInitialPose(initialPosition, initialOrientation);
      trajectory.setFinalPose(initialPosition, initialOrientation);
      trajectory.setTrajectoryTime(trajectoryDuration.getValue());

      controlLinearX.set(true);
      controlLinearY.set(true);
      controlLinearZ.set(true);
      controlAngularX.set(true);
      controlAngularY.set(true);
      controlAngularZ.set(true);
      trajectory.showVisualization();
   }

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();
   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();

   @Override
   public void doControl()
   {
      robotArm.updateControlFrameAcceleration();
      robotArm.updateIDRobot();
      centerOfMassFrame.update();

      updateTrajectory();
      updateFeedbackCommands();

      controllerCoreCommand.clear();
      controllerCoreCommand.addFeedbackControlCommand(handSpatialCommand);

      if (USE_PRIVILEGED_CONFIGURATION)
         controllerCoreCommand.addInverseDynamicsCommand(privilegedConfigurationCommand);
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();

      ControllerCoreOutput controllerCoreOutput = controllerCore.getControllerCoreOutput();
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = controllerCoreOutput.getLowLevelOneDoFJointDesiredDataHolder();

      if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.OFF
            || controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.VIRTUAL_MODEL)
         controllerCoreMode.set(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

      if (controllerCoreModeHasChanged.getAndSet(false))
         controllerModeListeners.forEach(listener -> listener.controllerCoreModeHasChanged(controllerCoreMode.getEnumValue()));

      controllerCoreCommand.setControllerCoreMode(controllerCoreMode.getEnumValue());

      if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         robotArm.updateSCSRobotJointTaus(lowLevelOneDoFJointDesiredDataHolder);
      else
         robotArm.updateSCSRobotJointConfiguration(lowLevelOneDoFJointDesiredDataHolder);

      if (setRandomConfiguration.getValue())
      {
         robotArm.setRandomConfiguration();
         setRandomConfiguration.set(false);
      }

      robotJointLimitWatcher.doControl();
   }

   public void updateFeedbackCommands()
   {
      FramePose3D controlFramePose = new FramePose3D(robotArm.getHandControlFrame());
      controlFramePose.changeFrame(robotArm.getHand().getBodyFixedFrame());

      trajectory.getAngularData(orientation, angularVelocity, angularAcceleration);
      trajectory.getLinearData(position, linearVelocity, linearAcceleration);

      handSpatialCommand.setControlFrameFixedInEndEffector(controlFramePose);
      handSpatialCommand.setWeightForSolver(handWeight.getValue());
      handSpatialCommand.setPositionGains(handPositionGains);
      handSpatialCommand.setOrientationGains(handOrientationGains);
      handSpatialCommand.setSelectionMatrix(computeSpatialSelectionMatrix());
      handSpatialCommand.set(position, linearVelocity);
      handSpatialCommand.set(orientation, angularVelocity);
      if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.INVERSE_DYNAMICS)
         handSpatialCommand.changeFrameAndSetFeedForward(angularAcceleration, linearAcceleration);
      else if (controllerCoreMode.getEnumValue() == WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
         handSpatialCommand.changeFrameAndSetFeedForward(angularVelocity, linearVelocity);
   }

   public void updateTrajectory()
   {
      if (goToTarget.getValue())
      {
         FramePoint3D initialPosition = new FramePoint3D(robotArm.getHandControlFrame());
         initialPosition.changeFrame(worldFrame);
         FrameQuaternion initialOrientation = new FrameQuaternion(robotArm.getHandControlFrame());
         initialOrientation.changeFrame(worldFrame);
         trajectory.setInitialPose(initialPosition, initialOrientation);
         FramePoint3D finalPosition = new FramePoint3D(handTargetPosition);
         FrameQuaternion finalOrientation = new FrameQuaternion();
         handTargetOrientation.getFrameOrientationIncludingFrame(finalOrientation);
         trajectory.setFinalPose(finalPosition, finalOrientation);
         trajectory.setTrajectoryTime(trajectoryDuration.getValue());
         trajectory.initialize();
         trajectoryStartTime.set(yoTime.getValue());
         goToTarget.set(false);
      }

      trajectory.compute(yoTime.getValue() - trajectoryStartTime.getValue());
   }

   private SelectionMatrix6D computeSpatialSelectionMatrix()
   {
      SelectionMatrix6D selectionMatrix = new SelectionMatrix6D();

      selectionMatrix.selectAngularX(controlAngularX.getValue());
      selectionMatrix.selectAngularY(controlAngularY.getValue());
      selectionMatrix.selectAngularZ(controlAngularZ.getValue());

      selectionMatrix.selectLinearX(controlLinearX.getValue());
      selectionMatrix.selectLinearY(controlLinearY.getValue());
      selectionMatrix.selectLinearZ(controlLinearZ.getValue());

      return selectionMatrix;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return name;
   }

   public YoFramePoint3D getHandTargetPosition()
   {
      return handTargetPosition;
   }

   public YoFrameYawPitchRoll getHandTargetOrientation()
   {
      return handTargetOrientation;
   }

   public YoBoolean getGoToTarget()
   {
      return goToTarget;
   }
}
