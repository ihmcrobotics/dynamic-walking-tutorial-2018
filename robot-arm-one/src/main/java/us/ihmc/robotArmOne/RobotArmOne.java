package us.ihmc.robotArmOne;

import java.util.EnumMap;

import us.ihmc.dwc.utilities.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotArmOne extends Robot
{
   private final PinJoint shoulderYawJoint;
   private final PinJoint shoulderRollJoint;
   private final PinJoint shoulderPitchJoint;
   private final PinJoint elbowPitchJoint;
   private final PinJoint wristPitchJoint;
   private final PinJoint wristRollJoint;
   private final PinJoint wristYawJoint;

   private final EnumMap<SevenDoFArmJointEnum, PinJoint> jointMap = new EnumMap<>(SevenDoFArmJointEnum.class);

   public RobotArmOne()
   {
      super("RobotArm1");

      shoulderYawJoint = createArmJoint(SevenDoFArmJointEnum.shoulderYaw);
      shoulderRollJoint = createArmJoint(SevenDoFArmJointEnum.shoulderRoll);
      shoulderPitchJoint = createArmJoint(SevenDoFArmJointEnum.shoulderPitch);
      elbowPitchJoint = createArmJoint(SevenDoFArmJointEnum.elbowPitch);
      wristPitchJoint = createArmJoint(SevenDoFArmJointEnum.wristPitch);
      wristRollJoint = createArmJoint(SevenDoFArmJointEnum.wristRoll);
      wristYawJoint = createArmJoint(SevenDoFArmJointEnum.wristYaw);

      setupLinks();

      addRootJoint(shoulderYawJoint);
      shoulderYawJoint.addJoint(shoulderRollJoint);
      shoulderRollJoint.addJoint(shoulderPitchJoint);
      shoulderPitchJoint.addJoint(elbowPitchJoint);
      elbowPitchJoint.addJoint(wristPitchJoint);
      wristPitchJoint.addJoint(wristRollJoint);
      wristRollJoint.addJoint(wristYawJoint);

      jointMap.put(SevenDoFArmJointEnum.shoulderYaw, shoulderYawJoint);
      jointMap.put(SevenDoFArmJointEnum.shoulderRoll, shoulderRollJoint);
      jointMap.put(SevenDoFArmJointEnum.shoulderPitch, shoulderPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.elbowPitch, elbowPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.wristPitch, wristPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.wristRoll, wristRollJoint);
      jointMap.put(SevenDoFArmJointEnum.wristYaw, wristYawJoint);
   }

   private void setupLinks()
   {
      Link shoulderYawChildLink = createChildLink(SevenDoFArmJointEnum.shoulderYaw);
      Link shoulderRollChildLink = createChildLink(SevenDoFArmJointEnum.shoulderRoll);
      Link shoulderPitchChildLink = createChildLink(SevenDoFArmJointEnum.shoulderPitch);
      Link elbowPitchChildLink = createChildLink(SevenDoFArmJointEnum.elbowPitch);
      Link wristPitchChildLink = createChildLink(SevenDoFArmJointEnum.wristPitch);
      Link wristRollChildLink = createChildLink(SevenDoFArmJointEnum.wristRoll);
      Link wristYawChildLink = createChildLink(SevenDoFArmJointEnum.wristYaw);

      shoulderYawJoint.setLink(shoulderYawChildLink);
      shoulderRollJoint.setLink(shoulderRollChildLink);
      shoulderPitchJoint.setLink(shoulderPitchChildLink);
      elbowPitchJoint.setLink(elbowPitchChildLink);
      wristPitchJoint.setLink(wristPitchChildLink);
      wristRollJoint.setLink(wristRollChildLink);
      wristYawJoint.setLink(wristYawChildLink);
   }

   private PinJoint createArmJoint(SevenDoFArmJointEnum jointEnum)
   {
      PinJoint pinJoint = new PinJoint(jointEnum.getJointName(), jointEnum.getJointOffset(), this, jointEnum.getJointAxis());
      return pinJoint;
   }

   private Link createChildLink(SevenDoFArmJointEnum jointEnum)
   {
      Link childLink = new Link(jointEnum.getChildLinkName());
      childLink.setMass(jointEnum.getChildLinkMass());
      childLink.setComOffset(jointEnum.getChildLinkCoM());
      childLink.setMomentOfInertia(jointEnum.getChildLinkInertia());
      childLink.setLinkGraphics(jointEnum.getChildLinkGraphics());
      return childLink;
   }

   public double getCurrentJointPosition(SevenDoFArmJointEnum jointEnum)
   {
      return jointMap.get(jointEnum).getQ();
   }

   public double getCurrentJointVelocity(SevenDoFArmJointEnum jointEnum)
   {
      return jointMap.get(jointEnum).getQD();
   }

   public void setDesiredJointEffort(SevenDoFArmJointEnum jointEnum, double desiredEffort)
   {
      jointMap.get(jointEnum).setTau(desiredEffort);
   }
}
