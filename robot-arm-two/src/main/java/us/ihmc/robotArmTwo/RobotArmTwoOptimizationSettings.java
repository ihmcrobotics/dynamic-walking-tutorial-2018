package us.ihmc.robotArmTwo;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

public class RobotArmTwoOptimizationSettings implements ControllerCoreOptimizationSettings
{
   @Override
   public double getJointVelocityWeight()
   {
      return ControllerCoreOptimizationSettings.super.getJointVelocityWeight();
   }

   @Override
   public double getJointAccelerationWeight()
   {
      return 0.005;
   }

   @Override
   public double getJointJerkWeight()
   {
      return 0.1;
   }

   @Override
   public double getRhoWeight()
   {
      return 0;
   }

   @Override
   public double getRhoMin()
   {
      return 0;
   }

   @Override
   public double getRhoRateDefaultWeight()
   {
      return 0;
   }

   @Override
   public Vector2D getCoPWeight()
   {
      return new Vector2D();
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   {
      return new Vector2D();
   }

   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   {
      return 0;
   }

   @Override
   public int getNumberOfContactPointsPerContactableBody()
   {
      return 0;
   }

   @Override
   public int getNumberOfContactableBodies()
   {
      return 0;
   }
}
