package us.ihmc.robotArm.robots;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;

public interface ControllerCoreModeChangedListener
{
   void controllerCoreModeHasChanged(WholeBodyControllerCoreMode newMode);
}
