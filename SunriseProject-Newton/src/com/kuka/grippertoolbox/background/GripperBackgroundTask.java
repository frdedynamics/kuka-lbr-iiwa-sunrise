package com.kuka.grippertoolbox.background;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.IGripper;
import com.kuka.grippertoolbox.api.userkey.IGripperKeyManager;
import com.kuka.roboticsAPI.applicationModel.tasks.CycleBehavior;
import com.kuka.roboticsAPI.applicationModel.tasks.RoboticsAPICyclicBackgroundTask;
import com.kuka.sunrise.common.task.categories.BackgroundTaskCategory;

/**
 * This BackgroundTask creates the gripper user keys for manual usage.
 * <p>
 * This {@link RoboticsAPICyclicBackgroundTask} will automatically create the user key bar for manual usage of the
 * gripper functions {@link IGripper#gripAsync()} and {@link IGripper#releaseAsync()}. The userkeys are disabled when
 * no enabling switch is pressed, the operation mode is not one of T1, T2 or AUT or when the bus communication fails.
 * </p>
 * <p>
 * DO NOT MODIFY!
 * </p>
 */
@BackgroundTaskCategory(autoStart = true)
public class GripperBackgroundTask extends RoboticsAPICyclicBackgroundTask
{
    @Inject
    private IGripperKeyManager _gripperUserKeyManager;

    @Inject
    private IGripper _gripper;

    /**
     * Creates the gripper user key bar.
     * <p>
     * DO NOT MODIFY!
     * </p>
     */
    @Override
    public void initialize()
    {
        initializeCyclic(0, 10, TimeUnit.MINUTES, CycleBehavior.BestEffort);
        getLogger().info("initialize GripperBackgroundTask");

        _gripper.initialize();
        _gripperUserKeyManager.install(getApplicationUI(), _gripper);
    }

    @Override
    protected void runCyclic()
    {
        // nothing to do here
    }

    @Override
    public void dispose()
    {
        try
        {
            _gripperUserKeyManager.dispose();
        }
        catch (Exception e)
        {
            getLogger().error(e.getMessage());
        }
        
        super.dispose();
    }
}
