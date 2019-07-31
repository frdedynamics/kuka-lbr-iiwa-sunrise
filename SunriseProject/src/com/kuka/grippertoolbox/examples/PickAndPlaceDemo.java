package com.kuka.grippertoolbox.examples;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.lin;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.linRel;
import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import java.util.concurrent.TimeUnit;

import javax.inject.Inject;

import com.kuka.grippertoolbox.api.gripper.AbstractGripper;
import com.kuka.grippertoolbox.api.state.GripperState;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.applicationModel.tasks.UseRoboticsAPIContext;
import com.kuka.roboticsAPI.conditionModel.ForceCondition;
import com.kuka.roboticsAPI.conditionModel.ICondition;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.Robot;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.PTP;
import com.kuka.roboticsAPI.persistenceModel.PersistenceException;
import com.kuka.roboticsAPI.uiModel.ApplicationDialogType;
import com.kuka.sunrise.common.task.categories.ApplicationCategory;

/**
 * This application demonstrates advanced usage of the GripperToolbox API.
 * <p>
 * At the beginning it creates a gripper tool and attaches it to the robot, which is to a basic position. It checks the
 * initial state of the gripper and opens it if it's closed. If the gripper does already carry an item, it places it at
 * a target position and terminates.
 * <p>
 * The application tries to pick an item from frame '/pick' and places it at frame '/place', using the tools default
 * motion frame.
 * <p>
 * DO THIS BEFORE STARTING THE APPLICATION: <br>
 * ======================================== <br>
 * 
 * 1. create the frames "/pick" and "/place" in the
 * application data <br>
 * 2. synchronize the project <br>
 * 3. teach these frames with tcp of the installed gripper<br>
 * 4. then start the application
 * <p>
 * corresponding to the common security nodes, test the application
 * first in Operational Mode T1
 */
@UseRoboticsAPIContext
@ApplicationCategory
public class PickAndPlaceDemo extends RoboticsAPIApplication
{
    private static final JointPosition BASIC_POSITION = new JointPosition(0,
            Math.PI / 4, 0, -Math.PI / 2, 0, Math.PI / 4, 0);

    private static final double JOINT_VELOCITY_REL = 0.2;

    private static final String INFORMATION_TEXT = "This application starts moving the robot to "
            + BASIC_POSITION.toString() + ".\nContinue?";

    private static final String MISSING_FRAME_INFO = "The application needs teached Frames named '/pick' and '/place' but couldn't find them.\n"
            + "Please create the frames in the application data and teach them with the gripper's TCP. See the comments in the source code for more information.\n"
            + "The application will be disposed now.";

    private static final String PICK_FRAME_PATH = "/pick";
    private static final String PLACE_FRAME_PATH = "/place";

    @Inject
    private Robot robot;

    @Inject
    private AbstractGripper gripper;

    @Override
    public void run() throws InterruptedException
    {
        if (!checkPickAndPlaceFrames())
        {
            getApplicationUI().displayModalDialog(ApplicationDialogType.ERROR,
                    MISSING_FRAME_INFO, "OK");
            return;
        }

        // show dialog and wait for user to confirm.
        final int isCancel = getApplicationUI()
                .displayModalDialog(ApplicationDialogType.QUESTION,
                        INFORMATION_TEXT, "OK", "Cancel");
        if (isCancel == 1)
        {
            return;
        }

        final PTP basicPositionMotion = ptp(BASIC_POSITION)
                .setJointVelocityRel(JOINT_VELOCITY_REL);

        // Attach the gripper to the flange of the robot.
        gripper.attachTo(robot.getFlange());

        // Move the robot to a feasible position.
        robot.move(basicPositionMotion);

        // First of all, check the current position of the gripper.
        switch (gripper.getGripperState())
        {
        /* The gripper already carries an item, so we need to put it away. Since
         * we can't place another object there, terminate the application. */
        case GRIPPED_ITEM:
            placeAt(getFrame(PLACE_FRAME_PATH));
            robot.move(basicPositionMotion);
            return;

        // $FALL-THROUGH$
        case GRIPPED:
        case UNDEFINED:
            /* Notice: Be sure no item is gripped! */
            gripper.releaseAsync();
            break;

        // The gripper is already in released state, so do nothing
        default:
            break;
        }

        // If we fail to get an item from this frame, we ask the user if he
        // wants to continue without.
        if (!pickFrom(getFrame(PICK_FRAME_PATH)))
        {
            final int userSelection = getApplicationUI().displayModalDialog(
                    ApplicationDialogType.QUESTION,
                    "Failed to pick an item, continue demo without item anyways?",
                    "Yes", "No");
            if (userSelection == 1)
            {
                gripper.releaseAsync();
                robot.move(basicPositionMotion);
                return;
            }
        }

        // Put the item to this target frame.
        if (!placeAt(getFrame(PLACE_FRAME_PATH)))
        {
            getLogger().warn("Couldn't find a safe spot to place the item.");
        }

        // Finally move the robot back to it's basic position.
        robot.move(basicPositionMotion);
    }

    /**
     * Tries to fetch an item from this frame. This method fails, if no item was
     * found.
     *
     * @param itemFrame
     *            The frame where the item roughly is placed.
     * @return <code>true</code>, if the picking was successful. Otherwise <code>false</code>.
     * @throws InterruptedException
     */
    private boolean pickFrom(final ObjectFrame itemFrame)
            throws InterruptedException
    {
        boolean success = false;

        // First compute a frame that is 50 mm away from the object and move there.
        final Frame preFrame = itemFrame.copyWithRedundancy();
        preFrame.setZ(itemFrame.getZ() + 50);

        gripper.move(lin(preFrame).setJointVelocityRel(JOINT_VELOCITY_REL));

        /* Now move 100 mm along the z-axis of the default motion frame and stop
         * the motion if the torque sensors detect a small resisting force. if
         * nothing was found, return immediately false. */
        if (contactMotion(gripper.getDefaultMotionFrame(), 100))
        {
            /* Now try to grab the object and wait until the grippers
             * sensors indicate a gripped item. Notice: The hardware of some
             * grippers doesn't support this feature. In that case, the
             * success of the grip-action must be verified in a different
             * manner. */
            gripper.gripAsync();
            final boolean itemGripped = gripper.waitForGripperState(1, TimeUnit.SECONDS, GripperState.GRIPPED_ITEM);

            if (itemGripped)
            {
                // Lift the item a little bit before moving on.
                gripper.move(lin(preFrame).setJointVelocityRel(JOINT_VELOCITY_REL));

                // Everything went fine.
                success = true;
            }
            else
            {
                /* Happens when the time waiting for a successful grip exceeded
                 * the timeout of 1 second. */
                getLogger().warn("Failed to pick an item.");
            }

        }

        return success;
    }

    /**
     * Tries to put an item at a target destination. The method fails, if no
     * solid ground was detected.
     *
     * @param targetFrame
     *            The Frame where the object shall be placed.
     * @return True, if the placing was successful, false otherwise.
     */
    private boolean placeAt(final ObjectFrame targetFrame)
    {
        boolean success = false;

        // First, compute a frame that is 50 mm away from the target position and move there.
        final Frame preFrame = targetFrame.copyWithRedundancy();
        preFrame.setZ(targetFrame.getZ() + 50);

        gripper.move(lin(preFrame).setJointVelocityRel(JOINT_VELOCITY_REL));

        /* Now move 100 mm along the z-axis of the default motion frame and stop
         * the motion if the torque sensors detect a small resisting force. If
         * no solid ground could be detected, it's not safe to release the item. */
        if (contactMotion(gripper.getDefaultMotionFrame(), 100))
        {
            // open the gripper
            gripper.releaseAsync();

            // move back to the preFrame
            gripper.move(lin(preFrame).setJointVelocityRel(JOINT_VELOCITY_REL));
            success = true;
        }

        return success;
    }

    /**
     * Simple contact motion using Cartesian forces.
     *
     * @param tcp
     *            The frame to move with, usually the tcp.
     * @param zOffset
     *            The offset of the origin in z-direction [mm].
     * @return True for a successful contact, false otherwise.
     */
    private boolean contactMotion(final ObjectFrame tcp, final double zOffset)
    {
        // Create the force condition with 10 Newton.
        final ICondition forceCondition = ForceCondition.createSpatialForceCondition(tcp, 10);

        // Slowly move the tcp in z-direction until either the offset is reached
        // or the ForceCondition stops the motion.
        final IMotionContainer motion = tcp.move(linRel(0, 0, zOffset).setCartVelocity(20).breakWhen(forceCondition));
        return motion.hasFired(forceCondition);
    }

    /**
     * Check if the given frame and its teach information exists and if the frames
     * has been teached with the "/TCP" of the gripper.
     *
     * @param frame
     *            frame that should be checked
     *
     * @return True if the frame and its TeachInforamtion exists, false
     *         otherwise
     */
    private boolean checkPickAndPlaceFrames()
    {
        try
        {
            final Object teachInfoPick = getFrame(PICK_FRAME_PATH).getAdditionalFrameData().get(AbstractFrame.TEACHINFO_PARAMETER);
            final Object teachInfoPlace = getFrame(PLACE_FRAME_PATH).getAdditionalFrameData().get(AbstractFrame.TEACHINFO_PARAMETER);

            if ((teachInfoPick == null) || (teachInfoPlace == null))
            {
                getLogger().error("Teach Information of frame '" + PICK_FRAME_PATH + "' or '"
                        + PLACE_FRAME_PATH + "' does not exist.");
                return false;
            }

            if (!teachInfoPlace.toString().contains("/TCP") || !teachInfoPick.toString().contains("/TCP"))
            {
                getLogger().error("The frames '" + PICK_FRAME_PATH + "' and '"
                        + PLACE_FRAME_PATH + "' have not been teached with the correct tcp");
                return false;
            }
        }
        catch (final PersistenceException pe)
        {
            getLogger().error("The frames '" + PICK_FRAME_PATH + "' and/or '"
                    + PLACE_FRAME_PATH + "' do not exist.", pe);
            return false;
        }

        return true;
    }
}
