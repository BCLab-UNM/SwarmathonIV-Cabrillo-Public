#include "DropOffController.h"
#include "Logger.h"

DropOffController::DropOffController() {
    cameraOffsetCorrection = 0.020; //meters
    centeringTurn = 0.15; //radians
    seenEnoughCenterTagsCount = 8;
    collectionPointVisualDistance = 0.5; //in meters
    reachedCollectionPoint = false;
    spinSize = 0.20; //in meters aka 10cm 
    addSpinSizeAmmount = 0.10; //in meters

    result.cmdVel = 0;
    result.angleError = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    result.goalDriving = false; //set true for when driving is goal location oriented and false when it is target or time oriented
    result.centerGoal; //goal that is center location or based upon the center location.
    result.reset = false;
    result.timer = false;
    result.useOdom = false;

    left = false;
    right = false;

    circularCenterSearching = false;
    spinner = M_PI_4 + (M_PI_4 / 2);
    centerApproach = false;
    timeWithoutSeeingEnoughCenterTags = time(0);
    seenEnoughCenterTags = false;
    centerSeen = false;
    timeElapsedSinceTimeSinceSeeingEnoughCenterTags = time(0);
    circularCenterSearching = false;
    prevCount = 0;

    searchVelocity = 0.15;

    useOdom = false;
}



void DropOffController::calculateDecision() {

    result.goalDriving = true; //assumewe are driving to the center unless we see targets or have seen targets.
    result.timer = false;


    //if we are in the routine for exciting the circle once we have droppeda block off and reseting all our flags
    //to resart our search.
    if(reachedCollectionPoint)
    {
        result.goalDriving = false;

        //timerStartTime was reset before we entered reachedCollectionPoint so
        //we can now use it for our timeing of 2 seconds

        if (timerTimeElapsed >= 6)
        {
        	// Do this 6 seconds after detecting we're in the center (and ready to dropp off a block.).
            result.reset = true; //tell mobility to reset to search parameters
            Logger::chat("DOC: Telling mobility to reset search parameters.");
        }
        else if (timerTimeElapsed >= 1)
        {
        	// Do this 1 second after detecting we're in the center.
            //open fingers
            float angle;
            angle = M_PI_2;
            result.fingerAngle = angle;
            angle= 0;
            result.wristAngle = angle; //raise wrist

            result.cmdVel = -0.2;
            result.angleError = 0.0;
            Logger::chat("DOC: Dropping off block. %d", timerTimeElapsed);
        }else{
            Logger::chat("DOC: Reached collection point.");
        }
        return;
    }

    //check to see if we are driving to the center location or if we need to drive in a circle and look.
    if (distanceToCenter > collectionPointVisualDistance && !circularCenterSearching && count == 0) {
    	// XXX: Kiley: if we're here the rover is trying to drive back to
    	// the nest. In this case we should report back to mobility an absolute goal
    	// in result.centerGoal and tell mobility that it's absolute by setting result.useOdom = false.
    	//

    	result.useOdom = false;
        //set angle to center as goal heading
        result.centerGoal.theta = atan2(centerLocation.y - currentLocationMap.y, centerLocation.x - currentLocationMap.x);

        //set center as goal position
        result.centerGoal.x = centerLocation.x;
        result.centerGoal.y = centerLocation.y;
        //spinWasTrue = true; only turn on for random walk to center
    }
    else if (timerTimeElapsed >=5)//spin search for center
    {
    	// XXX: Kiley: If we're here the rover is trying to find the center after going to it's GPS
    	// coordinates and not finding it. That seems like it'll happen a lot! Return a relative goal
    	// in result.centerGoal. Remember, setRelativeGoal() takes r, theta. You can "abuse" the member
    	// variable in result.centerGoal to be used this way. If you look closely at the code below
    	// you can figure out what term is "r" and what term is "theta".
    	//
        //sets a goal that is 60cm from the centerLocation and spinner
        //radians counterclockwise from being purly along the x-axis.
    	result.useOdom = true;
    	result.centerGoal.x = spinSize + addSpinSize;
    	result.centerGoal.theta = spinner;
        //result.centerGoal.x = centerLocation.x + (spinSize + addSpinSize) * cos(spinner);
        //result.centerGoal.y = centerLocation.y + (spinSize + addSpinSize) * sin(spinner);
        //result.centerGoal.theta = atan2(result.centerGoal.y - currentLocation.y, result.centerGoal.x - currentLocation.x);

//        spinner += 45*(M_PI/180); //add 45 degrees in radians to spinner.
//        if (spinner > 2*M_PI)
//        {
//            spinner -= 2*M_PI;
	    addSpinSize += addSpinSizeAmmount;
//        }
        circularCenterSearching = true;
        //safety flag to prevent us trying to drive back to the
        //center since we have a block with us and the above point is
        //greater than collectionPointVisualDistance from the center.
    }


    //reset timeWithoutSeeingEnoughCenterTags timout timer to current time
    if ((!centerApproach && !seenEnoughCenterTags) || (count > 0 && !seenEnoughCenterTags)) {
    	timeWithoutSeeingEnoughCenterTags = time(0);
    }

    //Logger::chat("count: %d countLeft: %d countRight: %d timerTimeElapsed: %f", count, countLeft, countRight, timerTimeElapsed);
    if (count > 0 || seenEnoughCenterTags || prevCount > 0) //if we have a target and the center is located drive towards it.
    {
        centerSeen = true;
        result.goalDriving = false;

        if (seenEnoughCenterTags) //if we have seen enough tags
        {
        	// XXX: Tuneable
            if ((countLeft/2) > countRight ) //and there are too many on the left
            {
                right = false; //then we say non on the right to cause us to turn right
            }
        	// XXX: Tuneable
            else if ((countRight/2) > countLeft)
            {
                left = false; //or left in this case
            }
        }

//        if (seenEnoughCenterTags) {
//        	Logger::chat("in");
//            result.angleError = -sumCog * 20;
//            result.cmdVel = searchVelocity * .75;
//        } else {
//        if (!seenEnoughCenterTags){
//        	Logger::chat("!seenEnough");
//        	result.angleError = sumCog * 20;
//        	result.cmdVel = searchVelocity * .75;
//        }

        float turnMax = .2;
        float turnMult = 20;
        float turnDirection = 1;
        //reverse tag rejection when we have seen enough tags that we are on a
        //trajectory in to the square we dont want to follow an edge.
        if (seenEnoughCenterTags) {
//        	Logger::chat("Seen Enough Center Tags, turn=-1");
        	turnDirection = -1;
        	turnMax = .1;
        	turnMult = 5;
        }
//        centeringTurn = -sumCog * 10;

        //otherwise turn till tags on both sides of image then drive straight
        if (left && right) {
        	Logger::chat("Straight on");
            result.cmdVel = searchVelocity;
            result.angleError = 0.0;
        }
        else if (sumCog > 0) {
        	result.angleError = -sumCog * turnMult * turnDirection;
        	result.cmdVel = searchVelocity * .5;
        } else if (sumCog < 0) {
        	result.angleError = -sumCog * turnMult * turnDirection;
        	result.cmdVel = searchVelocity * .5;
        }

        if (result.angleError > turnMax)
        	result.angleError = turnMax;
        else if (result.angleError < -turnMax)
        	result.angleError = -turnMax;


//        } else if (right && left) {
//        	Logger::chat("Straight on");
//        	result.cmdVel = searchVelocity;
//        	result.angleError = 0.0;
//        }
//        else if (right) {
//            result.cmdVel = -0.1 * turnDirection;
//            result.angleError = -centeringTurn*turnDirection;
//        }
//        else if (left){
//        	result.cmdVel = -0.1 * turnDirection;
//            result.angleError = centeringTurn*turnDirection;
//        }
//        else
//        {
//            result.cmdVel = searchVelocity;
//            result.angleError = 0.0;
//        }


//        else if (sumCog > 0) {
//        	Logger::chat("COG: to the right");
//        	result.angleError = -sumCog * 10 * turnDirection;
//        	result.cmdVel = searchVelocity * .75;
//        } else if (sumCog < 0) {
//        	Logger::chat("COG: to the left");
//        	result.angleError = -sumCog * 10 * turnDirection;
//        	result.cmdVel = searchVelocity * .75;
//        }

//        if (seenEnoughCenterTags) {
//        	result.angleError = sumCog * 15;
//        	result.cmdVel = searchVelocity * .75;
//        } else {
//        	result.angleError = -sumCog * 15;
//        	result.cmdVel = searchVelocity * .75;
//        }





        //must see greater than this many tags before assuming we are driving into the center and not along an edge.
    	// XXX: Tuneable was:
        //if (count > seenEnoughCenterTagsCount)
        if (count > seenEnoughCenterTagsCount && countLeft >= count/3 && countRight >= count/3)
        {
            if (!seenEnoughCenterTags) {
            	Logger::chat("Triggered approach. seen: %d l: %d r: %d", count, countLeft, countRight);
            }
            seenEnoughCenterTags = true; //we have driven far enough forward to be in the circle.
            timeWithoutSeeingEnoughCenterTags = time(0);
        }
        if (count > 0) //reset gaurd to prevent drop offs due to loosing tracking on tags for a frame or 2.
        {
            timeWithoutSeeingEnoughCenterTags = time(0);
        }
        //time since we dropped below countGuard tags
        timeElapsedSinceTimeSinceSeeingEnoughCenterTags = time(0) - timeWithoutSeeingEnoughCenterTags;

        //we have driven far enough forward to have passed over the circle.
        if (count == 0 && seenEnoughCenterTags && timeElapsedSinceTimeSinceSeeingEnoughCenterTags > 1) {
            centerSeen = false;
        }
        centerApproach = true;
        prevCount = count;
        count = 0;
    }
    //was on approach to center and did not seenEnoughCenterTags
    //for maxTimeAllowedWithoutSeeingCenterTags seconds so reset.
    else if (centerApproach) {
        result.goalDriving = false;
        int maxTimeAllowedWithoutSeeingCenterTags = 6; //seconds

        timeElapsedSinceTimeSinceSeeingEnoughCenterTags = time(0) - timeWithoutSeeingEnoughCenterTags;
        if (timeElapsedSinceTimeSinceSeeingEnoughCenterTags > maxTimeAllowedWithoutSeeingCenterTags)
        {
            //go back to drive to center base location instead of drop off attempt
            reachedCollectionPoint = false;
            seenEnoughCenterTags = false;
            centerApproach = false;
        }
        else
        {
            result.cmdVel = searchVelocity;
            result.angleError = 0.0;
        }

        right = false;
        left = false;
        count = 0;
        centerSeen = false;
        Logger::chat("DOC: Aborted center approach because I din't see enough tags.");
        return;

    }

    if (!centerSeen && seenEnoughCenterTags)
    {
        reachedCollectionPoint = true;
        timerStartTime = time(0);
        result.goalDriving = false;
        centerApproach = false;
        result.timer = true;
    }

    return;
}


void DropOffController::setDataLocations(geometry_msgs::Pose2D center, geometry_msgs::Pose2D currentOdom, geometry_msgs::Pose2D currentMap, bool odom, float sync) {

    centerLocation = center;
    currentLocationOdom = currentOdom;
    currentLocationMap = currentMap;
    useOdom = odom;
    timerTimeElapsed = sync;
    calculateDecision();

}


void DropOffController::reset() {
    result.cmdVel = 0;
    result.angleError = 0;
    result.fingerAngle = -1;
    result.wristAngle = -1;
    result.goalDriving = false; //set true for when driving is goal location oriented and false when it is target or time oriented
    result.centerGoal; //goal that is center location or based upon the center location.
    result.reset = false;
    spinner = M_PI_4;
    addSpinSize = 0;
    prevCount = 0;

    left = false;
    right = false;
    centerSeen = false;

    //reset flags
    reachedCollectionPoint = false;
    seenEnoughCenterTags = false;
    circularCenterSearching = false;

    useOdom = false;
}

void DropOffController::setDataTargets(int ccount, int lleft, int rright, double ssumCog)
{
    count = ccount;
    sumCog = ssumCog;
    if (rright > 0)
    {
        right = true;
    }
    else
    {
        right = false;
    }
    if (lleft > 0)
    {
        left = true;
    }
    else
    {
        left = false;
    }

    countLeft = lleft;
    countRight = rright;
}

DropOffController::~DropOffController() {

}
