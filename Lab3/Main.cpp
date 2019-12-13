#include <cmath>
#include "aria.h"

// Defining constants.
const int BASE_VEL = 100;
const int DESIRED_DIST = 500;
const double MAX_VEL = 2000;
const double MIN_VEL = 0;
const double KP = 0.3;
const double KI = 0.0005;
const double KD = 0.1;


int main(int argc, char **argv)
{
	// Create instances.
	Aria::init();
	ArRobot robot;
	ArSensorReading *sonarSensor[8];

	// Parse command line arguments.
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	// Connect to robot (and laser, etc).
	ArRobotConnector robotConnector(&argParser, &robot);
	if (robotConnector.connectRobot()) std::cout << "Robot connected!" << std::endl;

	// Enable the motors in the robot.
	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	// Declare / initialize variables.
	int errorPrev = 0;
	int errorSum = 0;
	unsigned int distances[4];

	// "Warm up" time.
	ArUtil::sleep(7000);

	// Main loop of the program. Keep it running indefinitely.
	while (true)
	{
		bool badReadings[4] = { false, false, false, false };
		int badReadingsCount = 0;

		// Get sonar readings for right sensors (4 to 7).
		for (int i = 0; i < 4; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i + 4);
			distances[i] = sonarSensor[i]->getRange();

			// If the sensor reading is above or equal to 5000, mark it as a bad reading and
			// increase the count of bad readings.
			if (distances[i] >= 5000)
			{
				badReadings[i] = true;
				badReadingsCount++;
			}
		}

		// Only continue if at least one reading was good.
		if (badReadingsCount < 4)
		{
			// Set the initial minimum distance to some impossible value.
			unsigned int distanceMin = 10000;

			// Get the minimum value of the good readings.
			for (int i = 0; i < 4; i++)
			{
				if (!badReadings[i] && distances[i] < distanceMin)
					distanceMin = distances[i];
			}

			// Calculate the current error and set a maximum negative value.
			int error = DESIRED_DIST - distanceMin;
			if (error < -500) error = -500;

			// Calculate the error difference.
			const int errorDif = error - errorPrev;
			errorPrev = error;

			// Add the error to the accumulated error over time.
			errorSum += error;
			
			// Calculate the output of the PID process.
			double pidOutput = KP * error + KI * errorSum + KD * errorDif;
			
			// If the velocity resulting from combining the base velocity and the PID output
			// exceed the maximum allowed velocity, set the output of the PID to be the value that
			// makes the final velocity the maximum. Then remove the current error from the
			// accumulated error to prevent overshooting.
			if (BASE_VEL + pidOutput > MAX_VEL)
			{
				pidOutput = MAX_VEL - BASE_VEL;
				errorSum -= error;
			}

			// If the velocity resulting from substracting the PID output from the base velocity
			// exceed the maximum allowed velocity, set the output of the PID to be the value that
			// makes the final velocity the maximum. Then remove the current error from the
			// accumulated error to prevent overshooting.
			else if (BASE_VEL - pidOutput > MAX_VEL)
			{
				pidOutput = -1 * (MAX_VEL - BASE_VEL);
				errorSum -= error;
			}

			// Calculate the left and right velocity using the PID output and the base velocity.
			double leftVel = BASE_VEL - pidOutput;
			double rightVel = BASE_VEL + pidOutput;

			// When the left or right velocity is less than a minimum threshold, set them to the
			// that minimum value (don't allow them to go below).
			if (leftVel < MIN_VEL) leftVel = MIN_VEL;
			if (rightVel < MIN_VEL) rightVel = MIN_VEL;

			// Set the left and right wheel speed of the robot.
			robot.setVel2(leftVel, rightVel);
		}

		// Wait a little bit for the next iteration.
		ArUtil::sleep(100);
	}

	// Stop the robot.
	robot.lock();
	robot.stop();
	robot.unlock();

	// Terminate all threads and exit.
	Aria::exit();

	return 0;
}