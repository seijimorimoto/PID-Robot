#include <cmath>
#include "aria.h"

const int BASE_VEL = 100;
const int DESIRED_DIST = 500;
//const int MAX_HISTORY = 10;
const double MAX_VEL = 2000;
const double MIN_VEL = 0;
const double KP = 0.4;
const double KI = 0.0005;
const double KD = 0.1;


int main(int argc, char **argv)
{
	// create instances
	Aria::init();
	ArRobot robot;
	ArSensorReading *sonarSensor[8];

	// parse command line arguments
	ArArgumentParser argParser(&argc, argv);
	argParser.loadDefaultArguments();

	// connect to robot (and laser, etc)
	ArRobotConnector robotConnector(&argParser, &robot);

	if (robotConnector.connectRobot())
		std::cout << "Robot connected!" << std::endl;

	robot.runAsync(false);
	robot.lock();
	robot.enableMotors();
	robot.unlock();

	int errorPrev = 0;
	int errorSum = 0;
	unsigned int distances[4];

	ArUtil::sleep(7000);

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

			// Calculate current error.
			int error = DESIRED_DIST - distanceMin;
			if (error < -500) error = -500;

			// Calculate the error difference.
			const int errorDif = error - errorPrev;
			errorPrev = error;

			// Add the error to the accumulated error over time.
			errorSum += error;

			double pidOutput = KP * error + KI * errorSum + KD * errorDif;
			std::cout << pidOutput << std::endl;
			if (BASE_VEL + pidOutput > MAX_VEL)
			{
				pidOutput = MAX_VEL - BASE_VEL;
				errorSum -= error;
			}
			else if (BASE_VEL - pidOutput > MAX_VEL)
			{
				pidOutput = -1 * (MAX_VEL - BASE_VEL);
				errorSum -= error;
			}

			double leftVel = BASE_VEL - pidOutput;
			double rightVel = BASE_VEL + pidOutput;

			if (leftVel < MIN_VEL) leftVel = MIN_VEL;
			if (rightVel < MIN_VEL) rightVel = MIN_VEL;

			std::cout << leftVel << " " << rightVel << std::endl;

			robot.setVel2(leftVel, rightVel);
		}

		ArUtil::sleep(100);
	}

	// stop the robot
	robot.lock();
	robot.stop();
	robot.unlock();

	// terminate all threads and exit
	Aria::exit();

	return 0;
}