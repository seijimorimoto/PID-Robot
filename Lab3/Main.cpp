#include <cmath>
#include <queue>
#include "aria.h"
#define PI 3.14159265

const int BASE_VEL = 50;
const int DESIRED_DIST = 500;
const int MAX_HISTORY = 10;
const double KP = 0.3;
const double KI = 0.01;
const double KD = -0.1;


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
	std::queue<int> errorHistory;

	while (true)
	{
		bool badReadings[4] = { false, false, false, false };
		int badReadingsCount = 0;

		// Get sonar readings for right sensors (4 to 7).
		for (int i = 0; i < 4; i++)
		{
			sonarSensor[i] = robot.getSonarReading(i+4);
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
			if (error < -1000) error = -1000;

			// If the previous error is impossible, that means this is the first reading, so we set
			// the previous error value to 0 (so that the error difference is 0).
			//errorPrev = (errorPrev >= 5000) ? error : errorPrev;

			// Calculate the error difference.
			const int errorDif = error - errorPrev;
			errorPrev = error;

			// Check if the history of errors is full.
			if (errorHistory.size() ==  MAX_HISTORY)
			{
				// Remove the oldest error
				const int errorOld = errorHistory.front();
				errorSum = errorSum + error - errorOld;
				errorHistory.pop();
				errorHistory.push(error);
			}
			else
			{
				errorHistory.push(error);
				errorSum += error;
			}

			const double pidOutput = KP * error + KI * errorSum + KD * errorDif;
			int leftVel = BASE_VEL - pidOutput;
			int rightVel = BASE_VEL + pidOutput;

			if (leftVel < 0) leftVel = 0;
			if (rightVel < 0) rightVel = 0;
			
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