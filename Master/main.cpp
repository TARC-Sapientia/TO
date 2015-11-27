/* Teleoperation System for Mobile Manipulators Framework
 *
 * Copyright (C) 2015 
 * RECONFIGURABLE CONTROL OF ROBOTIC SYSTEMS OVER NETWORKS Project
 * Robotics and Control Systems Laboratory
 * Department of Electrical Engineering
 * Sapientia Hungarian University of Transylvania
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * For more details see the project homepage:
 * <http://www.ms.sapientia.ro/~martonl/MartonL_Research_TE.htm>
 */



#include "Master.h"
#include <iostream>
#include <string>

#ifdef __linux__
#include <sys/time.h>
#include <sys/resource.h>
#endif

using namespace std;

int main(int argc, char* argv[])
{
#ifdef _WIN32
	DWORD dwError;
	if(!SetPriorityClass(GetCurrentProcess(), HIGH_PRIORITY_CLASS))
	{
		dwError = GetLastError();
	}	
#elif __linux__
	setpriority(PRIO_PROCESS, 0, -20);
#endif
	if (argc != 2) {
		cout << "Usage: " << argv[0] << " config_file_path/config_file_name.xml" << endl;
		return -1;
	}

	try
	{
		Master master;
		master.Initialize(string(argv[1]));

		cout << "Press ENTER to measure clock difference.." << endl;
		cin.ignore(std::numeric_limits<streamsize>::max(),'\n');
		master.GetClockDifferences();

		cout << "Press ENTER to start.." << endl;
		cin.ignore(std::numeric_limits<streamsize>::max(),'\n');

		master.Start();

		char c;
		do 
		{
			c = cin.get();
		} while ( c != 'q');
	}
	catch (exception& e)
	{
		cout << e.what() << endl;
	}

	return 0;
}
