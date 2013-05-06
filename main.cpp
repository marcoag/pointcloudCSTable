/*
 *    Copyright (C) 2012-2013 by Luis J. Manso
 *    Robotics and Artificial Vision Laboratory (University of Extremadura)
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <QtCore>
#include "worker.h"

int main(int argc, char* argv[])
{
	// Read point clouds from hard disk
	printf("Reading point clouds from hard disk...\n");
	boost::shared_ptr< PCLPointCloud > real_points = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	boost::shared_ptr< PCLPointCloud > virtual_points0 = boost::shared_ptr< PCLPointCloud >(new PCLPointCloud);
	readPCD(argv[1], real_points);
	readPCD(argv[2], virtual_points0);

	// ICP
	//

	// Paper
	Worker *worker = new Worker(real_points, virtual_points0);

	// Visualization


	// deletes
	delete worker;
	return 0;
}



