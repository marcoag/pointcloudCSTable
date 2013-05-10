/*
 *    Copyright (C) 2010 by RoboLab - University of Extremadura
 *
 *    This file is part of RoboComp
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
#ifndef SerializableMatrix_H
#define SerializableMatrix_H

#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>

template <typename T>
class SerializableMatrix
{
public:
	bool readLine(char *buffer, FILE *file)
	{
		int i=0;
		while(fread(buffer+i, 1, 1, file)==1)
		{
			if (*(buffer+i) == '\n')
			{
				*(buffer+i) = '\0';
				return false;
			}
			i = i + 1;
		}
		*(buffer+i) = '\0';
		return true;
	}
	bool load(std::string path)
	{
		FILE *fd = fopen(path.c_str(), "r");
		if (fd != NULL)
		{
			char buff[1025];
			unsigned int rows, columns, depth;
			// rows
			readLine(buff, fd);
			rows = atoi(buff);
			// cols
			readLine(buff, fd);
			columns = atoi(buff);
			// depth
			readLine(buff, fd);
			depth = atoi(buff);
			// data
			resize(rows, columns, depth);
			for (uint64_t i=0; i<rows*columns*depth; ++i)
			{
				readLine(buff, fd);
				data[i] = boost::lexical_cast<T>(std::string(buff));
			}
			fclose(fd);
			return true;
		}
		else
		{
			printf ("Can't open %s for reading\n", path.c_str());
			fflush(stdout);
			exit(-3);
			return false;
		}
	}

	void save(std::string path)
	{
		try
		{
			std::ostream output(path);
			output << width << std::endl;
			output << height << std::endl;
			output << depth << std::endl;
			for (uint64_t i=0; i<width*height*depth; i++)
			{
				output << data[i] << std::endl;
			}
		}
		catch (...)
		{
			printf ("Can't open %s for writting\n", path.c_str());
			fflush(stdout);
			exit(-1);
		}
	}

	SerializableMatrix()
	{
		width = height = depth = 0;
		data.resize(1);
	}

	void resize(int newWidth, int newHeight=1, int newDepth=1)
	{
		width = newWidth;
		height = newHeight;
		depth = newDepth;
		data.resize(width*height*depth);
	}

	inline void memset0()
	{
		memset(getDataPointer(), 0, sizeof(T)*width*height*depth);
	}

	inline std::vector<T> *getVector()
	{
		return &data;
	}

	inline unsigned int getWidth()
	{
		return width;
	}

	inline unsigned int getHeight()
	{
		return height;
	}

	inline unsigned int getDepth()
	{
		return depth;
	}

	inline unsigned int getSize()
	{
		if (width*height*depth != data.size()) exit(-934);
		return width*height*depth;
	}

	inline T *getDataPointer()
	{
		if (data.size() > 0)
			return &data[0];
		return NULL;
	}

	inline T get1D(unsigned int x)
	{
		if (data.size() > x) return data[x];
		else throw std::string("SerializableMatrix::get1D() out of bounds.");
	}

	inline T get2D(unsigned int x, unsigned int y)
	{
		if (data.size() > x + y*width) return data[x + y*width];
		else throw std::string("SerializableMatrix::get2D() out of bounds.");
	}

	inline T get3D(unsigned int x, unsigned int y, unsigned int z)
	{
		if (data.size() > x + y*width + z*height*width) return data[x + y*width + z*height*width];
		else throw std::string("SerializableMatrix::get3D() out of bounds.");
	}

	inline void set1D(unsigned int x, T val)
	{
		if (data.size() > x) data[x] = val;
		else throw std::string("SerializableMatrix::set1D() out of bounds.");
	}

	inline void set2D(unsigned int x, unsigned int y, T val)
	{
		if (data.size() > x + y*width) data[x + y*width] = val;
		else throw std::string("SerializableMatrix::set2D() out of bounds.");
	}

	inline void set3D(unsigned int x, unsigned int y, unsigned int z, T val)
	{
		if (data.size() > x + y*width + z*height*width) data[x + y*width + z*height*width] = val;
		else throw std::string("SerializableMatrix::set2D() out of bounds.");
	}

	inline void inc1D(unsigned int x)
	{
		if (data.size() > x) data[x] = data[x] + 1;
		else throw std::string("SerializableMatrix::inc1D() out of bounds.");
	}

	inline void inc2D(unsigned int x, unsigned int y)
	{
		if (data.size() > x + y*width) data[x + y*width] = data[x + y*width] + 1;
		else throw std::string("SerializableMatrix::inc2D() out of bounds.");
	}

	inline void inc3D(unsigned int x, unsigned int y, unsigned int z)
	{
		if (data.size() > x + y*width + z*height*width) data[x + y*width + z*height*width] = data[x + y*width + z*height*width] + 1;
		else throw std::string("SerializableMatrix::inc3D() out of bounds.");
	}

private:
	unsigned int width;
	unsigned int height;
	unsigned int depth;
	std::vector<T> data;
};

#endif
