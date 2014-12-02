//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef MAPMANAGER_H_
#define MAPMANAGER_H_

#include <omnetpp.h>
#include "INETDefs.h"
#include "Coord.h"
#include "sqlite3.h"


class INET_API MapManager : public cSimpleModule {

public:
    ~MapManager() {};

    /**
     * Initialize method.
     */
    virtual void initialize();

    /**
     * Gets the buildings (Ids) in the area (radius) of the sender.
     * Extracts more detailed information and store it in the allBuildings-list.
     */
    virtual std::vector<int> getBuildingsInArea(const Coord& txPos, const Coord& rxPos);

    /**
     * Returns the positions of possibly multiple intersections of the building which is between sender and receiver.
     */
    virtual std::vector<Coord> getIntersectionBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId);

    /**
     * Returns all possible diffraction corners which are not obstructed by other walls of the building.
     */
    virtual std::vector<Coord> getDiffractionCorners(float tx_x, float tx_y, float rx_x, float rx_y, int bId);

        /**
     * Examines if the abstracted area of a building is between the sender and the receiver.
     */
    virtual bool isAreaBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId);

    /**
     * Examine if a building (with more detailed geometries) is between the sender and the receiver.
     */
    virtual bool isBuildingBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId);

protected:
    /**
     * Offset of the playground to the (0,0)-position equator,prime meridian.
     * Needs to be in-line with the net-offset from the traffic simulator
     */
    double offsetX;
    double offsetY;

    /** Handle for database access and queries. */
    sqlite3 *db;

    /** Structure of a corner. */
    struct Corner{
        unsigned int id;
        float posX;
        float posY;
        Corner(unsigned int i, float x, float y):id(i), posX(x), posY(y){}
    };

    /** Structure of a wall. */
    struct Wall{
        Corner from_corner;
        Corner to_corner;
        Wall(Corner fc, Corner tc):from_corner(fc), to_corner(tc){};
    };

    /** Structure of a building. */
    struct Building{
        unsigned int ID;
        float minX;
        float minY;
        float maxX;
        float maxY;
        std::vector<Wall> walls;
        Building() : ID(0), minX(0),minY(0),maxX(0),maxY(0), walls(){}
        Building(unsigned int vID, float miX, float miY, float maX, float maY, std::vector<Wall> w) : ID(vID), minX(miX),minY(miY),maxX(maX),maxY(maY), walls(w){}
    };

    /** Structure of a SQLite result. */
    struct resultQuery{
        char** result;
        uint32_t nrows;
        uint32_t ncols;
        resultQuery() : nrows(0), ncols(0){}
        resultQuery(char** vResult, uint32_t vNrows, uint32_t vNcols) : result(vResult), nrows(vNrows),ncols(vNcols){}
    };

    /** Cache for all buildings to minimize the database requests. */
    std::vector <Building> allBuildings;

    /**
     * Returns the intersection point between two vector line segments, determined by the input points.
     */
    Coord getIntersection(float P1x, float P1y, float P2x, float P2y, float P3x, float P3y, float P4x, float P4y);

    /**
     * Executes and formats a sql-query.
     */
    resultQuery getResultSQL(std::string SQLExpression);

    /**
     * Checks if buildings are already cached in the allBuildings-list.
     */
    bool isBuildingAlloc(int bId);
};

#endif /* MAPMANAGER_H_ */
