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

#include <MapManager.h>
#include <string>
#include <sstream>
#include <algorithm>
#include <limits>

Define_Module(MapManager);

/**
 * Initialize basic parameters (read from omnetpp.ini) and open scenario-database for building-access.
 */
void MapManager::initialize() {

    // Read from omnetpp.ini and assign projection offsets (e.g. copied from sumo-net-files or vissim-files)
    offsetX = par ("offsetx").doubleValue();
    offsetY = par ("offsety").doubleValue();

    // Get database name from omnetpp.ini
    const char* database = par("database").stdstringValue().c_str();
    EV << "MapManager initialize: open db=" << database << endl;

    // Open db and query table-names, just to be sure that the db has the appropriate format
    int rc = sqlite3_open(database, &db);
    if (rc) {
        fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
        EV << "Can't open database: " << sqlite3_errmsg(db) << endl;
        sqlite3_close(db);
    }
    char *zErrMsg;
    char **result;
    int nrow,ncol;
    std::string query = "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name";
    rc = sqlite3_get_table(db,query.c_str(),&result,&nrow,&ncol,&zErrMsg);
    if (rc == SQLITE_OK){
        EV << "tables:" << endl;
        for (int i = 1; i <= nrow; i++) {
            EV << i << ") " << result[i] << endl;
        }
    }
}

/**
 * Core method of the mapManager to get the buildings in the area between the sender and the receiver node.
 * In every call it gets the information of building-table and returns these to the propagation model.
 * When a building is not cached, it also gets more detailed information about the walls and corners.
 */
std::vector<int> MapManager::getBuildingsInArea(const Coord& txPos, const Coord& rxPos) {

    std::vector<int> areaBuildings;

    // The area is a circle around the midpoint between sender and receiver
    // Determine midpoint between tx and rx
    double px = txPos.x + (rxPos.x - txPos.x) / 2;
    double py = txPos.y + (rxPos.y - txPos.y) / 2;

    // Calc (squared) radius around the midpoint through the distance between tx and rx
    double txrxDist = txPos.distance(rxPos);
    double sqrRadius = txrxDist * txrxDist / 4; // radius² for pythagorasian distance calculation

    // Compile query for buildings in area (which means buildings in the radius)
    std::ostringstream squaredRadius, minX, maxX, minY, maxY;
    squaredRadius << std::fixed << sqrRadius;
    minX << std::fixed << "(" << offsetX << "+min_x-" << px << ")*(" << offsetX << "+min_x-" << px << ")";
    maxX << std::fixed << "(" << offsetX << "+max_x-" << px << ")*(" << offsetX << "+max_x-" << px << ")";
    minY << std::fixed << "(" << offsetY << "+min_y-" << py << ")*(" << offsetY << "+min_y-" << py << ")";
    maxY << std::fixed << "(" << offsetY << "+max_y-" << py << ")*(" << offsetY << "+max_y-" << py << ")";

    std::ostringstream buildingQuery;
    buildingQuery << "SELECT id, min_x, min_y, max_x, max_y FROM Building WHERE (" <<
            squaredRadius.str() << " >= (" << minX.str() << "+" << minY.str() << ")) OR (" <<
            squaredRadius.str() << " >= (" << minX.str() << "+" << maxY.str() << ")) OR (" <<
            squaredRadius.str() << " >= (" << maxX.str() << "+" << minY.str() << ")) OR (" <<
            squaredRadius.str() << " >= (" << maxX.str() << "+" << maxY.str() << "))";
    //EV << "MapManager getBuildingsinArea building-query=" << buildingQuery.str() << endl;
    resultQuery rQ = getResultSQL(buildingQuery.str());
    //EV << "MapManager getBuildingsinArea got n=" << rQ.nrows << endl;

    // Loop over each building in area
    for (unsigned int bldg = 1; bldg <= rQ.nrows; bldg++) {

        // Mark buildings in area to return to the propagation model
        int bId = atoi(rQ.result[rQ.ncols * bldg]);
        areaBuildings.push_back(bId); // push the id only in the return list

        // Get more detailed information about the building and cache the compiled building-object in the allBuildings-list
        if (isBuildingAlloc(bId) == false) {

            // Store abstracted area
            float bMinX = atof(rQ.result[rQ.ncols * bldg + 1]);
            float bMinY = atof(rQ.result[rQ.ncols * bldg + 2]);
            float bMaxX = atof(rQ.result[rQ.ncols * bldg + 3]);
            float bMaxY = atof(rQ.result[rQ.ncols * bldg + 4]);

            // Query from- and to-corners from the wall and corner tables
            std::ostringstream fromCornerQuery;
            fromCornerQuery << "SELECT Wall.sequence_number, Corner.id, Corner.x, Corner.y FROM Corner, Wall, Building " <<
                    "WHERE (Corner.id=Wall.from_corner_id AND Wall.building_id=Building.id AND Building.id=" << bId << ") ORDER BY Wall.sequence_number";
            resultQuery rQfc = getResultSQL(fromCornerQuery.str());

            std::ostringstream toCornerQuery;
            toCornerQuery << "SELECT Wall.sequence_number, Corner.id, Corner.x, Corner.y FROM Corner, Wall, Building " <<
                    "WHERE (Corner.id=Wall.to_corner_id AND Wall.building_id=Building.id AND Building.id=" << bId << ") ORDER BY Wall.sequence_number";
            resultQuery rQtc = getResultSQL(toCornerQuery.str());

            // Check if the building has the same number of from- and to-corners, which is actually the number of walls
            if (rQfc.nrows == rQtc.nrows) {
                int numWalls = rQfc.nrows;
                // And check if the building has any walls at least
                if (numWalls > 0) {
                    std::vector<Wall> walls;
                    // Fill detailed information for each wall
                    for (int wallCnt = 1; wallCnt <= numWalls; wallCnt++) {
                        int seqNumberFc = atoi(rQfc.result[rQfc.ncols * wallCnt]);
                        int idFc = atoi(rQfc.result[rQfc.ncols * wallCnt + 1]);
                        float posxFc = atof(rQfc.result[rQfc.ncols * wallCnt + 2]);
                        float posyFc = atof(rQfc.result[rQfc.ncols * wallCnt + 3]);

                        int seqNumberTc = atoi(rQtc.result[rQtc.ncols * wallCnt]);
                        int idTc = atoi(rQtc.result[rQtc.ncols * wallCnt + 1]);
                        float posxTc = atof(rQtc.result[rQtc.ncols * wallCnt + 2]);
                        float posyTc = atof(rQtc.result[rQtc.ncols * wallCnt + 3]);

                        // Just to be really save
                        if (seqNumberFc == seqNumberTc) {
                            Wall w(Corner(idFc, posxFc + offsetX, posyFc + offsetY),
                                    Corner(idTc, posxTc + offsetX, posyTc + offsetY));
                            walls.push_back(w);
                        }
                        else {
                            EV << "ERROR unequal sequence numbers" << endl;
                        }
                    }
                    // Write the whole building-information (id, abstracted area and detailed wall/corner-info) to allBuildings
                    allBuildings.push_back(Building(bId, bMinX + offsetX, bMinY + offsetY, bMaxX + offsetX, bMaxY + offsetY, walls));
                }
                else {
                    EV << "WARN: building without walls: " << bId << endl;
                }
            }
            else {
                EV << "ERROR: unequal number of from and to corners" << endl;
            }
        }
    }

    // Finally, print some info about the buildings in the area
    // (either freshly extracted from the scenario-db and now cached in allBuildings or already cached from previous calls)
    //std::vector<Building>::iterator itb;
    //EV << "MapManager allBuildings" << endl;
    //for (itb = allBuildings.begin(); itb < allBuildings.end(); itb++) {
    //    if (std::find(areaBuildings.begin(), areaBuildings.end(), itb->ID) != areaBuildings.end()) {
    //        EV << itb->ID << ": Area((" << itb->minX << "," << itb->minY << ")-(" << itb->maxX << "," << itb->maxY << ")) Corners(";
    //        std::vector<Wall>::iterator itw;
    //        for (itw = itb->walls.begin(); itw < itb->walls.end(); itw++) {
    //            EV << "(" << itw->from_corner.id << "-" << itw->from_corner.posX << "," << itw->from_corner.posY << ")";
    //        }
    //        EV << ")" << endl;
    //    }
    //}

    return areaBuildings;
}

/**
 * Returns the positions of the intersections of the building which is between sender and receiver.
 * Actually iterates through all walls of the building and checks if it intersects with
 * the line segment drawn between the sender and receiver position.
 */
std::vector<Coord> MapManager::getIntersectionBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId){
    std::vector<Building>::iterator itBuildings;
    std::vector<Coord> intersections;
    for (itBuildings = allBuildings.begin(); itBuildings < allBuildings.end(); itBuildings++)
    {
        if(bId==(int)itBuildings->ID){
            std::vector<Wall>::iterator itWalls;
            for (itWalls = itBuildings->walls.begin(); itWalls < itBuildings->walls.end(); itWalls++){
                Coord v = Coord(0,0);
                v = getIntersection(tx_x,tx_y,rx_x,rx_y,itWalls->from_corner.posX,itWalls->from_corner.posY,itWalls->to_corner.posX,itWalls->to_corner.posY);
                if (v.x != 0 || v.y!=0){
                    intersections.push_back(v);
                }
            }
        }
    }
    return intersections;
}

/**
 * Get diffraction corners by checking if there is
 * no obstruction between the line of sight of first node (sender) and the corner of the building and
 * no obstruction between this corner and the line of sight of the second node (receiver).
 */
std::vector<Coord> MapManager::getDiffractionCorners(float tx_x, float tx_y, float rx_x, float rx_y, int bId) {
    // Save founde diffraction corners (return object)
    std::vector<Coord> diffCorners;

    std::vector<Building>::iterator itBuildings;
    for (itBuildings = allBuildings.begin(); itBuildings < allBuildings.end(); itBuildings++) {
        if (bId == (int) itBuildings->ID) {
            //EV << "MapManager getDiffractionCorners: building=" << bId << endl;
            // Get unobstructed Corners
            std::vector<Wall>::iterator itCornerWalls, itWalls;
            // Loop over all corners (from_corners of walls) of the building and
            // check if their line of sight to sender and receiver intersects with other walls
            for (itCornerWalls = itBuildings->walls.begin(); itCornerWalls < itBuildings->walls.end(); itCornerWalls++) {
                Corner potentialCorner = itCornerWalls->from_corner;
                bool dcFound = false;
                //EV << "MapManager getDiffractionCorners: potentialCorner=" << potentialCorner.id;
                // Loop over other walls of the building (omit walls with own corner)
                for (itWalls = itBuildings->walls.begin(); itWalls < itBuildings->walls.end(); itWalls++) {
                    if ((potentialCorner.id != itWalls->from_corner.id) && (potentialCorner.id != itWalls->to_corner.id)) {
                        //EV << ", otherWall=" << itWalls->from_corner.id << "-" << itWalls->to_corner.id;
                        // First check the line of sight between the sender and the corner
                        Coord intersection = Coord(0, 0);
                        intersection = getIntersection(tx_x, tx_y, potentialCorner.posX, potentialCorner.posY,
                                itWalls->from_corner.posX, itWalls->from_corner.posY, itWalls->to_corner.posX, itWalls->to_corner.posY);
                        if ((intersection.x != 0) || (intersection.y != 0)) {
                            // This corner is no diffraction corner and no further checks are needed
                            dcFound = false;
                            break;
                        }
                        else {
                            // When line of sight to the sender does not intersect, check if the line of sight to the receiver is obstructed
                            intersection = getIntersection(rx_x, rx_y, potentialCorner.posX, potentialCorner.posY,
                                    itWalls->from_corner.posX, itWalls->from_corner.posY, itWalls->to_corner.posX, itWalls->to_corner.posY);
                            if ((intersection.x != 0) && (intersection.y != 0)) {
                                // This corner is no diffraction corner and no further checks are needed
                                dcFound = false;
                                break;
                            }
                            else {
                                // When neither sender->corner nor corner->receiver is obstructed, diffraction can happen
                                // However, check other remaining walls (no break here)
                                dcFound = true;
                            }
                        }
                    }
                }
                if (dcFound) {
                    // Save this corner, as it is the diffraction corner
                    diffCorners.push_back(Coord(potentialCorner.posX, potentialCorner.posY));
                }
            }
        }
    }
    return diffCorners;
}

/**
 * Check if the abstracted area of a building is between two nodes (e.g. sender and receiver).
 * Actually determines the line of sight from the node positions and
 * the building geometries (min-max-area) from the bId and checks if there is an intersection.
 * (Code by Micha).
 */
bool MapManager::isAreaBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId) {
    std::vector<Building>::iterator itBuildings;
    bool iAB = false;
    for (itBuildings = allBuildings.begin(); itBuildings < allBuildings.end(); itBuildings++) {
        if(bId==(int)itBuildings->ID){
            //EV << "MapManager isAreaBetween bId=" << bId << endl;
            Coord v = Coord(0,0);
            v = getIntersection(tx_x,tx_y,rx_x,rx_y,itBuildings->maxX,itBuildings->maxY,itBuildings->maxX,itBuildings->minY);
            if (v.x != 0 || v.y!=0){
                iAB=true;
                break;
            }
            v = getIntersection(tx_x,tx_y,rx_x,rx_y,itBuildings->maxX,itBuildings->maxY,itBuildings->minX,itBuildings->maxY);
            if (v.x != 0 || v.y!=0){
                iAB=true;
                break;
            }
            v = getIntersection(tx_x,tx_y,rx_x,rx_y,itBuildings->minX,itBuildings->minY,itBuildings->minX,itBuildings->maxY);
            if (v.x != 0 || v.y!=0){
                iAB=true;
                break;
            }
            v = getIntersection(tx_x,tx_y,rx_x,rx_y,itBuildings->minX,itBuildings->minY,itBuildings->maxX,itBuildings->minY);
            if (v.x != 0 || v.y!=0){
                iAB=true;
                break;
            }
        }
    }
    return iAB;
}

/**
 * Check if a building is between two nodes (e.g. sender and receiver). More detailed check than "isAreaBetween()".
 * Actually determines the line of sight from the node positions and
 * the building geometries (individual walls) from the bId and checks if there is an intersection.
 * (Code by Micha).
 */
bool MapManager::isBuildingBetween(float tx_x, float tx_y, float rx_x, float rx_y, int bId) {
    std::vector<Building>::iterator itBuildings;
    bool iBB = false;
    for (itBuildings = allBuildings.begin(); itBuildings < allBuildings.end(); itBuildings++) {
        if(bId==(int)itBuildings->ID){
            //EV << "MapManager isBuildingBetween bId=" << bId << endl;
            std::vector<Wall>::iterator itWalls;
            for (itWalls = itBuildings->walls.begin(); itWalls < itBuildings->walls.end(); itWalls++){
                Coord v = Coord(0,0);
                v = getIntersection(tx_x,tx_y,rx_x,rx_y,itWalls->from_corner.posX,itWalls->from_corner.posY,itWalls->to_corner.posX,itWalls->to_corner.posY);
                if (v.x != 0 || v.y!=0){
                    iBB=true;
                    break;
                }
            }
        }
    }
    return iBB;
}

/**
 * Returns the intersection point between two vector line segments.
 * The first line is determined by P1 and P2, the second one by P3 and P4.
 */
Coord MapManager::getIntersection(float P1x, float P1y, float P2x, float P2y, float P3x, float P3y, float P4x, float P4y) {

    //EV << "MapManager getIntersection from=(" << P1x << "," << P1y << " - " << P2x << "," << P2y << ") to=(" << P3x << "," << P3y << " - " << P4x << "," << P4y << ")" << endl;

    bool intersect;
    double a, b;

    // Set up the linear system, where both line segments are equal:
    // P1 + a (P2 - P1) = P3 + b (P4 - P3)  ... (1)
    // a (P2 - P1) + b (P3 - P4) = (P3 - P1)  ... (1a)

    // And solve the system with the Cramer-rule (determine "a" and "b")
    // a = detA/det; b = detB/det  ... (2)
    // with  det = |P2-P1  P3-P4|;  detA = |P3-P1  P3-P4|;  detB = |P2-P1  P3-P1|  ... (3)

    double det = (P2x - P1x) * (P3y - P4y) - (P2y - P1y) * (P3x - P4x);
    if (abs(det) < std::numeric_limits<double>::epsilon()) {
        intersect = false; // no intersection (no solution for the system), when the denominator == 0
    }
    else {
        double detA = (P3x - P1x) * (P3y - P4y) - (P3y - P1y) * (P3x - P4x);
        double detB = (P2x - P1x) * (P3y - P1y) - (P2y - P1y) * (P3x - P1x);
        a = detA / det;
        b = detB / det;
        if (a >= 0 && a <= 1 && b >= 0 && b <= 1) {
            intersect = true; // intersection in the line segment
        }
        else {
            intersect = false; // intersection eventually possible, but not within the given segment
        }
    }

    if (intersect) {
        // Apply "a" to the left side of the equation (1) to determine the point of intersection
        double Pix = P1x + a * (P2x - P1x);
        double Piy = P1y + a * (P2y - P1y);
        //double Piix = P3x + b * (P4x - P3x);
        //double Piiy = P3y + b * (P4y - P3y);
        //EV << "MapManager getIntersection found=(" << Pix << "," << Piy << "), alt=(" << Piix << "," << Piiy << ")" << endl;
        return Coord(Pix, Piy);
    }
    else {
        return Coord(0, 0);
    }


}

/**
 * Execute a given sql-query on the scenario-database and format the result for later processing.
 * (Code by Micha).
 */
MapManager::resultQuery MapManager::getResultSQL(std::string SQLExpression) {
    char *zErrMsg;
    char **result;
    resultQuery rQ;
    int rc;
    int nrow,ncol;

    rc = sqlite3_get_table(db,SQLExpression.c_str(),&result,&nrow,&ncol,&zErrMsg);
    if (rc == SQLITE_OK){
        rQ.ncols = ncol;
        rQ.nrows = nrow;
        rQ.result = result;
    }

    if (zErrMsg){
        EV << "SQLite ERROR: " << zErrMsg << endl;
    }
    return rQ;
}

/**
 * Determines if a building and its detailed information is already in the cache,
 * otherwise it has to be queried from the scenario-database.
 * (Code by Micha).
 */
bool MapManager::isBuildingAlloc(int bId) {
    std::vector<Building>::iterator itBuildings;
    bool iAB = false;
    for (itBuildings = allBuildings.begin(); itBuildings < allBuildings.end(); itBuildings++) {
        if(bId==(int)itBuildings->ID){
            iAB = true;
        }
    }
    return iAB;
}
