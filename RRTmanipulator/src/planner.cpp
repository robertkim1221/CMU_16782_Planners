/*=================================================================
 *
 * planner.cpp
 *
 *=================================================================*/
#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

#include "RRTBase.h"

#include "PRM.h"
#include <memory>
/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10

#ifndef MAPS_DIR
#define MAPS_DIR "../maps"
#endif
#ifndef OUTPUT_DIR
#define OUTPUT_DIR "../output"
#endif


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;


#include <iomanip> // for std::setprecision

// Additional helper function to generate valid configurations
std::vector<std::vector<double>> generateValidConfigurations(int numConfigs, int numOfDOFs, double* map, int x_size, int y_size) {
    std::vector<std::vector<double>> validConfigs;
    while (validConfigs.size() < numConfigs) {
        std::vector<double> config(numOfDOFs);
        for (int i = 0; i < numOfDOFs; ++i) {
            config[i] = ((double)rand() / RAND_MAX) * 2 * PI;
        }

        if (IsValidArmConfiguration(config.data(), numOfDOFs, map, x_size, y_size)) {
            validConfigs.push_back(config);
        }
    }
    return validConfigs;
}
//*******************************************************************************************************************//
//                                                                                                                   //
//                                                GIVEN FUNCTIONS                                                    //
//                                                                                                                   //
//*******************************************************************************************************************//

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                          DEFAULT PLANNER FUNCTION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

void planner(
    double* map,
	int x_size,
	int y_size,
	double* armstart_anglesV_rad,
	double* armgoal_anglesV_rad,
    int numofDOFs,
    double*** plan,
    int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("the arm is already at the goal\n");
        return;
    }
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    int firstinvalidconf = 1;
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size) && firstinvalidconf) {
            firstinvalidconf = 1;
            printf("ERROR: Invalid arm configuration!!!\n");
        }
    }    
    *planlength = numofsamples;
    
    return;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              RRT IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRT(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
	// Timer start for metrics
    auto startTimer = std::chrono::high_resolution_clock::now();

    // Convert start and goal angles into std::vector format
    std::vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    std::vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    // Set RRT parameters
    int maxIterations = 100000;
    double stepSize = 0.6;

    // Create an instance of RRTBase
    RRTBase rrt(start, goal, maxIterations, stepSize, map, numofDOFs, x_size, y_size);

    // Run the RRT planning
    std::shared_ptr<Node> result = rrt.buildRRT();

    // If a path is found, retrace it into `plan`
    if (result != nullptr) {
        rrt.backtrackRRT(result, plan, planlength);
    } else {
        std::cout << "No path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
    }
   // Final metrics output
    auto finishTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - startTimer);

    std::cout << "RRT Planning Duration: " << totalDuration.count() << " ms" << std::endl;
    std::cout << "Path Length: " << *planlength << std::endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                         RRT CONNECT IMPLEMENTATION                                                //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTConnect(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
	// Timer start for metrics
    auto startTimer = std::chrono::high_resolution_clock::now();

    // Convert start and goal angles into std::vector format
    std::vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    std::vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    // Set RRTConnect parameters
    int maxIterations = 10000;
    double stepSize = 0.6;

    // Create an instance of RRTBase
    RRTBase rrt(start, goal, maxIterations, stepSize, map, numofDOFs, x_size, y_size);

    // Run the RRTConnect planning
    std::pair<std::shared_ptr<Node>, std::shared_ptr<Node>> connection = rrt.buildRRTConnect();

    // If a connection is found, extract the path
    if (connection.first && connection.second) {
        rrt.extractPathConnect(connection.first, connection.second, plan, planlength);
    } else {
        std::cout << "No path found!" << std::endl;
        *plan = nullptr;
        *planlength = 0;
    }
   // Final metrics output
    auto finishTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - startTimer);

    std::cout << "RRTConnect Planning Duration: " << totalDuration.count() << " ms" << std::endl;
    std::cout << "Path Length: " << *planlength << std::endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                           RRT STAR IMPLEMENTATION                                                 //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerRRTStar(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
	// Timer start for metrics
    auto startTimer = std::chrono::high_resolution_clock::now();

    // Convert start and goal angles into std::vector format
    std::vector<double> start(armstart_anglesV_rad, armstart_anglesV_rad + numofDOFs);
    std::vector<double> goal(armgoal_anglesV_rad, armgoal_anglesV_rad + numofDOFs);

    // Set RRT* parameters
    int maxIterations = 100000;
    double stepSize = 0.6;

    // Create an instance of RRTBase configured for RRT*
    RRTBase rrtStar(start, goal, maxIterations, stepSize, map, numofDOFs, x_size, y_size);

    // Build the RRT* tree, returning the node at the goal if path found
    std::shared_ptr<Node> pathFound = rrtStar.buildRRTStar();

    // If a path is found, retrace it into `plan`
    if (pathFound != nullptr) {
        rrtStar.backtrackRRT(pathFound, plan, planlength);
		std::cout << "RRTstar Path Found" << std::endl;
	} else {
        std::cout << "RRTStar Path NOT Found" << std::endl;
        *plan = nullptr;
        *planlength = 0;
    }


    // Final metrics output
    auto finishTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - startTimer);

    std::cout << "RRT* Planning Duration: " << totalDuration.count() << " ms" << std::endl;
}

//*******************************************************************************************************************//
//                                                                                                                   //
//                                              PRM IMPLEMENTATION                                                   //
//                                                                                                                   //
//*******************************************************************************************************************//

static void plannerPRM(
    double *map,
    int x_size,
    int y_size,
    double *armstart_anglesV_rad,
    double *armgoal_anglesV_rad,
    int numofDOFs,
    double ***plan,
    int *planlength)
{
    // Timer start for metrics
    auto startTimer = std::chrono::high_resolution_clock::now();

    // PRM Parameters
    int samplingSteps = 20;
    double neighborhood_size = 20.0;

    // PRM Instance with DOF and neighborhood setup
    PRMBase prmInstance(numofDOFs, neighborhood_size);

    // Graph containers
    std::vector<std::vector<int>> connectionGraph;  // Use vector of vectors for edges
    std::vector<std::unique_ptr<double[]>> configMap;  // Vector for configurations
    int configCounter = 0;
	int maxCounter = 1000;
    
	// Reserve space for graph (1000 nodes and edges)
    connectionGraph.resize(maxCounter);
    configMap.reserve(maxCounter);

	// Build PRM graph by sampling configurations
	while (configCounter < maxCounter) {
		// Sample random configuration
		auto sampledConfigPtr = prmInstance.generateRandomConfig();

		// Insert vertex if valid in configuration space
		if (IsValidArmConfiguration(sampledConfigPtr.get(), numofDOFs, map, x_size, y_size)) {
			configMap.push_back(std::move(sampledConfigPtr));

			// Directly find and add neighbors without calling a separate function
			for (size_t i = 0; i < configMap.size() - 1; ++i) { // Exclude the last added config
				double distance = prmInstance.calculateDistance(configMap[configCounter].get(), configMap[i].get());
				if (distance < neighborhood_size) { // Check if within neighborhood
					// Check if the path is valid
					if (prmInstance.validatePath(configMap[configCounter].get(), configMap[i].get(), samplingSteps, map, x_size, y_size)) {
						// Avoid duplicate entries
						if (std::find(connectionGraph[configCounter].begin(), connectionGraph[configCounter].end(), i) == connectionGraph[configCounter].end()) {
							connectionGraph[configCounter].push_back(i);
							connectionGraph[i].push_back(configCounter);
						}
					}
				}
			}
			++configCounter;
		}
	}

    // Connect the closest configurations for start and goal
    int startNode = configCounter;
    int goalNode = configCounter + 1;
    prmInstance.linkClosestNode(armstart_anglesV_rad, connectionGraph, configMap, startNode);
    prmInstance.linkClosestNode(armgoal_anglesV_rad, connectionGraph, configMap, goalNode);

    // Perform A* search to locate a path from start to goal
    std::vector<int> finalPath = prmInstance.aStarSearch(startNode, goalNode, connectionGraph, configMap);

    // Convert the path to the required format
    std::vector<std::vector<double>> planVector;
    if (!finalPath.empty()) {
        planVector.reserve(finalPath.size());
        for (int idx : finalPath) {
            planVector.push_back(std::vector<double>(configMap[idx].get(), configMap[idx].get() + numofDOFs));
        }
        *planlength = planVector.size();

        // Convert std::vector to double** (if required)
        *plan = new double*[*planlength];
        for (int i = 0; i < *planlength; ++i) {
            (*plan)[i] = new double[numofDOFs];
            std::copy(planVector[i].begin(), planVector[i].end(), (*plan)[i]);
        }
    } else {
        std::cout << "Path not found." << std::endl;
    }

    // Final metrics output
    auto finishTime = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::milliseconds>(finishTime - startTimer);

    std::cout << "PRM Planning Duration: " << totalDuration.count() << " ms" << std::endl;
}


//*******************************************************************************************************************//
//                                                                                                                   //
//                                                MAIN FUNCTION                                                      //
//                                                                                                                   //
//*******************************************************************************************************************//

/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

    std::string mapDirPath = MAPS_DIR;
    std::string mapFilePath = mapDirPath + "/" + argv[1];
    std::cout << "Reading problem definition from: " << mapFilePath << std::endl;
	tie(map, x_size, y_size) = loadMap(mapFilePath);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);

    std::string outputDir = OUTPUT_DIR;
	string outputFile = outputDir + "/" + argv[6];
	std::cout << "Writing solution to: " << outputFile << std::endl;

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.

	double** plan = NULL;
	int planlength = 0;
    // // Generate 20 valid start and goal configurations
    // int numPairs = 20;
    // std::vector<std::vector<double>> startConfigs = generateValidConfigurations(numPairs, numOfDOFs, map, x_size, y_size);
    // std::vector<std::vector<double>> goalConfigs = generateValidConfigurations(numPairs, numOfDOFs, map, x_size, y_size);

    // // Display the generated pairs
    // std::cout << "Generated Start-Goal Pairs for Map2:" << std::endl;
    // for (int i = 0; i < numPairs; ++i) {
    //     std::cout << "Pair " << (i + 1) << ":\n";

    //     // Print start configuration
    //     std::cout << "  Start: ";
    //     for (const auto& angle : startConfigs[i]) {
    //         std::cout << std::fixed << std::setprecision(4) << angle << " ";
    //     }
        
    //     // Print goal configuration
    //     std::cout << "\n  Goal: ";
    //     for (const auto& angle : goalConfigs[i]) {
    //         std::cout << std::fixed << std::setprecision(4) << angle << " ";
    //     }

    //     std::cout << "\n" << std::endl;
    // }

    // Call the corresponding planner function
    if (whichPlanner == PRM)
    {
        plannerPRM(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRT)
    {
        plannerRRT(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTCONNECT)
    {
        plannerRRTConnect(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else if (whichPlanner == RRTSTAR)
    {
        plannerRRTStar(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }
    else
    {
        planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
    }

	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << mapFilePath << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}


