import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

plannerList = ["RRT", "RRTCONNECT", "RRTSTAR", "PRM"]

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    problems = [
                ["map2.txt", "1.7135,2.7421,4.8160", "1.0038,0.2993,4.1594"],
                ["map2.txt", "1.7410,5.7417,3.3285", "0.7851,5.9637,3.0422"],
                ["map2.txt", "1.4652,1.9247,2.2055", "1.0768,2.3406,5.9045"],
                ["map2.txt", "1.8046,1.1205,0.9659", "0.7782,3.3071,1.0058"],
                ["map2.txt", "0.5717,1.7253,0.0188", "1.5315,4.4709,1.6566"],
                ["map2.txt", "1.2551,2.0059,3.9538", "0.0203,1.6721,4.1764"],
                ["map2.txt", "0.4465,2.2955,1.5900", "0.2849,0.2603,0.9633"],
                ["map2.txt", "1.7728,1.2187,0.0711", "0.9324,6.2681,4.0182"],
                ["map2.txt", "1.2053,6.1779,1.5334", "0.8841,5.7876,1.3927"],
                ["map2.txt", "0.9911,6.2074,1.6158", "1.5403,0.6734,2.4063"],
                ["map2.txt", "1.4677,0.6386,1.3786", "1.0192,1.0027,0.3627"],
                ["map2.txt", "1.4263,2.0029,4.3972", "0.9378,2.9412,2.3914"],
                ["map2.txt", "1.6264,2.3262,2.4694", "0.7649,0.4827,0.9004"],
                ["map2.txt", "1.7548,0.4917,2.3232", "1.2334,2.1307,3.6171"],
                ["map2.txt", "1.6752,6.0952,1.5503", "0.9104,1.8317,4.1900"],
                ["map2.txt", "1.0755,0.0267,3.1145", "1.6596,1.8284,5.2380"],
                ["map2.txt", "1.3561,2.4239,3.9387", "1.8412,1.1385,2.2464"],
                ["map2.txt", "0.3611,6.2008,3.4080", "1.2232,6.0804,3.2866"],
                ["map2.txt", "1.1830,2.2074,3.8587", "1.7384,5.9800,0.7366"],
                ["map2.txt", "0.7324,1.3167,0.7142", "1.5282,5.9152,3.7493"]]
    scores = []
    for aPlanner in [0, 1, 2, 3]:
        print("\nTESTING " + plannerList[aPlanner] + "\n")
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "../output/grader_out/tmp.txt"
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPos, goalPos,
                aPlanner, outputSolutionFile)
            print("EXECUTING: " + str(commandPlan))
            commandVerify = "../build/verifier {} {} {} {} {}".format(
                inputMap, numDOFs, startPos, goalPos,
                outputSolutionFile)
            print("EXECUTING: " + str(commandVerify))
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py ../output/grader_out/tmp.txt --gifFilepath=../output/grader_out/grader_{}{}.gif".format(plannerList[aPlanner], i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./../build/planner", "../output/grader_out/grader_results.csv")