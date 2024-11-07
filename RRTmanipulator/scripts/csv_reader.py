import pandas as pd

# Load the CSV file
df = pd.read_csv("../output/grader_out/grader_results.csv")

# Group by planner and calculate summary statistics
summary = df.groupby("planner").agg(
    Avg_NumSteps=("numSteps", "mean"),
    Std_NumSteps=("numSteps", "std"),
    Avg_Cost=("cost", "mean"),
    Std_Cost=("cost", "std"),
    Avg_TimeSpent=("timespent", "mean"),
    Std_TimeSpent=("timespent", "std"),
    SuccessRate=("success", "mean"),
    Std_SuccessRate=("success", "std")
)

# Convert success rate to percentage
summary["SuccessRate"] *= 100
summary["Std_SuccessRate"] *= 100

# Display the summary
print(summary)