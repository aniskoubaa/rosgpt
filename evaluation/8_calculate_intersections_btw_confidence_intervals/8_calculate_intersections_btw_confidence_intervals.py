import pandas as pd
from sympy import Interval

# Read the first CSV file
file1 = pd.read_csv('/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_SEM_human_score.txt', header=None, names=['Model', 'Case', 'Interval'])
file1[['avg', 'confid']] = file1['Interval'].str.extract(r'(\S+) \\pm (\S+)').astype(float)
print(file1[['avg', 'confid']])

# Read the second CSV file
file2 = pd.read_csv('/home/riotu/Dropbox/Adel/ChatGPT/ROSGPT/evaluations/average_and_SEM_GPT4_score.txt', header=None, names=['Model', 'Case', 'Interval'])
file2[['avg', 'confid']] = file2['Interval'].str.extract(r'(\S+) \\pm (\S+)').astype(float)
print(file2[['avg', 'confid']])

# Initialize a counter for intersections
intersection_count = 0

# Iterate through each index in both files
for index in range(len(file1)):
    # Check if intervals intersect
    interval1 = Interval(file1.at[index, 'avg'] - file1.at[index, 'confid'], file1.at[index, 'avg'] + file1.at[index, 'confid'])
    interval2 = Interval(file2.at[index, 'avg'] - file2.at[index, 'confid'], file2.at[index, 'avg'] + file2.at[index, 'confid'])

    print(interval1,interval2)

    if interval1.intersect(interval2):
        print(f"Intersection: {file1.at[index, 'Model']} - {file1.at[index, 'Case']} and {file2.at[index, 'Model']} - {file2.at[index, 'Case']}")
        intersection_count += 1

# Print the total number of intersections
print(f"Total number of intersections: {intersection_count}")
